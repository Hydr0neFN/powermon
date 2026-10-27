// powermon on ESP32-C3 -- electricity-meter pulse counter -> MQTT -> Home Assistant.
//
// Port of src/main.cpp (ESP8266). Differences: analogReadMilliVolts() on GPIO3,
// no web server (config is data/config.json via `uploadfs`, firmware updates via
// ArduinoOTA), plus the BLE presence scanner as node "kitchen" and a 1 Hz
// wifi_rssi topic. MQTT topics, JSON fields and HA discovery are unchanged.

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <LittleFS.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <esp_system.h>
#include <time.h>

#include "ble_presence.h"
#include "csi.h"
#include "pins.h"
#include "pulse.h"
#include "tm1637.h"

static const char kVersion[] = "powermon 3.1 c3 csi";

// ---------------------------------------------------------------- config ----

struct Config {
  char wifiSsid[33] = "";
  char wifiPass[65] = "";
  char mqttHost[64] = "nl-pi.local";  // mDNS by default: survives a re-addressing
  uint16_t mqttPort = 1883;
  char mqttUser[32] = "powermon";
  char mqttPass[64] = "";
  char prefix[32] = "power";
  char nodeId[32] = "powermon";
  char otaPass[48] = "";             // empty = OTA disabled (never unauthenticated)
  uint32_t pulsesPerKwh = 10000;     // 0.1 Wh/pulse
  double energyOffsetKwh = 0;        // aligns the counter with the meter's own reading
  double adcAttenDb = 11;            // 0, 2.5, 6 or 11 (see README "ADC tuning")
  int adcOversample = 8;             // ADC reads averaged per 5 ms sample
  int minSwingMv = kDefaultMinSwingMv;
  int armSwingMv = kDefaultArmSwingMv;
  int maxWatts = 12000;              // a window above this is sensor chatter, not the meter: discarded
  int maxPulseMs = 1000;             // longer than this above threshold = ambient light, not a flash
  int smoothSamples = 4;             // boxcar over this many 5 ms samples (4 = 20 ms = one 50 Hz cycle)
  int bleEnabled = 1;
  int wifiTxQdbm = 0;                // WiFi.setTxPower units (0.25 dBm); 0 = default
} cfg;

static const char kCfgPath[] = "/config.json";

// Hand-rolled reader, same as the ESP8266 build: fixed tiny schema, tolerant of
// whitespace so a pretty-printed file from `uploadfs` parses.
static bool cfgLoad() {
  File f = LittleFS.open(kCfgPath, "r");
  if (!f) return false;
  String s = f.readString();
  f.close();
  auto isWs = [](char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n'; };
  auto valueAt = [&](const char* key) -> int {
    int i = s.indexOf(String("\"") + key + "\"");
    if (i < 0) return -1;
    i += strlen(key) + 2;
    while (i < (int)s.length() && isWs(s[i])) i++;
    if (i >= (int)s.length() || s[i] != ':') return -1;
    i++;
    while (i < (int)s.length() && isWs(s[i])) i++;
    return i;
  };
  auto str = [&](const char* key, char* dst, size_t n) {
    int i = valueAt(key);
    if (i < 0 || s[i] != '"') return;
    i++;
    int j = s.indexOf('"', i);
    if (j < 0) return;
    String v = s.substring(i, j);
    strncpy(dst, v.c_str(), n - 1);
    dst[n - 1] = 0;
  };
  auto num = [&](const char* key, double& dst) {
    int i = valueAt(key);
    if (i < 0) return;
    dst = s.substring(i).toDouble();
  };
  str("wifiSsid", cfg.wifiSsid, sizeof cfg.wifiSsid);
  str("wifiPass", cfg.wifiPass, sizeof cfg.wifiPass);
  str("mqttHost", cfg.mqttHost, sizeof cfg.mqttHost);
  str("mqttUser", cfg.mqttUser, sizeof cfg.mqttUser);
  str("mqttPass", cfg.mqttPass, sizeof cfg.mqttPass);
  str("prefix", cfg.prefix, sizeof cfg.prefix);
  str("nodeId", cfg.nodeId, sizeof cfg.nodeId);
  str("otaPassword", cfg.otaPass, sizeof cfg.otaPass);
  double d;
  d = cfg.mqttPort;        num("mqttPort", d);        cfg.mqttPort = (uint16_t)d;
  d = cfg.pulsesPerKwh;    num("pulsesPerKwh", d);    if (d > 0) cfg.pulsesPerKwh = (uint32_t)d;
  d = cfg.energyOffsetKwh; num("energyOffsetKwh", d); cfg.energyOffsetKwh = d;
  d = cfg.adcAttenDb;      num("adcAttenDb", d);      cfg.adcAttenDb = d;
  d = cfg.adcOversample;   num("adcOversample", d);   if (d >= 1 && d <= 64) cfg.adcOversample = (int)d;
  d = cfg.minSwingMv;      num("minSwingMv", d);      cfg.minSwingMv = (int)d;
  d = cfg.armSwingMv;      num("armSwingMv", d);      cfg.armSwingMv = (int)d;
  d = cfg.maxWatts;        num("maxWatts", d);        cfg.maxWatts = (int)d;
  d = cfg.maxPulseMs;      num("maxPulseMs", d);      cfg.maxPulseMs = (int)d;
  d = cfg.smoothSamples;   num("smoothSamples", d);   if (d >= 1 && d <= 16) cfg.smoothSamples = (int)d;
  d = cfg.bleEnabled;      num("bleEnabled", d);      cfg.bleEnabled = (int)d;
  d = cfg.wifiTxQdbm;      num("wifiTxQdbm", d);      cfg.wifiTxQdbm = (int)d;
  return true;
}

// ---------------------------------------------------------------- energy ----

static const char kEnergyPath[] = "/energy";
static double gPulses = 0;  // since the offset was last set
static uint32_t gLastPersistMs = 0;
static double gLastPersisted = 0;

static void energyLoad() {
  File f = LittleFS.open(kEnergyPath, "r");
  if (!f) return;
  gPulses = f.readString().toDouble();
  f.close();
  gLastPersisted = gPulses;
}

// Flash has a finite erase count: write when the total moved by 0.01 kWh or
// every 10 minutes, whichever comes first. A power cut loses at most that much.
static void energyPersist(bool force) {
  uint32_t now = millis();
  double deltaKwh = (gPulses - gLastPersisted) / (double)cfg.pulsesPerKwh;
  if (gPulses == gLastPersisted) return;
  if (!force && deltaKwh < 0.01 && (now - gLastPersistMs) < 600000UL) return;
  File f = LittleFS.open(kEnergyPath, "w");
  if (!f) return;
  f.print(gPulses, 0);
  f.close();
  gLastPersisted = gPulses;
  gLastPersistMs = now;
}

static double totalKwh() {
  return cfg.energyOffsetKwh + gPulses / (double)cfg.pulsesPerKwh;
}

// ----------------------------------------------------------------- state ----

PulseDetector detector;
TM1637 display(kPinClk, kPinDio);
WiFiClient net;
PubSubClient mqtt(net);

static uint32_t gWindowPulses = 0;
static uint32_t gWindowStartMs = 0;
static uint32_t gLastPulseMs = 0;
static bool gHaveReading = false;
// Pulses in the first seconds after boot are not trusted: the detector is still
// learning the baseline, and the sensor may still be being handled.
static const uint32_t kWarmupMs = 30000;
static float gWatts = 0;
static uint32_t gRejectedWindows = 0;
static char gTopicState[96], gTopicAvail[96], gTopicDiag[96];
static bool gOtaReady = false;
static bool gBleStarted = false;
static const char* gResetWhy = "?";

// Night blanking of the display, 00:30-07:30 local; display stays on until SNTP
// has set the clock (a blank display with unknown time would look dead).
static bool isNight() {
  time_t t = time(nullptr);
  if (t < 1700000000) return false;
  struct tm lt;
  localtime_r(&t, &lt);
  int minutes = lt.tm_hour * 60 + lt.tm_min;
  return minutes >= 30 && minutes < 450;
}

static void recomputeWatts(uint32_t now) {
  uint32_t elapsed = now - gWindowStartMs;
  if (elapsed == 0) return;
  float wh = gWindowPulses * 1000.0f / (float)cfg.pulsesPerKwh;
  float w = wh * 3600000.0f / (float)elapsed;

  // Photodiode off the meter (room light ripple, loose sensor) chatters at tens
  // of pulses per second. No household draws this: un-count the window instead of
  // sending kilowatts and phantom kWh to Home Assistant.
  if (cfg.maxWatts > 0 && w > (float)cfg.maxWatts) {
    gPulses -= gWindowPulses;
    gRejectedWindows++;
    gWatts = 0;
    gWindowPulses = 0;
    gWindowStartMs = now;
    return;
  }

  // No pulse this window: power is not zero, only below what one pulse in the
  // time since the last would imply. Decay toward that bound.
  if (gWindowPulses == 0) {
    uint32_t since = now - gLastPulseMs;
    float bound = since ? (1000.0f / cfg.pulsesPerKwh) * 3600000.0f / (float)since : 0;
    w = (bound < gWatts) ? bound : gWatts;
  }
  gWatts = w;
  if (gWindowPulses > 0) gHaveReading = true;
  gWindowPulses = 0;
  gWindowStartMs = now;
}

// ------------------------------------------------------------------ adc -----

static adc_attenuation_t attenFromDb(double db) {
  if (db < 1.0) return ADC_0db;
  if (db < 4.0) return ADC_2_5db;
  if (db < 8.5) return ADC_6db;
  return ADC_11db;
}

// Mean of a few conversions: the C3 ADC is noisier than the ESP8266's, and the
// detector's arm swing is only a few tens of millivolts.
static int readMv() {
  uint32_t sum = 0;
  for (int i = 0; i < cfg.adcOversample; i++) sum += analogReadMilliVolts(kPinPhotodiode);
  return (int)(sum / cfg.adcOversample);
}

// Boxcar over smoothSamples consecutive 5 ms samples. Lamps on the 50 Hz mains
// flicker at 50/100 Hz; a boxcar that is exactly one mains period long cancels
// it, while a meter LED flash (>= ~30 ms) still comes through. Without it the
// board on a desk counted ~50 phantom pulses per second.
static int smoothMv(int mv) {
  static int ring[16];
  static int n = 0, idx = 0, sum = 0;
  if (n == cfg.smoothSamples) sum -= ring[idx]; else n++;
  ring[idx] = mv;
  sum += mv;
  idx = (idx + 1) % cfg.smoothSamples;
  return sum / n;
}

// ------------------------------------------------------------------ mqtt ----

static bool mqttPublishRaw(const char* topic, const char* payload, bool retain) {
  return mqtt.connected() && mqtt.publish(topic, payload, retain);
}

static void publishDiscovery() {
  char topic[160], payload[512];

  struct Entity {
    const char* id;
    const char* name;
    const char* unit;
    const char* devClass;
    const char* stateClass;
    const char* field;
  };
  static const Entity kEntities[] = {
      {"power",  "Power",         "W",   "power",           "measurement",      "watts"},
      {"energy", "Energy",        "kWh", "energy",          "total_increasing", "total_kwh"},
      {"swing",  "Sensor signal", "",    nullptr,           "measurement",      "swing"},
      {"rssi",   "WiFi signal",   "dBm", "signal_strength", "measurement",      "rssi"},
  };

  for (const Entity& e : kEntities) {
    char unitFrag[48] = "";
    if (e.unit[0]) snprintf(unitFrag, sizeof unitFrag, ",\"unit_of_meas\":\"%s\"", e.unit);
    char classFrag[48] = "";
    if (e.devClass) snprintf(classFrag, sizeof classFrag, ",\"dev_cla\":\"%s\"", e.devClass);

    snprintf(topic, sizeof topic, "homeassistant/sensor/%s_%s/config", cfg.nodeId, e.id);
    int n = snprintf(
        payload, sizeof payload,
        "{\"name\":\"%s\",\"uniq_id\":\"%s_%s\",\"stat_t\":\"%s\",\"avty_t\":\"%s\","
        "\"val_tpl\":\"{{ value_json.%s }}\",\"stat_cla\":\"%s\"%s%s,"
        "\"dev\":{\"ids\":[\"%s\"],\"name\":\"Power monitor\",\"mf\":\"DIY\",\"mdl\":\"%s\"}}",
        e.name, cfg.nodeId, e.id, gTopicState, gTopicAvail, e.field, e.stateClass,
        unitFrag, classFrag, cfg.nodeId, kVersion);
    if (n >= (int)sizeof payload) {
      Serial.printf("[MQTT] WARN: discovery for %s truncated\n", e.id);
      continue;
    }
    mqtt.publish(topic, payload, true);
  }
  Serial.println(F("[MQTT] HA discovery published"));
}

static void mqttOnMessage(char* topic, uint8_t* payload, unsigned int length) {
  // CSI first: it shares presence/<node>/cmd with BLE, which swallows every presence/ topic.
  if (csi::onMessage(topic, payload, length)) return;
  if (blepresence::onMessage(topic, payload, length)) return;
}

static void mqttSubscribeAll() {
  auto sub = [](const char* t) { mqtt.subscribe(t); };
  blepresence::subscribe(sub);
  csi::subscribe(sub);
}

static bool mqttConnect() {
  snprintf(gTopicState, sizeof gTopicState, "%s/%s/state", cfg.prefix, cfg.nodeId);
  snprintf(gTopicAvail, sizeof gTopicAvail, "%s/%s/availability", cfg.prefix, cfg.nodeId);
  snprintf(gTopicDiag, sizeof gTopicDiag, "%s/%s/diag", cfg.prefix, cfg.nodeId);

  mqtt.setServer(cfg.mqttHost, cfg.mqttPort);
  mqtt.setBufferSize(MQTT_MAX_PACKET_SIZE);
  mqtt.setCallback(mqttOnMessage);
  // 60 s keepalive: slack for slow DNS / OTA polls, see the ESP8266 build.
  mqtt.setKeepAlive(60);
  mqtt.setSocketTimeout(10);

  bool ok = cfg.mqttUser[0]
                ? mqtt.connect(cfg.nodeId, cfg.mqttUser, cfg.mqttPass, gTopicAvail, 0, true, "offline")
                : mqtt.connect(cfg.nodeId, gTopicAvail, 0, true, "offline");
  if (!ok) {
    Serial.printf("[MQTT] connect to %s:%u failed rc=%d\n", cfg.mqttHost, cfg.mqttPort,
                  mqtt.state());
    return false;
  }
  mqtt.publish(gTopicAvail, "online", true);
  publishDiscovery();
  mqttSubscribeAll();
  Serial.printf("[MQTT] connected to %s:%u\n", cfg.mqttHost, cfg.mqttPort);
  return true;
}

// Same fields as the ESP8266 build, plus baseline_mv / swing_mv / peak_mv. The
// legacy "swing" and "baseline" now carry millivolts too (they were raw counts).
static void publishState() {
  char payload[256];
  snprintf(payload, sizeof payload,
           "{\"watts\":%.1f,\"total_kwh\":%.4f,\"swing\":%d,\"baseline\":%d,"
           "\"baseline_mv\":%d,\"swing_mv\":%d,\"peak_mv\":%d,"
           "\"rssi\":%d,\"uptime_s\":%lu}",
           gWatts, totalKwh(), detector.swing(), detector.baseline(), detector.baseline(),
           detector.swing(), detector.peak(), WiFi.RSSI(), (unsigned long)(millis() / 1000UL));
  mqtt.publish(gTopicState, payload);
}

// Published every 10 s even before the first pulse (the state topic is not, so
// HA never sees a made-up reading): lets the sensor be tuned and the board
// monitored without a first pulse.
// ADC poll health since the last diag: widest gap between two 5 ms samples and their count
// (should be ~2000 per 10 s). A gap well above 5 ms means loop() was starved (radio, CSI, BLE).
static uint32_t gMaxGapMs = 0, gSamples = 0;

static void publishDiag() {
  char payload[400];
  snprintf(payload, sizeof payload,
           "{\"baseline_mv\":%d,\"swing_mv\":%d,\"peak_mv\":%d,\"atten_db\":%.1f,\"pulses\":%.0f,"
           "\"have_reading\":%d,\"rejected\":%u,\"heap\":%u,\"gap_ms\":%u,\"samples\":%u,\"rst\":\"%s\",\"ver\":\"%s\"}",
           detector.baseline(), detector.swing(), detector.peak(), cfg.adcAttenDb, gPulses,
           gHaveReading ? 1 : 0, (unsigned)gRejectedWindows, (unsigned)ESP.getFreeHeap(), (unsigned)gMaxGapMs, (unsigned)gSamples, gResetWhy, kVersion);
  gMaxGapMs = gSamples = 0;
  mqtt.publish(gTopicDiag, payload);
}

// 1 Hz link quality for the presence work (and the later CSI module).
static void publishWifiRssi() {
  char topic[64], payload[64];
  snprintf(topic, sizeof topic, "presence/%s/wifi_rssi", BLE_NODE_NAME);
  snprintf(payload, sizeof payload, "{\"rssi\":%d,\"ch\":%d,\"up\":%lu}", WiFi.RSSI(),
           (int)WiFi.channel(), (unsigned long)(millis() / 1000UL));
  mqtt.publish(topic, payload);
}

// ------------------------------------------------------------------- ota ----

// Initialised before BLE so a misbehaving scanner can never lock out updates.
// Only possible once WiFi is up (ArduinoOTA starts mDNS), so loop() retries.
static void otaSetup() {
  if (gOtaReady) return;
  if (!cfg.otaPass[0]) {
    Serial.println(F("[OTA] no otaPassword in config.json, OTA disabled"));
    gOtaReady = true;  // do not retry
    return;
  }
  ArduinoOTA.setHostname(cfg.nodeId);
  ArduinoOTA.setPassword(cfg.otaPass);
  ArduinoOTA.onStart([]() {
    blepresence::pause(true);
    csi::pause(true);
    display.showNumber(-1);  // dashes
    Serial.println(F("[OTA] start"));
  });
  ArduinoOTA.onEnd([]() { Serial.println(F("\n[OTA] done, rebooting")); });
  ArduinoOTA.onError([](ota_error_t e) {
    Serial.printf("[OTA] error %u\n", (unsigned)e);
    blepresence::pause(false);
    csi::pause(false);
  });
  ArduinoOTA.begin();
  gOtaReady = true;
  Serial.println(F("[OTA] listening on 3232"));
}

// ------------------------------------------------------------------ main ----

static const char* resetReasonName(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON: return "poweron";
    case ESP_RST_SW: return "sw";
    case ESP_RST_PANIC: return "panic";
    case ESP_RST_INT_WDT: return "int_wdt";
    case ESP_RST_TASK_WDT: return "task_wdt";
    case ESP_RST_WDT: return "wdt";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_DEEPSLEEP: return "deepsleep";
    default: return "other";
  }
}

void setup() {
  Serial.begin(115200);
  // Without a host on the USB port a full CDC buffer must not stall the loop.
  Serial.setTxTimeoutMs(0);
  gResetWhy = resetReasonName(esp_reset_reason());
  Serial.printf("\n[BOOT] %s reset=%s\n", kVersion, gResetWhy);

  if (!LittleFS.begin(true)) Serial.println(F("[BOOT] LittleFS mount FAILED"));
  Serial.printf("[BOOT] config %s\n", cfgLoad() ? "loaded" : "MISSING (using defaults)");
  energyLoad();

  display.begin();
  display.setBrightness(kDisplayBrightness);
  display.clear();

  analogReadResolution(12);
  analogSetPinAttenuation(kPinPhotodiode, attenFromDb(cfg.adcAttenDb));
  detector.configure(cfg.minSwingMv, cfg.armSwingMv);
  detector.setMaxPulseMs(cfg.maxPulseMs > 0 ? (uint32_t)cfg.maxPulseMs : 0);
  Serial.printf("[ADC] GPIO%u atten=%.1f dB, oversample=%d, min/arm swing %d/%d mV\n",
                kPinPhotodiode, cfg.adcAttenDb, cfg.adcOversample, cfg.minSwingMv,
                cfg.armSwingMv);

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.setHostname(cfg.nodeId);
  // Modem sleep stays on (WIFI_PS_MIN_MODEM): the WiFi/BLE coexistence code
  // aborts when the BLE controller starts with power save off.
  WiFi.setSleep(WIFI_PS_MIN_MODEM);
  if (cfg.wifiTxQdbm > 0) WiFi.setTxPower((wifi_power_t)cfg.wifiTxQdbm);
  // The reason code separates "AP kicked us" (8), "stopped hearing it" (200,
  // beacon timeout) and a key-exchange failure (15).
  WiFi.onEvent([](arduino_event_id_t, arduino_event_info_t info) {
    Serial.printf("[WiFi] disconnected reason=%d\n", (int)info.wifi_sta_disconnected.reason);
  }, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  if (cfg.wifiSsid[0]) {
    Serial.printf("[WiFi] ssid=%s from config\n", cfg.wifiSsid);
    WiFi.begin(cfg.wifiSsid, cfg.wifiPass);
  } else {
    Serial.println(F("[WiFi] no ssid in config, trying the stored one"));
    WiFi.begin();
  }
  Serial.print(F("[WiFi] connecting"));
  for (int i = 0; i < 120 && WiFi.status() != WL_CONNECTED; i++) {
    delay(250);
    Serial.print('.');
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED)
    Serial.printf("[WiFi] up: %s\n", WiFi.localIP().toString().c_str());
  // No recovery SoftAP, by design: reconfigure with `uploadfs`.

  configTzTime("CET-1CEST,M3.5.0,M10.5.0/3", "pool.ntp.org", "time.google.com");

  if (WiFi.status() == WL_CONNECTED) otaSetup();  // before BLE

  detector.begin(smoothMv(readMv()));
  gWindowStartMs = gLastPulseMs = millis();
  Serial.println(F("[BOOT] ready"));
}

void loop() {
  uint32_t now = millis();
  bool wifiUp = WiFi.status() == WL_CONNECTED;

  if (wifiUp && !gOtaReady) otaSetup();
  if (gOtaReady) ArduinoOTA.handle();

  // Started once, after OTA is up. The module itself skips the scanner for one
  // boot after a panic / watchdog reset and when the heap is short.
  if (wifiUp && gOtaReady && !gBleStarted) {
    gBleStarted = true;
    if (cfg.bleEnabled) blepresence::setup(mqttPublishRaw);
    csi::setup(mqttPublishRaw, BLE_NODE_NAME);
  }
  if (gBleStarted) {
    blepresence::loop();
    csi::loop();
  }

  // 5 ms polling: a meter LED flash is tens of ms wide, so this samples it many
  // times over and leaves the radio its share of the single core.
  static uint32_t lastSampleMs = 0;
  if (now - lastSampleMs >= 5) {
    if (now - lastSampleMs > gMaxGapMs) gMaxGapMs = now - lastSampleMs;
    gSamples++;
    lastSampleMs = now;
    int mv = smoothMv(readMv());
    if (detector.update(mv, now) && now >= kWarmupMs) {
      gPulses += 1;
      gWindowPulses++;
      gLastPulseMs = now;
    }
  }

  static bool warm = false;
  if (!warm && now >= kWarmupMs) {
    warm = true;
    gWindowPulses = 0;
    gWindowStartMs = gLastPulseMs = now;
  }

  static uint32_t lastPublishMs = 0;
  if (now - lastPublishMs >= 10000) {
    lastPublishMs = now;
    recomputeWatts(now);
    if (isNight()) {
      display.clear();
      if (gHaveReading && mqtt.connected()) publishState();
      if (gHaveReading) energyPersist(false);
    } else if (!gHaveReading) {
      display.showNumber(-1);
      Serial.printf("[POWER] waiting for first pulse, base=%d peak=%d swing=%d mV\n",
                    detector.baseline(), detector.peak(), detector.swing());
    } else {
      // Three digits: at or above 1 kW show kilowatts with a decimal point.
      int roundedWatts = (int)(gWatts + 0.5f);
      if (roundedWatts > 999) display.showKilowatts(roundedWatts);
      else display.showNumber(roundedWatts);
      if (mqtt.connected()) publishState();
      energyPersist(false);
      Serial.printf("[POWER] watts=%.1f total=%.4f kWh swing=%d mV\n", gWatts, totalKwh(),
                    detector.swing());
    }
    if (mqtt.connected()) publishDiag();
  }

  static uint32_t lastRssiMs = 0;
  if (now - lastRssiMs >= 1000) {
    lastRssiMs = now;
    if (wifiUp && mqtt.connected()) publishWifiRssi();
  }

  static bool lastWifiUp = false;
  if (wifiUp != lastWifiUp) {
    lastWifiUp = wifiUp;
    if (wifiUp) Serial.printf("[WiFi] up: %s rssi=%d\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());
    else Serial.println(F("[WiFi] lost"));
  }

  if (wifiUp) {
    static bool wasMqttUp = false;
    bool mqttUp = mqtt.connected();
    if (wasMqttUp && !mqttUp) Serial.printf("[MQTT] dropped, state=%d\n", mqtt.state());
    wasMqttUp = mqttUp;

    if (!mqttUp) {
      static uint32_t lastTryMs = 0;
      if (now - lastTryMs >= 5000) {
        lastTryMs = now;
        mqttConnect();
      }
    } else {
      mqtt.loop();
    }
  } else {
    static uint32_t lastWifiTryMs = 0;
    if (now - lastWifiTryMs >= 20000) {
      lastWifiTryMs = now;
      Serial.println(F("[WiFi] reconnecting"));
      WiFi.reconnect();
    }
  }
  delay(1);  // yield to the RTOS idle task / WiFi
}
