// powermon -- electricity-meter pulse counter -> MQTT -> Home Assistant.
//
// Replaces "Duco RF Gateway + Power Monitor v1.0", whose source was lost. The
// old firmware died because its broker address was a compile-time constant that
// went stale; everything about the config here is runtime-resolvable so that
// cannot happen twice.

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <LittleFS.h>
#include <PubSubClient.h>
#include <time.h>

#include "pulse.h"
#include "tm1637.h"

static const char kVersion[] = "powermon 2.0";
static const uint8_t kPinClk = D3;
static const uint8_t kPinDio = D4;

// ---------------------------------------------------------------- config ----

struct Config {
  // WiFi credentials live here, in LittleFS, rather than in the source. The
  // firmware this replaced hardcoded them, which is why a reflash knocked the
  // device off the network -- there was nothing persisted to fall back on.
  char wifiSsid[33] = "";
  char wifiPass[65] = "";
  char mqttHost[64] = "nl-pi.local";  // mDNS by default: survives a re-addressing
  uint16_t mqttPort = 1883;
  char mqttUser[32] = "powermon";
  char mqttPass[64] = "";
  char prefix[32] = "power";
  char nodeId[32] = "powermon";
  uint32_t pulsesPerKwh = 10000;  // 0.1 Wh/pulse
  double energyOffsetKwh = 0;     // aligns the counter with the meter's own reading
} cfg;

static const char kCfgPath[] = "/cfg.json";

// A hand-rolled reader rather than ArduinoJson: the schema is fixed and tiny,
// and this keeps the dependency list to PubSubClient alone.
static bool cfgLoad() {
  File f = LittleFS.open(kCfgPath, "r");
  if (!f) return false;
  String s = f.readString();
  f.close();
  // Tolerate whitespace around the colon: this file is written both by cfgSave()
  // (compact) and by whatever tool provisions it at flash time (often pretty-
  // printed). A parser that only accepted one of those silently produced a
  // config full of defaults.
  auto valueAt = [&](const char* key) -> int {
    int i = s.indexOf(String("\"") + key + "\"");
    if (i < 0) return -1;
    i += strlen(key) + 2;
    while (i < (int)s.length() && (s[i] == ' ' || s[i] == '\t')) i++;
    if (i >= (int)s.length() || s[i] != ':') return -1;
    i++;
    while (i < (int)s.length() && (s[i] == ' ' || s[i] == '\t')) i++;
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
  double d;
  d = cfg.mqttPort;        num("mqttPort", d);        cfg.mqttPort = (uint16_t)d;
  d = cfg.pulsesPerKwh;    num("pulsesPerKwh", d);    cfg.pulsesPerKwh = (uint32_t)d;
  d = cfg.energyOffsetKwh; num("energyOffsetKwh", d); cfg.energyOffsetKwh = d;
  return true;
}

static void cfgSave() {
  File f = LittleFS.open(kCfgPath, "w");
  if (!f) return;
  f.printf(
      "{\"wifiSsid\":\"%s\",\"wifiPass\":\"%s\","
      "\"mqttHost\":\"%s\",\"mqttPort\":%u,\"mqttUser\":\"%s\",\"mqttPass\":\"%s\","
      "\"prefix\":\"%s\",\"nodeId\":\"%s\",\"pulsesPerKwh\":%u,\"energyOffsetKwh\":%.4f}",
      cfg.wifiSsid, cfg.wifiPass, cfg.mqttHost, cfg.mqttPort, cfg.mqttUser,
      cfg.mqttPass, cfg.prefix, cfg.nodeId, cfg.pulsesPerKwh, cfg.energyOffsetKwh);
  f.close();
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

// Flash has a finite erase count, so the running total is only written when it
// has moved by a full kWh-hundredth or once every 10 minutes -- whichever comes
// first. A power cut loses at most that much.
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
ESP8266WebServer web(80);

static uint32_t gWindowPulses = 0;
static uint32_t gWindowStartMs = 0;
static uint32_t gLastPulseMs = 0;
static bool gHaveReading = false;
// Pulses in the first seconds after boot are not trusted: the detector is still
// learning the baseline, and the sensor may still be being handled.
static const uint32_t kWarmupMs = 30000;
static float gWatts = 0;
static char gTopicState[96], gTopicAvail[96];

// The 802.11 reason code is the one piece of evidence that separates "the AP
// kicked us" (8 = assoc leave, band steering) from "we stopped hearing it"
// (200 = beacon timeout, the signature of RF desense) from a key-exchange
// failure (15 = 4-way handshake timeout, WPA3/PMF interop). WiFi.status() alone
// cannot tell these apart.
static WiFiEventHandler gOnDisconnect;

// Averaging over the publish window rather than reporting 1/interval: a single
// interval is noisy at low load, and at 0.1 Wh/pulse a 60 W lamp only produces a
// pulse every six seconds.
// The display sits in the bedroom's line of sight, so it goes dark from 00:30 to
// 07:30 local time. Until SNTP has set the clock the display stays on: a blank
// display with no known time would look like a dead device.
static bool isNight() {
  time_t t = time(nullptr);
  if (t < 1700000000) return false;  // clock not set yet
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

  // With no pulse this window, power is not zero -- it is merely below what one
  // pulse in the time since the last one would imply. Decay toward that bound
  // instead of dropping to zero and flickering back up on the next pulse.
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

// ------------------------------------------------------------------ mqtt ----

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
    // Build the optional fragments first. Inlining String(...).c_str() into the
    // snprintf argument list would hand it a pointer to a temporary that is
    // already destroyed by the time snprintf reads it.
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

static bool mqttConnect() {
  snprintf(gTopicState, sizeof gTopicState, "%s/%s/state", cfg.prefix, cfg.nodeId);
  snprintf(gTopicAvail, sizeof gTopicAvail, "%s/%s/availability", cfg.prefix, cfg.nodeId);

  mqtt.setServer(cfg.mqttHost, cfg.mqttPort);
  mqtt.setBufferSize(MQTT_MAX_PACKET_SIZE);
  // PubSubClient defaults to a 15s keepalive, which the broker enforces at
  // 1.5x. Anything that stalls loop() for twenty-odd seconds -- a slow DNS
  // lookup, a burst of ADC work, an OTA poll -- then costs a full reconnect.
  // 60s leaves real slack without meaningfully delaying dead-peer detection.
  mqtt.setKeepAlive(60);
  mqtt.setSocketTimeout(10);

  bool ok = cfg.mqttUser[0]
                ? mqtt.connect(cfg.nodeId, cfg.mqttUser, cfg.mqttPass, gTopicAvail, 0, true, "offline")
                : mqtt.connect(cfg.nodeId, gTopicAvail, 0, true, "offline");
  if (!ok) {
    // rc=-2 is a TCP-level failure (wrong host, or nothing listening); rc=5 is
    // the broker rejecting the credentials. The old firmware showed both in turn.
    Serial.printf("[MQTT] connect to %s:%u failed rc=%d\n", cfg.mqttHost, cfg.mqttPort,
                  mqtt.state());
    return false;
  }
  mqtt.publish(gTopicAvail, "online", true);
  publishDiscovery();
  Serial.printf("[MQTT] connected to %s:%u\n", cfg.mqttHost, cfg.mqttPort);
  return true;
}

static void publishState() {
  char payload[192];
  snprintf(payload, sizeof payload,
           "{\"watts\":%.1f,\"total_kwh\":%.4f,\"swing\":%d,\"baseline\":%d,"
           "\"rssi\":%d,\"uptime_s\":%lu}",
           gWatts, totalKwh(), detector.swing(), detector.baseline(), WiFi.RSSI(),
           (unsigned long)(millis() / 1000UL));
  mqtt.publish(gTopicState, payload);
}

// ------------------------------------------------------------------- web ----

static void webRoot() {
  char buf[1700];
  snprintf(buf, sizeof buf,
           "<!doctype html><meta name=viewport content='width=device-width,initial-scale=1'>"
           "<style>body{font:15px system-ui;margin:2rem;max-width:34rem}"
           "input{width:100%%;padding:.4rem;margin:.2rem 0 .8rem}b{font-size:1.6rem}</style>"
           "<h2>%s</h2><p><b>%.1f W</b> &middot; %.4f kWh"
           "<br><small>signal swing %d counts (baseline %d) &middot; RSSI %d dBm</small></p>"
           "<form method=post action=/save>"
           "WiFi SSID<input name=s value='%s'>"
           "WiFi password<input name=q type=password placeholder='(unchanged)'>"
           "MQTT host<input name=h value='%s'>Port<input name=p value='%u'>"
           "User<input name=u value='%s'>Password<input name=w type=password placeholder='(unchanged)'>"
           "Topic prefix<input name=x value='%s'>Pulses per kWh<input name=k value='%u'>"
           "Energy offset kWh<input name=e value='%.4f'>"
           "<button>Save &amp; reboot</button></form>",
           kVersion, gWatts, totalKwh(), detector.swing(), detector.baseline(), WiFi.RSSI(),
           cfg.wifiSsid, cfg.mqttHost, cfg.mqttPort, cfg.mqttUser, cfg.prefix, cfg.pulsesPerKwh,
           cfg.energyOffsetKwh);
  web.send(200, "text/html", buf);
}

static void webSave() {
  auto copy = [&](const char* arg, char* dst, size_t n) {
    if (!web.hasArg(arg)) return;
    strncpy(dst, web.arg(arg).c_str(), n - 1);
    dst[n - 1] = 0;
  };
  copy("s", cfg.wifiSsid, sizeof cfg.wifiSsid);
  copy("h", cfg.mqttHost, sizeof cfg.mqttHost);
  copy("u", cfg.mqttUser, sizeof cfg.mqttUser);
  copy("x", cfg.prefix, sizeof cfg.prefix);
  if (web.hasArg("p")) cfg.mqttPort = web.arg("p").toInt();
  if (web.hasArg("k") && web.arg("k").toInt() > 0) cfg.pulsesPerKwh = web.arg("k").toInt();
  // Changing the offset re-bases the counter, so the pulse tally restarts.
  if (web.hasArg("e")) {
    double v = web.arg("e").toDouble();
    if (v != cfg.energyOffsetKwh) {
      cfg.energyOffsetKwh = v;
      gPulses = 0;
      energyPersist(true);
    }
  }
  // An empty password field means "leave it alone" -- otherwise every save
  // through the form would wipe the stored credential.
  if (web.hasArg("w") && web.arg("w").length()) copy("w", cfg.mqttPass, sizeof cfg.mqttPass);
  if (web.hasArg("q") && web.arg("q").length()) copy("q", cfg.wifiPass, sizeof cfg.wifiPass);
  cfgSave();
  web.send(200, "text/html", "<meta http-equiv=refresh content='3;url=/'>saved, rebooting");
  delay(300);
  ESP.restart();
}

// ------------------------------------------------------------------ main ----

void setup() {
  Serial.begin(115200);
  Serial.printf("\n[BOOT] %s\n", kVersion);

  if (!LittleFS.begin()) Serial.println(F("[BOOT] LittleFS mount FAILED"));
  Serial.printf("[BOOT] config %s\n", cfgLoad() ? "loaded" : "MISSING (using defaults)");
  energyLoad();

  display.begin();
  display.clear();

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  // Modem sleep is the ESP8266 default and is a well-known cause of a station
  // that keeps running but stops answering: the radio naps between beacons, the
  // AP ages it out, and nothing in the sketch notices. This node is mains
  // powered, so there is nothing to save.
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  gOnDisconnect = WiFi.onStationModeDisconnected(
      [](const WiFiEventStationModeDisconnected& e) {
        Serial.printf("[WiFi] deauth reason=%d\n", (int)e.reason);
      });
  if (cfg.wifiSsid[0]) {
    Serial.printf("[WiFi] ssid=%s from config\n", cfg.wifiSsid);
    WiFi.begin(cfg.wifiSsid, cfg.wifiPass);
  } else {
    Serial.println(F("[WiFi] no ssid in config, trying the SDK's stored one"));
    WiFi.begin();
  }
  Serial.print(F("[WiFi] connecting"));
  for (int i = 0; i < 120 && WiFi.status() != WL_CONNECTED; i++) {
    delay(250);
    Serial.print('.');
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[WiFi] up: %s\n", WiFi.localIP().toString().c_str());
  }
  // There is deliberately no recovery SoftAP. WIFI_AP_STA time-slices the one
  // radio between the AP and the station, which costs the station link exactly
  // when it is already struggling. Reconfigure over USB instead.

  // Local time is only needed for the display's night blanking. The SNTP client
  // retries on its own, so this is fire-and-forget even if WiFi is not up yet.
  configTime("CET-1CEST,M3.5.0,M10.5.0/3", "pool.ntp.org", "time.google.com");

  MDNS.begin(cfg.nodeId);
  ArduinoOTA.setHostname(cfg.nodeId);
  ArduinoOTA.begin();

  web.on("/", webRoot);
  web.on("/save", HTTP_POST, webSave);
  web.begin();

  detector.begin(analogRead(A0));
  gWindowStartMs = gLastPulseMs = millis();
  Serial.println(F("[BOOT] ready"));
}

void loop() {
  ArduinoOTA.handle();
  web.handleClient();
  MDNS.update();

  // The ADC is polled every 5 ms, not every 1 ms.
  //
  // The ESP8266's single SAR ADC is shared between pin A0 and the RF front end's
  // own calibration, and analogRead() masks interrupts for ~100us. Hammering it
  // at 1 kHz desensitises the receiver enough to miss beacons, which shows up as
  // an association that drops ~20s after it forms and then cannot find the SSID
  // again. A meter LED flash is tens of milliseconds wide, so 5 ms still samples
  // it many times over. The yield() hands the SDK its slice back explicitly.
  static uint32_t lastSampleMs = 0;
  uint32_t now = millis();
  if (now - lastSampleMs >= 5) {
    lastSampleMs = now;
    int raw = analogRead(A0);
    yield();
    if (detector.update(raw, now) && now >= kWarmupMs) {
      gPulses += 1;
      gWindowPulses++;
      gLastPulseMs = now;
    }
  }

  // Nothing counted before kWarmupMs belongs in the first window.
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
    // No state until a real pulse has been measured: a reading taken before
    // then is either the warm-up or a sensor that is not on the meter, and
    // both would land in Home Assistant as a plausible-looking wrong number.
    if (isNight()) {
      display.clear();
      if (gHaveReading && mqtt.connected()) publishState();
      if (gHaveReading) energyPersist(false);
    } else if (!gHaveReading) {
      display.showNumber(-1);
      Serial.printf("[POWER] waiting for first pulse, swing=%d\n", detector.swing());
    } else {
      // The module has three digits, so at or above 1 kW it switches to
      // kilowatts with a decimal point ("1.23", "12.3").
      int roundedWatts = (int)(gWatts + 0.5f);
      if (roundedWatts > 999) display.showKilowatts(roundedWatts);
      else display.showNumber(roundedWatts);
      if (mqtt.connected()) publishState();
      energyPersist(false);
      Serial.printf("[POWER] watts=%.1f total=%.4f kWh swing=%d\n", gWatts, totalKwh(),
                    detector.swing());
    }
  }

  // Log every WiFi transition. Without this a dropped station looks exactly like
  // a broker problem: the sketch keeps printing power readings while nothing can
  // reach it. That cost an hour once.
  static wl_status_t lastWifi = WL_IDLE_STATUS;
  wl_status_t wifiNow = WiFi.status();
  if (wifiNow != lastWifi) {
    lastWifi = wifiNow;
    if (wifiNow == WL_CONNECTED) {
      Serial.printf("[WiFi] up: %s rssi=%d\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());
    } else {
      Serial.printf("[WiFi] lost, status=%d\n", (int)wifiNow);
    }
  }

  if (wifiNow == WL_CONNECTED) {
    // Log the MQTT state on the way down, not just on the way up: a reconnect
    // loop looks identical in the log to a healthy first connect otherwise.
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
    // setAutoReconnect does not always take, so drive it explicitly.
    static uint32_t lastWifiTryMs = 0;
    if (now - lastWifiTryMs >= 20000) {
      lastWifiTryMs = now;
      Serial.println(F("[WiFi] reconnecting"));
      if (cfg.wifiSsid[0]) WiFi.begin(cfg.wifiSsid, cfg.wifiPass); else WiFi.reconnect();
    }
  }
}
