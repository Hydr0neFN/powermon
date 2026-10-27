# powermon

ESP8266 electricity-meter pulse counter -> MQTT -> Home Assistant.

Replaces the lost `Duco RF Gateway + Power Monitor v1.0` firmware. Rebuilt from
strings recovered out of a flash dump (`duco-esp-flash.bin`, 2026-09-08) plus the
hardware notes below. The CC1101/Duco half is not reimplemented.

## Hardware

| Function            | Pin        | Note |
|---------------------|------------|------|
| Photodiode          | `A0`       | analog: the swing across the meter's LED window is too small for a digital input to latch |
| TM1637 CLK          | `D3` GPIO0 | boot-strap pin, must be high at reset -- TM1637 idles high, so this works |
| TM1637 DIO          | `D4` GPIO2 | boot-strap pin, same reasoning |

Meter constant: **0.1 Wh/pulse** (10000 imp/kWh).

## Why the old one stopped working

The broker address was a compile-time constant, `10.0.0.163`. The broker moved to
`10.0.0.20` and the firmware had no way to be told. This build resolves
`mqtt_host` at runtime and defaults it to an mDNS name, so an address change no
longer requires a reflash.

## Config

Held in LittleFS (`/cfg.json`), editable at `http://<device>/` -- broker host,
port, credentials, topic prefix, pulses-per-kWh, and an energy offset to align
the counter with the physical meter reading.

## Flashing

    pio run -t upload                      # serial, first time
    pio run -t upload -e esp12e_ota        # thereafter

## powermon on ESP32-C3

The ESP-12E was removed from the meter; a **ESP32-C3 SuperMini** replaces it (branch
`c3-port`, envs `c3` and `c3_ota`; `esp12e` and `src/main.cpp` are unchanged). Sources:
`src/c3/` (`pins.h` holds every pin and the assumed mV-per-count constant). There is
**no web server**: config is `data/config.json` (gitignored) pushed with `uploadfs`,
firmware goes over USB or ArduinoOTA.

### Pins

| Function    | Pin    | Note |
|-------------|--------|------|
| Photodiode  | GPIO3  | ADC1_CH3; 3V3 -> photodiode -> node, node -> 100k -> GND |
| TM1637 CLK  | GPIO1  | `LED.C68` 3-digit module; alt GPIO5 |
| TM1637 DIO  | GPIO0  | alt GPIO6 |

Change them in `src/c3/pins.h` only. Brightness 5, display dark 00:30-07:30.

### Toolchain and build

Use an isolated core dir (the default one has other projects' platform pins):

    set PLATFORMIO_CORE_DIR=C:\pio-pm
    pio run -e c3                                   # espressif32@6.9.0 (Arduino 2.0.17), min_spiffs: 2 x 1.9 MB + 128 KB LittleFS
    pio run -e c3 -t upload   --upload-port COM5    # first flash; upload_speed 115200 on purpose (flaky USB)
    pio run -e c3 -t uploadfs --upload-port COM5    # data/config.json -> LittleFS

If the port is gone: hold BOOT while plugging in (download mode), flash, then unplug and
power it **without** BOOT held (otherwise it stays in download mode).

### config.json (gitignored, `data/config.json`)

`wifiSsid wifiPass mqttHost(nl-pi.local) mqttPort mqttUser(powermon) mqttPass prefix(power) nodeId(powermon)
pulsesPerKwh(10000) energyOffsetKwh otaPassword adcAttenDb(11) adcOversample(8) smoothSamples(4) minSwingMv(12)
armSwingMv(36) maxPulseMs(1000) maxWatts(12000) bleEnabled(1) wifiTxQdbm(0)`. Empty `otaPassword` disables OTA (never unauthenticated).
`energyOffsetKwh` is 272.5194 = last `sensor.power_monitor_energy` HA recorded before the
bring-up (the real meter-side value at the 09:56:56 outage was 271.4275; ~1.09 kWh of phantom
pulses from the board sitting on a desk, un-filtered, were published by the first firmware
build and are in HA's history). Kept so the total_increasing sensor never goes backwards
(a drop would be read as a counter reset). Energy used while the board was off the meter
(09:57 onward) is not counted; reconcile with the meter display if it matters.
`/energy` on LittleFS holds the pulses counted since then. `uploadfs` rewrites the whole
partition and ERASES `/energy`: before re-running it set `energyOffsetKwh` to the current
`total_kwh` (from `power/powermon/state`), otherwise the total drops back.

### MQTT (compatible with the ESP8266 build)

`power/powermon/state` has the old fields (`watts total_kwh swing baseline rssi uptime_s`) plus
`baseline_mv swing_mv peak_mv`; `swing`/`baseline` are now millivolts. State is only published
after the first real pulse (unchanged); `power/powermon/diag` (10 s) carries `baseline_mv swing_mv
peak_mv atten_db pulses heap rst ver` from boot, for tuning. Same HA discovery and node id.
BLE presence as node `kitchen`: `presence/kitchen/apple`, `presence/kitchen/cmd` (on/off),
IRKs from `presence/config/irk`, and `presence/kitchen/wifi_rssi` (`{"rssi","ch","up"}`, 1 Hz).
CSI hook: `src/c3/csi.h/.cpp` (empty stubs, already called from `main_c3.cpp`).

### Phantom-pulse protection (added after the first bring-up on a desk)

Mains-lamp flicker (50 Hz) aliased against the 5 ms poll gave ~50 phantom pulses/s. Now: (1) a
`smoothSamples` boxcar (4 x 5 ms = one 50 Hz period) before the detector, (2) a pulse counts on
its falling edge and only if the flash lasted <= `maxPulseMs` (longer = room light, detector
re-learns), (3) a 10 s window above `maxWatts` is discarded and un-counted (`rejected` in
`power/powermon/diag`). On a desk this leaves a few tens of W of noise; on the meter it should be 0.
If real pulses are missed, lower `smoothSamples` (2) or `armSwingMv`.

### ADC tuning

Thresholds are in millivolts. The old ESP-12E "counts" are converted with an ASSUMED
1 mV/count (`kAssumedMvPerCount` in `pins.h`) -- not measured; check `swing_mv` on the live
board. Read `peak_mv` (= baseline + swing) from `power/powermon/diag` with the sensor on the meter:

- peak < 0.9 V: set `adcAttenDb` 2.5 (finer steps)
- peak < 1.3 V: set 6
- peak saturating (near 3.1 V at 11 dB): lower the 100k to 47k
- swing under `minSwingMv` / `armSwingMv` -> misaligned sensor (swing < 12 mV = guessing)

Then `uploadfs` again (or edit and re-flash only the FS).

### OTA

`pio run -e c3_ota -t upload` (host `powermon.local`, override with `--upload-port <ip>`). The
password is read from `data/config.json` by `c3_ota.py`; espota needs the PC to accept the
device's TCP connect-back on port 3233 (Windows firewall rule for the device IP, like lightbar2mqtt;
without it the invitation times out with "No response from the ESP" -- then run espota.py from nl-pi
instead (verified 12:42; use the stock espota.py from the framework package, not espota_slow): copy `firmware.bin` there, `python3 espota.py -P 3233 -i <ip> -p 3232 --auth=<otaPassword> -f firmware.bin`).
OTA is initialised before BLE and pauses BLE/CSI while uploading; BLE is skipped for one boot
after a panic/watchdog reset. Config over WiFi: `pio run -e c3_ota -t uploadfs`.

### Rollback

OTA keeps the previous image in the second slot, but there is no automatic fallback. To roll back,
`git checkout 6f85074` (esp12e baseline) is only for the old board; for the C3, check out the previous
commit of `c3-port` and `pio run -e c3_ota -t upload`, or USB-flash it.
