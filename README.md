**English** · [繁體中文](README.zh-TW.md)

# powermon

ESP8266 electricity-meter pulse counter -> MQTT -> Home Assistant.

Replaces the lost `Duco RF Gateway + Power Monitor v1.0` firmware. Rebuilt from
strings recovered out of a flash dump (`duco-esp-flash.bin`, 2026-09-08) plus the
hardware notes below. The CC1101/Duco half is not reimplemented.

## At a glance

| | |
|---|---|
| Problem | The broker address was a compile-time constant. The broker moved and the firmware had no way to be told. |
| What I built | A replacement ESP8266 firmware: photodiode pulse counter on the electricity meter, MQTT to Home Assistant, a 3-digit display, and a web config page. |
| Meter constant | 0.1 Wh/pulse (10000 imp/kWh) |
| Stack | ESP8266, Arduino framework, PlatformIO, LittleFS, PubSubClient |

## Hardware

| Function   | Pin        | Note |
|------------|------------|------|
| Photodiode | `A0`       | analog: the swing across the meter's LED window is too small for a digital input to latch |
| TM1637 CLK | `D3` GPIO0 | boot-strap pin, must be high at reset -- TM1637 idles high, so this works |
| TM1637 DIO | `D4` GPIO2 | boot-strap pin, same reasoning |

## How it works

```
meter LED --> photodiode --> A0 (ADC polled every 5 ms)
                                 |
              PulseDetector: baseline, peak follower,
              hysteresis, min gap 15 ms, arm swing 36
                                 |
              one pulse = 0.1 Wh; total = offset + pulses
                                 |
              every 10 s: watts averaged over the window
                                 |
           +-----------+---------+------------+
           |           |                      |
      TM1637      <prefix>/<nodeId>/state   HA discovery
      3 digits    (JSON)                   (4 sensors)
```

`src/pulse.h` derives the threshold from the signal: a slow EWMA tracks the dark
baseline, a fast-attack/slow-decay peak follower tracks the flash, and the
trigger points sit between them with hysteresis. `src/main.cpp` polls `A0` every
5 ms and counts a pulse on each flash after a 30 s warm-up. Every 10 s it
averages the watts over the window and publishes them.

## Why the old one stopped working

The broker address was a compile-time constant, `<old-broker-ip>`. The broker
moved to `<new-broker-ip>` and the firmware had no way to be told. This build
resolves `mqtt_host` at runtime and defaults it to an mDNS name, so an address
change no longer requires a reflash.

The code comments in `src/` explain the other choices:

- **Signal-derived threshold.** The swing stays below 12 counts in noise (swing
  must reach 36 to arm) and its DC level drifts with room light, so a fixed
  threshold would need constant recalibration (`src/pulse.h`).
- **No count until the swing is established.** Otherwise room-light flicker trips
  the thresholds; that caused a ~10 kW reading right after boot (`kArmSwing`).
- **30 s warm-up.** Pulses in the first seconds after boot are not trusted; the
  display shows `-1` and nothing is published until a real pulse (`kWarmupMs`).
- **`A0` read every 5 ms, not every 1 ms.** `analogRead()` shares one SAR ADC
  with the RF calibration; at 1 kHz the receiver missed beacons and the
  association dropped ~20 s after it formed (`loop()`).
- **WiFi sleep off, 802.11 disconnect reason logged.** Modem sleep lets the AP
  age the station out; the reason code separates an AP kick (8), a beacon
  timeout (200) and a handshake failure (15) (`setup()`).
- **Keepalive 60 s.** PubSubClient's default is 15 s, enforced at 1.5x; a stalled
  `loop()` would then cost a reconnect (`mqttConnect()`).
- **Energy persisted at every 0.01 kWh or every 10 minutes, whichever comes
  first.** Flash has a finite erase count; a power cut loses at most that much
  (`energyPersist()`).
- **Display dark 00:30 to 07:30 local time.** It sits in the bedroom's line of
  sight; it stays on until SNTP sets the clock (`isNight()`).
- **No recovery SoftAP.** Reconfigure over USB (`setup()`).

## Config

Held in LittleFS (`/cfg.json`), editable at `http://<device>/` -- broker host,
port, credentials, topic prefix, pulses-per-kWh, and an energy offset to align
the counter with the physical meter reading.

## Flashing

```
pio run -t upload                      # serial, first time
pio run -t upload -e esp12e_ota        # thereafter
```

## Repo layout

| Path | Content |
|---|---|
| `src/main.cpp` | Config, energy, MQTT, web server, `setup()` and `loop()` |
| `src/pulse.h` | `PulseDetector` |
| `src/tm1637.h` | TM1637 display driver |
| `platformio.ini` | PlatformIO environment (`esp12e`) |

## Limits and open items

- The CC1101/Duco half is not reimplemented.
- There is no recovery SoftAP. A wrong WiFi setting is fixed over USB.

## License

MIT License. See [LICENSE](LICENSE).
