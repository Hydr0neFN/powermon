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
