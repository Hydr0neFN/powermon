#pragma once
#include <Arduino.h>

// Every board-specific constant in one place (ESP32-C3 SuperMini).
//
// Photodiode circuit: 3V3 -> photodiode -> node, 100k from node to GND (same as
// the ESP-12E build). GPIO3 is ADC1_CH3; ADC1 keeps working while WiFi is up.
static const uint8_t kPinPhotodiode = 3;

// LED.C68 3-digit module speaking the TM1637 protocol. GPIO0/1 are not strapping
// pins on the C3 (those are 2, 8, 9). Alternative pair if these are taken: CLK 5, DIO 6.
static const uint8_t kPinClk = 1;
static const uint8_t kPinDio = 0;

static const uint8_t kDisplayBrightness = 5;  // 0..7

// ADC -> millivolts. analogReadMilliVolts() uses the factory eFuse calibration.
// ASSUMPTION (not measured): the old ESP-12E thresholds (swing "a few tens of
// counts", arm at 36, "misaligned" below 12) were in raw 10-bit counts of a
// 0..1 V input, i.e. about 1 mV per count. Re-tune from the published
// swing_mv / peak_mv once the sensor is on the meter (README, "ADC tuning").
static const float kAssumedMvPerCount = 1.0f;
static const int kDefaultMinSwingMv = (int)(12 * kAssumedMvPerCount);
static const int kDefaultArmSwingMv = (int)(36 * kAssumedMvPerCount);
