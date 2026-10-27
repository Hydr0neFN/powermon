#pragma once
#include <Arduino.h>

// Minimal TM1637 driver, bit-banged.
//
// Written out rather than pulled in as a library because the whole protocol is
// forty lines and the display here is a 3-digit module, which the common
// libraries address awkwardly.
class TM1637 {
 public:
  TM1637(uint8_t clk, uint8_t dio) : clk_(clk), dio_(dio) {}

  void begin() {
    pinMode(clk_, OUTPUT);
    pinMode(dio_, OUTPUT);
    digitalWrite(clk_, HIGH);
    digitalWrite(dio_, HIGH);
  }

  void setBrightness(uint8_t level) { brightness_ = level & 0x07; }

  // Right-aligned integer, blanking leading zeros. Values that do not fit show
  // as "---" rather than a misleading truncation.
  void showNumber(int value, uint8_t digits = 3) {
    uint8_t seg[6] = {0};
    int limit = 1;
    for (uint8_t i = 0; i < digits; i++) limit *= 10;
    if (value < 0 || value >= limit) {
      for (uint8_t i = 0; i < digits; i++) seg[i] = 0x40;  // dash
    } else {
      for (int8_t i = digits - 1; i >= 0; i--) {
        seg[i] = kDigits[value % 10];
        value /= 10;
        if (value == 0 && i > 0) {
          for (int8_t j = i - 1; j >= 0; j--) seg[j] = 0x00;
          break;
        }
      }
    }
    write(seg, digits);
  }

  // Watts at or above 1 kW, shown as kilowatts with as many decimals as fit:
  // 1234 W -> "1.23", 12345 W -> "12.3". Values that still do not fit fall
  // back to dashes.
  void showKilowatts(int watts, uint8_t digits = 3) {
    // Scale so the value fills every digit: hundredths of a kW below 10 kW,
    // tenths below 100 kW, and so on. dp is the digit carrying the point.
    int scaled = (watts + 5) / 10;  // hundredths of a kW, rounded
    int8_t dp = 0;
    int limit = 1;
    for (uint8_t i = 0; i < digits; i++) limit *= 10;
    while (scaled >= limit && dp < digits - 1) {
      scaled = (scaled + 5) / 10;
      dp++;
    }
    uint8_t seg[6] = {0};
    if (watts < 0 || scaled >= limit) {
      for (uint8_t i = 0; i < digits; i++) seg[i] = 0x40;  // dash
    } else {
      for (int8_t i = digits - 1; i >= 0; i--) {
        seg[i] = kDigits[scaled % 10];
        scaled /= 10;
      }
      seg[dp] |= 0x80;  // decimal point
    }
    write(seg, digits);
  }

  void clear(uint8_t digits = 3) {
    uint8_t seg[6] = {0};
    write(seg, digits);
  }

 private:
  static const uint8_t kDigits[10];

  void start() { digitalWrite(dio_, LOW); delayMicroseconds(kBitUs); digitalWrite(clk_, LOW); delayMicroseconds(kBitUs); }
  void stop()  { digitalWrite(clk_, LOW); delayMicroseconds(kBitUs); digitalWrite(dio_, LOW); delayMicroseconds(kBitUs);
                 digitalWrite(clk_, HIGH); delayMicroseconds(kBitUs); digitalWrite(dio_, HIGH); delayMicroseconds(kBitUs); }

  void writeByte(uint8_t b) {
    for (uint8_t i = 0; i < 8; i++) {
      digitalWrite(clk_, LOW);
      digitalWrite(dio_, (b & 0x01) ? HIGH : LOW);
      delayMicroseconds(kBitUs);
      digitalWrite(clk_, HIGH);
      delayMicroseconds(kBitUs);
      b >>= 1;
    }
    // Ninth clock: the chip pulls DIO low to acknowledge. Nothing here depends
    // on the ack, but the clock still has to be issued.
    digitalWrite(clk_, LOW);
    pinMode(dio_, INPUT);
    delayMicroseconds(kBitUs);
    digitalWrite(clk_, HIGH);
    delayMicroseconds(kBitUs);
    digitalWrite(clk_, LOW);
    pinMode(dio_, OUTPUT);
  }

  void write(const uint8_t* seg, uint8_t len) {
    start(); writeByte(0x40); stop();            // auto-increment address mode
    start(); writeByte(0xC0);                    // start at address 0
    // Always fill all six grid addresses. The chip's display RAM powers up with
    // random contents, and on this module grids the sketch never writes still
    // reach LEDs -- stray decimal points appeared after a cold boot.
    for (uint8_t i = 0; i < 6; i++) writeByte(i < len ? seg[i] : 0x00);
    stop();
    start(); writeByte(0x88 | brightness_); stop();
  }

  static const uint8_t kBitUs = 100;
  uint8_t clk_, dio_;
  uint8_t brightness_ = 5;
};

const uint8_t TM1637::kDigits[10] = {
    0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F};
