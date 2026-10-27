#pragma once
#include <Arduino.h>

// Pulse detection off an analog photodiode rather than a digital input.
//
// The meter's LED is read through a photodiode whose swing is only a few tens of
// ADC counts, and its DC level drifts with room light, time of day and how the
// sensor sits against the meter face. A fixed threshold would need recalibrating
// constantly, so the threshold is derived from the signal itself: a slow EWMA
// tracks the dark baseline, a fast-attack/slow-decay peak follower tracks how
// bright a flash actually is, and the trigger points sit between them with
// hysteresis so a noisy edge cannot produce two counts.
class PulseDetector {
 public:
  // A flash is at least this long, and two meter pulses are never closer than
  // this. 15 ms allows ~66 pulses/s -- far above the ~25/s a 9 kW load produces.
  static const uint32_t kMinGapMs = 15;

  void begin(int raw) {
    baseline_ = raw;
    peak_ = raw + minSwing_;
  }

  // Override the swing limits, in whatever unit update() is fed (ADC counts on
  // the ESP8266 build, millivolts on the ESP32-C3 build). Non-positive values
  // keep the defaults.
  void configure(int minSwing, int armSwing) {
    if (minSwing > 0) minSwing_ = minSwing;
    if (armSwing > 0) armSwing_ = armSwing;
  }

  // Feed one ADC sample. Returns true on the leading edge of a pulse.
  bool update(int raw, uint32_t nowMs) {
    // Baseline follows the dark level. It must not chase the flash itself, so it
    // only adapts while the signal sits below the current low threshold.
    if (raw < lowThreshold()) {
      baseline_ += (raw - baseline_) * kBaselineAlpha;
    }
    // Peak attacks fast, decays slowly back toward the baseline, so the detector
    // re-arms if the LED gets dimmer (sensor drift) without losing counts.
    if (raw > peak_) {
      peak_ = raw;
    } else {
      peak_ -= (peak_ - baseline_) * kPeakDecay;
    }

    // Until a real flash has pushed the peak well clear of the baseline, the
    // thresholds sit a few counts above noise and room-light flicker trips them
    // dozens of times a second -- that is where the ~10 kW reading right after
    // boot came from. Track, but do not count, until the swing is established.
    bool fired = false;
    if (!high_ && swing() >= armSwing_ && raw > highThreshold() &&
        (nowMs - lastPulseMs_) >= kMinGapMs) {
      high_ = true;
      lastPulseMs_ = nowMs;
      fired = true;
    } else if (high_ && raw < lowThreshold()) {
      high_ = false;
    }
    return fired;
  }

  int baseline() const { return (int)baseline_; }
  int peak() const { return (int)peak_; }
  // Signal quality, in ADC counts. Below kMinSwing the detector is guessing and
  // the sensor probably needs repositioning -- exposed so HA can show it.
  int swing() const { return (int)(peak_ - baseline_); }

 private:
  // Anything smaller than this is noise, not an LED flash. Default is in ADC
  // counts (ESP8266); the C3 build passes millivolt values via configure().
  int minSwing_ = 12;
  // Swing needed before any pulse is counted. Well above noise, well below the
  // ~900 counts a seated sensor sees.
  int armSwing_ = 36;
  static constexpr float kBaselineAlpha = 0.002f;
  static constexpr float kPeakDecay = 0.0005f;

  float effectiveSwing() const {
    float s = peak_ - baseline_;
    return s < minSwing_ ? (float)minSwing_ : s;
  }
  float highThreshold() const { return baseline_ + effectiveSwing() * 0.60f; }
  float lowThreshold() const { return baseline_ + effectiveSwing() * 0.35f; }

  float baseline_ = 0;
  float peak_ = 0;
  bool high_ = false;
  uint32_t lastPulseMs_ = 0;
};
