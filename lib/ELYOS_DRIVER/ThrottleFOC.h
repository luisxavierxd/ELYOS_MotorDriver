#pragma once
#include <cstdint>
#include <cmath>
#include <algorithm>
#include "FOC_Parameters.h"
#include "Pinout.h"
#include "elyos_hal.h"

class ThrottleFOC {
public:
  struct Config {
    // ---------- ADC / pedal ----------
    int pedalPin = THROTTLE_PIN;
    uint16_t adcMin = 490;          // calibrate!
    uint16_t adcMax = 2750;         // calibrate!
    uint16_t rawAdc_ = 0;
    float deadband = THROTTLE_DEADBAND;  // % pedal dead zone
    float faultLowMargin = FAULT_LOW_MARGIN;   // ADC counts below adcMin tolerated
    float faultHighMargin = FAULT_HIGH_MARGIN;  // ADC counts above adcMax tolerated

    // ---------- Filtering ----------
    // pedal LPF time constant in seconds
    float pedalFilterTau = THROTTLE_FILTER_TAU;  // 30 ms

    // ---------- Pedal shaping ----------
    // iq_norm = blendLinear * p + (1-blendLinear) * p^2
    float blendLinear = THROTTLE_BLEND_LINEAR;

    // ---------- Current limits ----------
    float iqMax = THROTTLE_IQ_MAX;                                  // absolute allowed q-axis current [A]
    float iqLaunchMax = THROTTLE_IQ_LAUNCH_MAX;                     // lower current cap near zero speed [A]
    float launchSpeedRadPerSec = THROTTLE_LAUNCH_SPEED_RAD_PER_SEC; // below this, use launch limit

    // Optional speed-based derating
    float speedForFullIq = THROTTLE_SPEED_FOR_FULL_IQ;   // rad/s where full iqMax becomes available

    // ---------- Rate limiting ----------
    float iqRampUp = THROTTLE_IQ_RAMP_UP;        // A/s, slower for efficiency
    float iqRampDown = THROTTLE_IQ_RAMP_DOWN;    // A/s, faster for safety/natural release

    // ---------- Optional creep / stiction ----------
    bool enableMinStartCurrent = THROTTLE_ENABLE_MIN_START_CURRENT;
    float minStartPedal = THROTTLE_MIN_START_PEDAL;       // if pedal exceeds this, optionally inject small current
    float minStartCurrent = THROTTLE_MIN_START_CURRENT;   // enough only to overcome stiction if needed

    // ---------- Safety behavior ----------
    bool cutToZeroOnFault = THROTTLE_CUT_TO_ZERO_ON_FAULT;
  };

  explicit ThrottleFOC(const Config& cfg) : cfg_(cfg) {}
  uint16_t getRaw() const { return cfg_.rawAdc_; }

  void begin() {
    elyos::Adc::setResolution(THROTTLE_RESOLUTION_BITS);
    elyos::Gpio::setMode(cfg_.pedalPin, elyos::GpioMode::Input);
    lastMicros_ = elyos::Time::micros();
    pedalFilt_ = 0.0f;
    iqRef_ = 0.0f;
    fault_ = false;
  }

  // motorSpeedRadPerSec: use motor.shaftVelocity() or your own filtered vehicle/motor speed
  float update(float motorSpeedRadPerSec) {
    const uint32_t now = elyos::Time::micros();
    float dt = (now - lastMicros_) * 1e-6f;
    lastMicros_ = now;

    // protect against dt glitches
    if (dt <= 0.0f || dt > 0.1f) {
      dt = 0.001f; // default 1 ms
    }

    // 1) Read pedal ADC via HAL
    uint16_t adc = elyos::Adc::read(cfg_.pedalPin, THROTTLE_RESOLUTION_BITS);
    cfg_.rawAdc_ = adc;

    // 2) Range/fault check
    fault_ = isAdcFault(adc);

    // 3) Normalize raw pedal to 0..1
    float pedal = normalizePedal(adc);

    // 4) Deadband
    pedal = applyDeadband(pedal, cfg_.deadband);

    // 5) Light LPF on pedal
    pedalFilt_ = lowPass(pedalFilt_, pedal, cfg_.pedalFilterTau, dt);

    // 6) Shape pedal with blended linear/quadratic curve
    float shaped = shapePedal(pedalFilt_);

    // 7) Compute effective current ceiling
    float iqCeiling = computeEffectiveIqLimit(fabsf(motorSpeedRadPerSec));

    // 8) Map shaped pedal to desired current
    float iqDesired = shaped * iqCeiling;

    // Optional minimum start current
    if (cfg_.enableMinStartCurrent &&
        pedalFilt_ > cfg_.minStartPedal &&
        iqDesired < cfg_.minStartCurrent) {
      iqDesired = cfg_.minStartCurrent;
    }

    // 9) Fault reaction
    if (fault_ && cfg_.cutToZeroOnFault) {
      iqDesired = 0.0f;
    }

    // 10) Asymmetric slew-rate limit
    iqRef_ = slewLimit(iqRef_, iqDesired, cfg_.iqRampUp, cfg_.iqRampDown, dt);

    // 11) Final clamp
    iqRef_ = constrainf(iqRef_, 0.0f, cfg_.iqMax);

    return iqRef_;
  }

  // Optional: immediate reset to zero current, e.g. on brake/safety event
  void reset() {
    pedalFilt_ = 0.0f;
    iqRef_ = 0.0f;
    fault_ = false;
    lastMicros_ = elyos::Time::micros();
  }

  bool faulted() const { return fault_; }
  float filteredPedal() const { return pedalFilt_; }
  float iqCommand() const { return iqRef_; }

private:
  Config cfg_;
  uint32_t lastMicros_ = 0;
  float pedalFilt_ = 0.0f;
  float iqRef_ = 0.0f;
  bool fault_ = false;

  static float constrainf(float x, float a, float b) {
    if (x < a) return a;
    if (x > b) return b;
    return x;
  }

  bool isAdcFault(uint16_t adc) const {
    if (adc + cfg_.faultLowMargin < cfg_.adcMin) return true;
    if (adc > cfg_.adcMax + cfg_.faultHighMargin) return true;
    return false;
  }

  float normalizePedal(uint16_t adc) const {
    const float denom = (float)(cfg_.adcMax - cfg_.adcMin);
    if (denom <= 1.0f) return 0.0f;
    float p = ((float)adc - (float)cfg_.adcMin) / denom;
    return constrainf(p, 0.0f, 1.0f);
  }

  static float applyDeadband(float p, float deadband) {
    if (deadband <= 0.0f) return p;
    if (p <= deadband) return 0.0f;
    return (p - deadband) / (1.0f - deadband);
  }

  static float lowPass(float y, float x, float tau, float dt) {
    if (tau <= 0.0f) return x;
    float alpha = dt / (tau + dt);
    return y + alpha * (x - y);
  }

  float shapePedal(float p) const {
    float linearPart = cfg_.blendLinear * p;
    float quadPart = (1.0f - cfg_.blendLinear) * p * p;
    return constrainf(linearPart + quadPart, 0.0f, 1.0f);
  }

  float computeEffectiveIqLimit(float absSpeed) const {
    // launch cap at very low speed for smooth starts and lower peaks
    if (absSpeed <= cfg_.launchSpeedRadPerSec) {
      return std::min(cfg_.iqLaunchMax, cfg_.iqMax);
    }

    // blend from launch limit to full iqMax as speed rises
    if (absSpeed < cfg_.speedForFullIq) {
      float t = (absSpeed - cfg_.launchSpeedRadPerSec) /
                (cfg_.speedForFullIq - cfg_.launchSpeedRadPerSec);
      t = constrainf(t, 0.0f, 1.0f);
      return cfg_.iqLaunchMax + t * (cfg_.iqMax - cfg_.iqLaunchMax);
    }

    return cfg_.iqMax;
  }

  static float slewLimit(float current, float target,
                         float upRate, float downRate, float dt) {
    float delta = target - current;
    float upStep = upRate * dt;
    float downStep = downRate * dt;

    if (delta > 0.0f) {
      if (delta > upStep) delta = upStep;
    } else {
      if (-delta > downStep) delta = -downStep;
    }
    return current + delta;
  }
};