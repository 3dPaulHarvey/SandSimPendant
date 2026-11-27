#ifndef SIMCIRCLE_SHAKE_INPUT_H
#define SIMCIRCLE_SHAKE_INPUT_H

#include "MotionCalibration.h"

namespace motion {

struct ShakePulse {
  // Per-axis RMS velocity impulse in legacy 3px cells/sec. Apply once, with
  // the caller's unit conversion; do not multiply this impulse by dt again.
  float agitation;
  float durationSeconds;
  // Integral of nominalKick^2 * dt, before 60Hz impulse normalization.
  float motionEnergy;
  // Deadzoned transient acceleration integrated over fresh samples, in g*s.
  // This intentionally rejects sustained/DC acceleration: direct accelerometer
  // tilt already supplies the simulation's continuous force.
  Vec3 linearImpulseGSeconds;
  ShakePulse() : agitation(0), durationSeconds(0), motionEnergy(0),
                 linearImpulseGSeconds() {}
};

// Detect brief motion without integrating an orientation estimate indefinitely.
// Each fresh acceleration is compared with the preceding acceleration rotated
// by the gyro. Exact rigid rotation cancels; translation changes survive in a
// 100ms high-pass memory. Rotate that memory too, keeping it in chip coordinates.
//
// This is a transient effect, not a gravity/linear-acceleration measurement.
// Constant acceleration decays away. With fixed acceleration and constant gyro
// bias, residual magnitude is bounded by tau * |omega| * |accel|: a 35dps bias at
// 1g contributes at most 0.0611g, below the 0.08g motion gate. Error cannot accumulate
// over minutes or become a persistent, saturated artificial shove. No sensor
// calibration or mounting/tilt direction is changed by this detector.
class ShakeInput {
 public:
  static constexpr float kResponseSeconds = 0.100f;
  Vec3 latestLinear;  // Gyro-compensated transient acceleration in g.
  float activity = 0; // Current excitation, 0..1, before pulse accumulation.

  void reset() {
    latestLinear = previousAccel_ = pendingImpulse_ = Vec3();
    activity = pendingDuration_ = pendingEnergy_ = 0;
    initialized_ = false;
  }

  void update(Vec3 accelG, Vec3 gyroDps, float dtSeconds) {
    const float accelerationLength = norm(accelG);
    const float gyroLength = norm(gyroDps);
    if (!isFinite(accelG) || !isFinite(gyroDps) ||
        !isfinite(accelerationLength) || !isfinite(gyroLength) ||
        accelerationLength > 8.0f || gyroLength > 4000.0f ||
        !isfinite(dtSeconds) || dtSeconds <= 0.0f) {
      reset();
      return;
    }
    if (!initialized_ || dtSeconds > 0.1f) {
      // First sample/outage establishes a reference and never emits a kick.
      reset();
      previousAccel_ = accelG;
      initialized_ = true;
      return;
    }

    Vec3 predictedPrevious = previousAccel_;
    Vec3 rotatedHistory = latestLinear;
    if (gyroLength > 0.000001f) {
      const Vec3 axis(gyroDps.x / gyroLength, gyroDps.y / gyroLength,
                      gyroDps.z / gyroLength);
      const float angle = -gyroLength * 0.017453292519943295f * dtSeconds;
      const float cosine = cosf(angle), sine = sinf(angle);
      predictedPrevious = rotate(previousAccel_, axis, cosine, sine);
      rotatedHistory = rotate(latestLinear, axis, cosine, sine);
    }

    const float decay = expf(-dtSeconds / kResponseSeconds);
    // Integrate a linearly changing acceleration over the interval. Scaling
    // the difference by this exact gain keeps response/energy consistent when
    // sample timing varies, and bounds constant gyro-bias error independently
    // of sample rate. The small-dt limit avoids float cancellation around 1.
    const float gain = dtSeconds < 0.00001f ? 1.0f :
                       kResponseSeconds * (1.0f - decay) / dtSeconds;
    latestLinear = Vec3(rotatedHistory.x * decay + (accelG.x - predictedPrevious.x) * gain,
                        rotatedHistory.y * decay + (accelG.y - predictedPrevious.y) * gain,
                        rotatedHistory.z * decay + (accelG.z - predictedPrevious.z) * gain);
    previousAccel_ = accelG;

    const float lateral = sqrtf(latestLinear.x * latestLinear.x +
                                latestLinear.y * latestLinear.y);
    if (lateral > 0.08f) {
      const float factor = (lateral - 0.08f) / lateral * dtSeconds;
      pendingImpulse_.x += latestLinear.x * factor;
      pendingImpulse_.y += latestLinear.y * factor;
    }
    const float normalImpulse = positive(fabsf(latestLinear.z) - 0.08f) * dtSeconds;
    pendingImpulse_.z += latestLinear.z < 0 ? -normalImpulse : normalImpulse;
    pendingImpulse_ = clampNorm(pendingImpulse_, 0.1f);
    const float xyExcitation = positive(lateral - 0.08f) * 0.45f;
    const float zExcitation = positive(fabsf(latestLinear.z) - 0.08f);
    const float strength = sqrtf(xyExcitation * xyExcitation +
                                 zExcitation * zExcitation);
    const float nominalKick = minimum(18.0f, strength * 18.0f);
    activity = nominalKick / 18.0f;

    // Preserve variance across sensor and physics consumption rates. Pending
    // energy/impulse is capped if a consumer stalls; consume clears it once.
    pendingEnergy_ = minimum(18.0f * 18.0f / 60.0f,
                            pendingEnergy_ + nominalKick * nominalKick * dtSeconds);
    pendingDuration_ = minimum(1.0f, pendingDuration_ + dtSeconds);
  }

  ShakePulse consume() {
    ShakePulse pulse;
    pulse.motionEnergy = pendingEnergy_;
    pulse.durationSeconds = pendingDuration_;
    pulse.agitation = minimum(18.0f, sqrtf(pendingEnergy_ * 60.0f));
    pulse.linearImpulseGSeconds = pendingImpulse_;
    pendingEnergy_ = pendingDuration_ = 0;
    pendingImpulse_ = Vec3();
    return pulse;
  }

 private:
  Vec3 previousAccel_, pendingImpulse_;
  float pendingDuration_ = 0, pendingEnergy_ = 0;
  bool initialized_ = false;

  static float minimum(float a, float b) { return a < b ? a : b; }
  static float positive(float value) { return value > 0 ? value : 0; }
  static Vec3 rotate(Vec3 value, Vec3 axis, float cosine, float sine) {
    const Vec3 perpendicular(axis.y * value.z - axis.z * value.y,
                             axis.z * value.x - axis.x * value.z,
                             axis.x * value.y - axis.y * value.x);
    const float parallel = (axis.x * value.x + axis.y * value.y + axis.z * value.z) *
                           (1.0f - cosine);
    return Vec3(value.x * cosine + perpendicular.x * sine + axis.x * parallel,
                value.y * cosine + perpendicular.y * sine + axis.y * parallel,
                value.z * cosine + perpendicular.z * sine + axis.z * parallel);
  }
};

} // namespace motion
#endif
