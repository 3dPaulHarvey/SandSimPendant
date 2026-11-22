#ifndef SIMCIRCLE_SHAKE_INPUT_H
#define SIMCIRCLE_SHAKE_INPUT_H

#include "MotionCalibration.h"

namespace motion {

struct ShakePulse {
  // An instantaneous, per-axis RMS random velocity impulse in cells/sec.
  // Apply once with Simulation::kick(0, 0, agitation), without another dt.
  float agitation;
  float durationSeconds;
  // Integral of nominalKick^2 * dt, before the 60Hz impulse normalization.
  float motionEnergy;
  // Deadzoned linear acceleration integrated over fresh samples, in g*s.
  // The caller may deliberately amplify this directional shove. Apply the
  // acceleration-to-velocity scale once; do not multiply by dt a second time.
  Vec3 linearImpulseGSeconds;
  ShakePulse() : agitation(0), durationSeconds(0), motionEnergy(0),
                 linearImpulseGSeconds() {}
};

// Extract bounded agitation from calibrated acceleration and gyro readings.
// Acceleration includes gravity. Existing screen-XY acceleration remains the
// physical base input. The pulse's signed linear integral lets a caller add
// deliberate dynamic-shove amplification, rather than accidentally counting
// gravity twice. XYZ linear acceleration also controls zero-mean agitation,
// with physical sensor Z weighted more strongly for screen-normal shaking.
// The MPU is assumed parallel to the display, as in the established XY map.
// This estimates gravity/linear acceleration, not XYZ position or velocity.
class ShakeInput {
 public:
  Vec3 latestLinear;  // Unthresholded acceleration minus body gravity, in g.
  float activity = 0; // Current excitation, 0..1; no artificial release tail.

  void reset() {
    latestLinear = Vec3();
    activity = 0;
    gravityUnit_ = Vec3(0, 0, 1);
    gravityMagnitude_ = 1.0f;
    previousAccel_ = Vec3();
    pendingImpulse_ = Vec3();
    stableSeconds_ = pendingDuration_ = pendingEnergy_ = 0;
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
      // A first reading or sampling outage establishes a reference, rather
      // than applying all the elapsed time as one potentially huge impulse.
      reset();
      if (accelerationLength > 0.0001f)
        gravityUnit_ = divide(accelG, accelerationLength);
      if (accelerationLength >= 0.8f && accelerationLength <= 1.2f)
        gravityMagnitude_ = accelerationLength;
      previousAccel_ = accelG;
      initialized_ = true;
      return;
    }

    // A fixed vector in world space rotates oppositely in chip coordinates:
    // dg_body/dt = -omega_body x g_body. Exact Rodrigues rotation avoids the
    // artificial Z residual that one Euler step creates during rapid tilts.
    if (gyroLength > 0.000001f) {
      const Vec3 axis = divide(gyroDps, gyroLength);
      const float angle = -gyroLength * 0.017453292519943295f * dtSeconds;
      gravityUnit_ = rotate(gravityUnit_, axis, angle);
    }
    gravityUnit_ = unitOr(gravityUnit_, Vec3(0, 0, 1));

    const float residual = norm(subtract(accelG, scaledGravity()));
    const float magnitudeError = fabsf(accelerationLength - 1.0f);
    const bool stable = gyroLength < 3.0f && accelerationLength >= 0.8f &&
                        accelerationLength <= 1.2f &&
                        norm(subtract(accelG, previousAccel_)) < 0.020f;
    stableSeconds_ = stable ? minimum(2.0f, stableSeconds_ + dtSeconds) : 0.0f;

    // Correct gyro drift cautiously. A near-1g magnitude alone is insufficient:
    // lateral acceleration can also have a magnitude close to gravity.
    float confidence = positive(1.0f - magnitudeError / 0.10f) *
                       positive(1.0f - residual / 0.18f);
    // Recover if initialization happened during motion or gyro drift caused a
    // large direction error. Ordinary near-1g rest recovers after0.75s. Wider
    // 0.8..1.2g rest must persist1.5s before learning pose-dependent magnitude,
    // preserving brief real pushes. One-pose bias calibration cannot correct
    // scale errors; forcing every still pose to1g can leave permanent jitter.
    // Long constant acceleration and static tilt remain indistinguishable.
    if (stableSeconds_ >= 0.75f && magnitudeError < 0.04f) confidence = 1.0f;
    if (stableSeconds_ >= 1.5f) {
      confidence = 1.0f;
      gravityMagnitude_ += smoothingAlpha(dtSeconds, 1.0f) *
                           (accelerationLength - gravityMagnitude_);
    }
    if (confidence > 0 && accelerationLength > 0.0001f) {
      const Vec3 observedGravity = divide(accelG, accelerationLength);
      gravityUnit_ = toward(gravityUnit_, observedGravity,
                            smoothingAlpha(dtSeconds, 0.75f) * confidence);
    }
    previousAccel_ = accelG;
    latestLinear = subtract(accelG, scaledGravity());

    const float lateral = sqrtf(latestLinear.x * latestLinear.x +
                                latestLinear.y * latestLinear.y);
    // A radial XY deadzone preserves push direction and keeps a normal-Z
    // shake from opening the lateral gate. Integrating signed acceleration
    // also lets opposite pushes cancel coherently between physics ticks.
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

    // Integrating squared impulses preserves short events between physics
    // ticks. At 60Hz consumption, a constant nominalKick becomes that same
    // RMS kick. At other rates, summed impulse variance is preserved until
    // the deliberate 18 cells/sec cap is reached. Pending energy is bounded
    // so a stalled consumer cannot later inject an unlimited random impulse.
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
  Vec3 gravityUnit_ = Vec3(0, 0, 1), previousAccel_, pendingImpulse_;
  float gravityMagnitude_ = 1, stableSeconds_ = 0, pendingDuration_ = 0, pendingEnergy_ = 0;
  bool initialized_ = false;

  static float minimum(float a, float b) { return a < b ? a : b; }
  static float positive(float value) { return value > 0 ? value : 0; }
  static Vec3 divide(Vec3 value, float denominator) {
    return Vec3(value.x / denominator, value.y / denominator, value.z / denominator);
  }
  static Vec3 subtract(Vec3 a, Vec3 b) {
    return Vec3(a.x - b.x, a.y - b.y, a.z - b.z);
  }
  Vec3 scaledGravity() const {
    return Vec3(gravityUnit_.x * gravityMagnitude_, gravityUnit_.y * gravityMagnitude_,
                gravityUnit_.z * gravityMagnitude_);
  }
  static float dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
  static Vec3 cross(Vec3 a, Vec3 b) {
    return Vec3(a.y * b.z - a.z * b.y,
                a.z * b.x - a.x * b.z,
                a.x * b.y - a.y * b.x);
  }
  static Vec3 unitOr(Vec3 value, Vec3 fallback) {
    const float length = norm(value);
    return length > 0.000001f ? divide(value, length) : fallback;
  }
  static Vec3 rotate(Vec3 value, Vec3 axis, float angle) {
    const float cosine = cosf(angle), sine = sinf(angle);
    const Vec3 perpendicular = cross(axis, value);
    const float parallel = dot(axis, value) * (1.0f - cosine);
    return Vec3(value.x * cosine + perpendicular.x * sine + axis.x * parallel,
                value.y * cosine + perpendicular.y * sine + axis.y * parallel,
                value.z * cosine + perpendicular.z * sine + axis.z * parallel);
  }
  static Vec3 toward(Vec3 from, Vec3 to, float fraction) {
    Vec3 axis = cross(from, to);
    float sine = norm(axis), cosine = dot(from, to);
    if (cosine > 1.0f) cosine = 1.0f;
    if (cosine < -1.0f) cosine = -1.0f;
    float angle;
    if (sine < 0.000001f) {
      if (cosine >= 0) return from;
      // Antipodal recovery needs an actual turn: normalizing a linear blend
      // of opposite vectors otherwise leaves the estimate stuck forever.
      axis = unitOr(cross(from, fabsf(from.x) < 0.8f ? Vec3(1, 0, 0) : Vec3(0, 1, 0)),
                    Vec3(1, 0, 0));
      angle = 3.14159265358979323846f;
    } else {
      axis = divide(axis, sine);
      angle = atan2f(sine, cosine);
    }
    return unitOr(rotate(from, axis, angle * fraction), from);
  }
};

}  // namespace motion

#endif
