#ifndef SIMCIRCLE_MOTION_CALIBRATION_H
#define SIMCIRCLE_MOTION_CALIBRATION_H

#include <math.h>
#include <stdint.h>

// Portable sensor math. Acceleration units are g; gyro units are degrees/sec.
// Flat calibration requires the physical MPU6050 +Z axis pointing upward.
// Keep the board still, face-up, for 400 samples (2 seconds at 200 Hz).
// This one-pose procedure corrects offsets; it does not calibrate axis gains.
namespace motion {

struct Vec3 {
  float x, y, z;
  constexpr Vec3(float px = 0, float py = 0, float pz = 0) : x(px), y(py), z(pz) {}
};

struct Vec2 {
  float x, y;
  constexpr Vec2(float px = 0, float py = 0) : x(px), y(py) {}
};

struct Calibration {
  Vec3 accelBias;
  Vec3 gyroBias;
  bool valid;
  Calibration(Vec3 accel = Vec3(), Vec3 gyro = Vec3(), bool isValid = false)
      : accelBias(accel), gyroBias(gyro), valid(isValid) {}
};

inline bool isFinite(Vec3 value) {
  return isfinite(value.x) && isfinite(value.y) && isfinite(value.z);
}

inline float norm(Vec3 value) {
  if (!isFinite(value)) return NAN;
  // Scaling avoids overflow even when a caller passes a corrupt large value.
  float scale = fabsf(value.x);
  if (fabsf(value.y) > scale) scale = fabsf(value.y);
  if (fabsf(value.z) > scale) scale = fabsf(value.z);
  if (scale == 0.0f) return 0.0f;
  const float x = value.x / scale, y = value.y / scale, z = value.z / scale;
  return scale * sqrtf(x * x + y * y + z * z);
}

inline Vec3 clampNorm(Vec3 value, float maximum) {
  if (!isFinite(value) || !isfinite(maximum) || maximum <= 0.0f) return Vec3();
  const float magnitude = norm(value);
  if (magnitude <= maximum) return value;
  // Normalize via the largest component first, including the rare case where
  // the finite input's mathematical norm exceeds the representable float range.
  float scale = fabsf(value.x);
  if (fabsf(value.y) > scale) scale = fabsf(value.y);
  if (fabsf(value.z) > scale) scale = fabsf(value.z);
  Vec3 unit(value.x / scale, value.y / scale, value.z / scale);
  const float factor = maximum / norm(unit);
  return Vec3(unit.x * factor, unit.y * factor, unit.z * factor);
}

inline Vec3 correctAcceleration(Vec3 raw, const Calibration &calibration) {
  if (!calibration.valid || !isFinite(calibration.accelBias)) return raw;
  return Vec3(raw.x - calibration.accelBias.x,
              raw.y - calibration.accelBias.y,
              raw.z - calibration.accelBias.z);
}

inline Vec3 correctGyro(Vec3 raw, const Calibration &calibration) {
  if (!calibration.valid || !isFinite(calibration.gyroBias)) return raw;
  return Vec3(raw.x - calibration.gyroBias.x,
              raw.y - calibration.gyroBias.y,
              raw.z - calibration.gyroBias.z);
}

// Match the sand sketch's established mounting choices. "swap" swaps physical
// chip axes before the screen's sign convention, yielding (-ay,+ax).
inline Vec2 mapGravity(Vec3 correctedAccel, bool swap = false,
                       bool flipX = false, bool flipY = false) {
  Vec2 gravity(swap ? -correctedAccel.y : -correctedAccel.x,
               swap ? correctedAccel.x : correctedAccel.y);
  if (flipX) gravity.x = -gravity.x;
  if (flipY) gravity.y = -gravity.y;
  return gravity;
}

// Use filtered += alpha * (sample - filtered). Initialize the first sample
// directly in the caller. A fixed time constant behaves the same at any Hz.
inline float smoothingAlpha(float dtSeconds, float tauSeconds) {
  if (!isfinite(dtSeconds) || !isfinite(tauSeconds) || dtSeconds <= 0.0f) return 0;
  if (tauSeconds <= 0.0f) return 1;
  return 1.0f - expf(-dtSeconds / tauSeconds);
}

enum class CalibrationError {
  None,
  TooFewSamples,
  InvalidSample,
  AccelMoving,
  GyroMoving,
  GyroTooFast,
  GravityMagnitude,
  NotFlat
};

struct CalibrationResult {
  Calibration calibration;
  CalibrationError error;
  Vec3 accelMean, gyroMean;
  Vec3 accelVariance, gyroVariance;
  Vec3 accelStd, gyroStd;
  uint32_t sampleCount;
  CalibrationResult() : error(CalibrationError::TooFewSamples), sampleCount(0) {}
  bool ok() const { return error == CalibrationError::None && calibration.valid; }
};

class StationaryCalibrator {
 public:
  explicit StationaryCalibrator(uint32_t requiredSamples = 400)
      : requiredSamples_(requiredSamples < 2 ? 2 : requiredSamples) { reset(); }

  void reset() {
    count_ = 0;
    invalidSample_ = false;
    accelMean_ = gyroMean_ = accelM2_ = gyroM2_ = Vec3();
  }

  uint32_t count() const { return count_; }
  bool ready() const { return count_ >= requiredSamples_; }

  // A rejected sample poisons this attempt; reset before trying again. Once
  // ready, the fixed window is frozen so late finish() calls cannot extend it.
  bool add(Vec3 accelerationG, Vec3 gyroDps) {
    if (ready()) return true;
    if (!isFinite(accelerationG) || !isFinite(gyroDps) ||
        norm(accelerationG) > 16.0f || norm(gyroDps) > 4000.0f) {
      invalidSample_ = true;
      return false;
    }
    ++count_;
    update(accelerationG, accelMean_, accelM2_);
    update(gyroDps, gyroMean_, gyroM2_);
    return true;
  }

  CalibrationResult finish() const {
    CalibrationResult result;
    result.sampleCount = count_;
    result.accelMean = accelMean_;
    result.gyroMean = gyroMean_;
    if (count_ > 1) {
      const float denominator = static_cast<float>(count_ - 1);
      result.accelVariance = divideNonnegative(accelM2_, denominator);
      result.gyroVariance = divideNonnegative(gyroM2_, denominator);
      result.accelStd = squareRoot(result.accelVariance);
      result.gyroStd = squareRoot(result.gyroVariance);
    }
    if (invalidSample_) result.error = CalibrationError::InvalidSample;
    else if (!ready()) result.error = CalibrationError::TooFewSamples;
    else if (exceedsAxis(result.accelStd, 0.020f)) result.error = CalibrationError::AccelMoving;
    else if (exceedsAxis(result.gyroStd, 1.0f)) result.error = CalibrationError::GyroMoving;
    // MPU6050 initial zero-rate offsets can reach +/-20 degrees/sec. Allow
    // stationary factory bias; variance rejects changing rotation. A constant
    // rotation cannot be distinguished fully from bias by a single pose.
    else if (norm(gyroMean_) > 35.0f) result.error = CalibrationError::GyroTooFast;
    else if (norm(accelMean_) < 0.85f || norm(accelMean_) > 1.15f)
      result.error = CalibrationError::GravityMagnitude;
    else if (fabsf(accelMean_.x) > 0.20f || fabsf(accelMean_.y) > 0.20f ||
             accelMean_.z < 0.8f || accelMean_.z > 1.2f)
      result.error = CalibrationError::NotFlat;
    else {
      result.error = CalibrationError::None;
      // Preserve physical gravity. Subtracting the entire mean would erase
      // the required 1g +Z reference and corrupt subsequent tilt readings.
      result.calibration = Calibration(Vec3(accelMean_.x, accelMean_.y, accelMean_.z - 1.0f),
                                       gyroMean_, true);
    }
    return result;
  }

 private:
  uint32_t requiredSamples_, count_;
  bool invalidSample_;
  Vec3 accelMean_, gyroMean_, accelM2_, gyroM2_;

  void update(Vec3 value, Vec3 &mean, Vec3 &m2) {
    const float n = static_cast<float>(count_);
    updateAxis(value.x, n, mean.x, m2.x);
    updateAxis(value.y, n, mean.y, m2.y);
    updateAxis(value.z, n, mean.z, m2.z);
  }

  static void updateAxis(float value, float n, float &mean, float &m2) {
    const float delta = value - mean;
    mean += delta / n;
    m2 += delta * (value - mean);
  }

  static Vec3 divideNonnegative(Vec3 value, float denominator) {
    return Vec3(value.x > 0 ? value.x / denominator : 0,
                value.y > 0 ? value.y / denominator : 0,
                value.z > 0 ? value.z / denominator : 0);
  }
  static Vec3 squareRoot(Vec3 value) {
    return Vec3(sqrtf(value.x), sqrtf(value.y), sqrtf(value.z));
  }
  static bool exceedsAxis(Vec3 value, float limit) {
    return value.x > limit || value.y > limit || value.z > limit;
  }
};

}  // namespace motion

#endif
