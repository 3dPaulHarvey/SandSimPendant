#include "../shared/MotionCalibration.h"
#include "../shared/BoardConfig.h"

#include <assert.h>
#include <float.h>
#include <stdio.h>

using namespace motion;

static bool near(float actual, float expected, float tolerance = 0.00002f) {
  return fabsf(actual - expected) <= tolerance;
}

static CalibrationResult stationary(Vec3 accel, Vec3 gyro = Vec3()) {
  StationaryCalibrator collector;
  for (int i = 0; i < 400; ++i) assert(collector.add(accel, gyro));
  assert(collector.ready());
  assert(collector.count() == 400);
  collector.add(Vec3(10, 10, 10), Vec3(500, 500, 500));
  assert(collector.count() == 400); // A ready collector stays frozen.
  return collector.finish();
}

int main() {
  CalibrationResult flat = stationary(Vec3(0, 0, 1));
  assert(flat.ok());
  Vec3 corrected = correctAcceleration(Vec3(0, 0, 1), flat.calibration);
  assert(near(corrected.x, 0) && near(corrected.y, 0) && near(corrected.z, 1));
  assert(near(norm(corrected), 1));

  const Vec3 accelBias(0.13f, -0.07f, 0.04f), gyroBias(0.6f, -0.2f, 0.1f);
  CalibrationResult biased = stationary(Vec3(accelBias.x, accelBias.y, 1 + accelBias.z), gyroBias);
  assert(biased.ok());
  assert(near(biased.calibration.accelBias.x, accelBias.x));
  assert(near(biased.calibration.accelBias.y, accelBias.y));
  assert(near(biased.calibration.accelBias.z, accelBias.z));
  corrected = correctAcceleration(Vec3(0.73f, -0.07f, 0.84f), biased.calibration);
  assert(near(corrected.x, 0.6f) && near(corrected.y, 0) && near(corrected.z, 0.8f));
  assert(near(norm(corrected), 1));
  assert(near(norm(correctGyro(gyroBias, biased.calibration)), 0));
  assert(near(biased.accelVariance.x, 0) && near(biased.gyroStd.z, 0));

  StationaryCalibrator shake;
  for (int i = 0; i < 400; ++i) shake.add(Vec3((i & 1) ? 0.05f : -0.05f, 0, 1), Vec3());
  assert(shake.finish().error == CalibrationError::AccelMoving);
  assert(near(shake.finish().accelVariance.x, 0.0025f * 400 / 399, 0.000001f));
  shake.reset();
  for (int i = 0; i < 400; ++i) shake.add(Vec3(0, 0, 1), Vec3(0, (i & 1) ? 2 : -2, 0));
  assert(shake.finish().error == CalibrationError::GyroMoving);
  assert(stationary(Vec3(0, 0, 1), Vec3(20, 0, 0)).ok());
  assert(stationary(Vec3(0, 0, 1), Vec3(0, 0, 40)).error == CalibrationError::GyroTooFast);
  assert(stationary(Vec3(1, 0, 0)).error == CalibrationError::NotFlat);
  assert(stationary(Vec3(0, 0, -1)).error == CalibrationError::NotFlat);
  assert(stationary(Vec3(0, 0, 0.5f)).error == CalibrationError::GravityMagnitude);
  assert(stationary(Vec3(0.3f, 0, 0.95f)).error == CalibrationError::NotFlat);

  StationaryCalibrator invalid;
  assert(invalid.finish().error == CalibrationError::TooFewSamples);
  assert(!invalid.add(Vec3(NAN, 0, 1), Vec3()));
  assert(invalid.finish().error == CalibrationError::InvalidSample);
  invalid.reset();
  assert(!invalid.add(Vec3(0, 0, 1), Vec3(0, INFINITY, 0)));
  assert(invalid.finish().error == CalibrationError::InvalidSample);
  invalid.reset();
  assert(!invalid.add(Vec3(FLT_MAX, FLT_MAX, FLT_MAX), Vec3()));

  float filtered200 = 0, filtered100 = 0, filteredVariable = 0;
  for (int i = 0; i < 200; ++i) filtered200 += smoothingAlpha(0.005f, 0.045f) * (1 - filtered200);
  for (int i = 0; i < 100; ++i) filtered100 += smoothingAlpha(0.010f, 0.045f) * (1 - filtered100);
  // A shorter transient verifies the complete response, beyond saturation.
  float short200 = 0, short100 = 0;
  for (int i = 0; i < 20; ++i) short200 += smoothingAlpha(0.005f, 0.045f) * (1 - short200);
  for (int i = 0; i < 10; ++i) short100 += smoothingAlpha(0.010f, 0.045f) * (1 - short100);
  for (int i = 0; i < 5; ++i) {
    filteredVariable += smoothingAlpha(0.003f, 0.045f) * (1 - filteredVariable);
    filteredVariable += smoothingAlpha(0.017f, 0.045f) * (1 - filteredVariable);
  }
  assert(near(filtered200, filtered100));
  assert(near(short200, short100) && near(short200, filteredVariable));
  assert(near(short200, 1 - expf(-0.1f / 0.045f)));
  assert(smoothingAlpha(-1, 0.1f) == 0 && smoothingAlpha(NAN, 1) == 0);
  assert(smoothingAlpha(0.01f, 0) == 1);

  Vec2 gravity = mapGravity(Vec3(0.3f, -0.4f, 0.8f));
  assert(near(gravity.x, -0.3f) && near(gravity.y, -0.4f));
  gravity = mapGravity(Vec3(0.3f, -0.4f, 0.8f), true, true, true);
  assert(near(gravity.x, -0.4f) && near(gravity.y, -0.3f));

  // Prior COM33 mount: screen gravity = (-ay,-ax). Synthetic readings for
  // left, toward the viewer, right, away, diagonal, and flat.
  const Vec3 previousMountedReadings[] = {
    Vec3(0, 1, 0), Vec3(-1, 0, 0), Vec3(0, -1, 0), Vec3(1, 0, 0),
    Vec3(-0.3f, 0.4f, 0.8f), Vec3(0, 0, 1)
  };
  const Vec2 expectedDirections[] = {
    Vec2(-1, 0), Vec2(0, 1), Vec2(1, 0), Vec2(0, -1),
    Vec2(-0.4f, 0.3f), Vec2(0, 0)
  };
  for (unsigned i = 0; i < sizeof(previousMountedReadings) / sizeof(previousMountedReadings[0]); ++i) {
    const Vec3 previous = previousMountedReadings[i];
    const Vec2 previousMapped = mapGravity(previous, true, false, true);
    assert(near(previousMapped.x, expectedDirections[i].x));
    assert(near(previousMapped.y, expectedDirections[i].y));

    // COM35's connector stays at the top while the module turns front-to-back.
    // Model that physical rotation about screen vertical as (x,-y,-z).
    // The new defaults must preserve each screen direction, including flat
    // negative Z, without changing the raw chip-coordinate measurements.
    const Vec3 faceFlipped(previous.x, -previous.y, -previous.z);
    const Vec2 mapped = mapGravity(faceFlipped, hardware::kImuSwapAxes,
                                   hardware::kImuFlipX, hardware::kImuFlipY);
    assert(near(mapped.x, expectedDirections[i].x));
    assert(near(mapped.y, expectedDirections[i].y));
  }

  Vec3 limited = clampNorm(Vec3(3, 4, 0), 2);
  assert(near(norm(limited), 2) && near(limited.x, 1.2f) && near(limited.y, 1.6f));
  limited = clampNorm(Vec3(FLT_MAX, FLT_MAX, FLT_MAX), 2);
  assert(isFinite(limited) && near(norm(limited), 2));
  assert(near(norm(clampNorm(Vec3(NAN, 1, 1), 2)), 0));
  const Vec3 raw(0.1f, 0.2f, 0.3f);
  assert(correctAcceleration(raw, Calibration()).x == raw.x);
  puts("Motion calibration: 1g preservation, bias correction, stationary/motion/orientation rejection,");
  puts("Welford variance, finite bounds, mounted tilt directions, and rate-independent smoothing passed.");
  return 0;
}
