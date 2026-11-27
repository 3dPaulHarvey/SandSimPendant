#include "../shared/ShakeInput.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

using motion::ShakeInput;
using motion::Vec3;

static Vec3 scaled(Vec3 a, float scale) {
  return Vec3(a.x * scale, a.y * scale, a.z * scale);
}

static void expectNoPulse(ShakeInput &input) {
  const motion::ShakePulse pulse = input.consume();
  assert(input.activity == 0);
  assert(pulse.agitation == 0 && pulse.motionEnergy == 0);
  assert(motion::norm(pulse.linearImpulseGSeconds) == 0);
}

static void stationaryBiasCannotAccumulate() {
  const Vec3 poses[] = {Vec3(0, 0, 1), Vec3(0, 1, 0), Vec3(0.6f, 0, 0.8f)};
  const Vec3 biases[] = {Vec3(3.5f, 0, 0), Vec3(8, -10, 5), Vec3(20, 20, 20), Vec3(0, 0, 35)};
  const float intervals[] = {0.005f, 0.01f, 0.02f, 0.08f};
  float largestResidual = 0;
  for (Vec3 pose : poses) for (Vec3 bias : biases) for (float dt : intervals) {
    ShakeInput input;
    input.update(pose, bias, dt);
    const int count = static_cast<int>(120 / dt);
    for (int i = 1; i <= count; ++i) {
      input.update(pose, bias, dt);
      expectNoPulse(input);
      const float residual = motion::norm(input.latestLinear);
      if (residual > largestResidual) largestResidual = residual;
      // Mathematical bound tau * |omega| * |acceleration|, with float tolerance.
      const float bound = ShakeInput::kResponseSeconds * motion::norm(bias) *
                          0.01745329252f * motion::norm(pose) + 0.0001f;
      assert(residual <= bound);
    }
  }
  printf("Two-minute fixed gyro biases up to 35dps: largest residual %.5fg, zero false impulses.\n",
         largestResidual);
}

static void thermalDriftAndNoiseRemainQuiet() {
  const Vec3 axis = scaled(Vec3(1, -1, 1), 1 / sqrtf(3.0f));
  const float dt = 0.005f;
  ShakeInput input;
  input.update(Vec3(0, 0, 1.15f), Vec3(), dt);
  float largestResidual = 0;
  for (int i = 1; i <= 24000; ++i) {
    const float seconds = i * dt;
    const float bias = 35 * seconds / 120;
    const Vec3 accel(0.002f * sinf(seconds * 17), 0.002f * cosf(seconds * 23),
                     1.15f + 0.002f * sinf(seconds * 31));
    input.update(accel, scaled(axis, bias), dt);
    expectNoPulse(input);
    const float residual = motion::norm(input.latestLinear);
    if (residual > largestResidual) largestResidual = residual;
  }
  printf("Two-minute thermal ramp 0->35dps with 1.15g scale and sensor noise: %.5fg maximum; zero pulses.\n",
         largestResidual);
}

static void motionThenRestAlwaysRecovers() {
  const float dt = 0.005f;
  const Vec3 bias(20, -20, 20);
  ShakeInput input;
  input.update(Vec3(0, 0, 1), bias, dt);
  float energy = 0;
  for (int i = 1; i <= 2000; ++i) {
    const float t = i * dt;
    input.update(Vec3(0.5f * sinf(31.4159265f * t),
                       0.4f * cosf(37.6991118f * t),
                       1 + 0.6f * sinf(50.2654825f * t)), bias, dt);
    energy += input.consume().motionEnergy;
  }
  assert(energy > 1); // Actual 5-8Hz shaking still produces a clear effect.

  // Stop in a new orientation, without changing stored calibration. This
  // intentionally disagrees with the preceding gyro estimate and must settle.
  const Vec3 pose(0.69f, -0.46f, 0.798436f); // Magnitude approximately1.15g.
  for (int i = 1; i <= 24000; ++i) {
    input.update(pose, bias, dt);
    if (i >= 200) expectNoPulse(input); // Fully quiet within one second.
    else input.consume();
  }
  printf("Ten seconds of strong shaking followed by two minutes at a new pose: quiet within 1s.\n");
}

static void constantInputAndAcceptedSpikeCannotLatch() {
  ShakeInput input;
  const float dt = 0.005f;
  input.update(Vec3(0, 0, 1), Vec3(), dt);
  input.update(Vec3(4, -4, 4), Vec3(), dt); // Finite sample within input bounds.
  assert(input.consume().agitation > 0);
  for (int i = 0; i < 200; ++i) {
    input.update(Vec3(0, 0, 1), Vec3(), dt);
    input.consume();
  }
  expectNoPulse(input);

  input.update(Vec3(0.4f, 0, 1), Vec3(), dt);
  assert(input.consume().agitation > 0);
  for (int i = 0; i < 24000; ++i) {
    input.update(Vec3(0.4f, 0, 1), Vec3(), dt);
    if (i >= 200) expectNoPulse(input);
    else input.consume();
  }
  assert(motion::norm(input.latestLinear) < 0.00001f);
  puts("A finite spike or held acceleration cannot latch additional shake/shove energy.");
}

int main() {
  stationaryBiasCannotAccumulate();
  thermalDriftAndNoiseRemainQuiet();
  motionThenRestAlwaysRecovers();
  constantInputAndAcceptedSpikeCannotLatch();
}
