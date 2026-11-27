#include <cassert>
#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <limits>
#include "../shared/ShakeInput.h"

using motion::ShakeInput;
using motion::ShakePulse;
using motion::Vec3;

static const float kPi = 3.14159265358979323846f;

static bool near(float a, float b, float tolerance = 0.0002f) {
  return std::fabs(a - b) <= tolerance;
}

static Vec3 rotateGravity(Vec3 axis, float angle) {
  const float length = motion::norm(axis);
  axis = Vec3(axis.x / length, axis.y / length, axis.z / length);
  // Exact world-up gravity represented in a chip rotated +angle about axis.
  const float c = std::cos(angle), s = -std::sin(angle);
  return Vec3(axis.y * s + axis.x * axis.z * (1 - c),
              -axis.x * s + axis.y * axis.z * (1 - c),
              c + axis.z * axis.z * (1 - c));
}

static void restingNoise() {
  ShakeInput input;
  input.update(Vec3(0, 0, 1), Vec3(), 0.005f);
  for (int i = 1; i <= 4000; ++i) {
    const float t = i * 0.005f;
    input.update(Vec3(0.002f * std::sin(31 * t),
                      0.002f * std::cos(43 * t),
                      1 + 0.003f * std::sin(57 * t)),
                 Vec3(0.04f * std::sin(13 * t), 0.05f * std::cos(17 * t), 0.03f),
                 0.005f);
    assert(input.activity == 0);
    assert(input.consume().agitation == 0);
    assert(motion::norm(input.latestLinear) < 0.015f);
  }
}

static void rotationRejectsShake() {
  const float rates[] = {30, -60, 360, -720};
  const Vec3 axes[] = {Vec3(0, 1, 0), Vec3(1, 0, 0), Vec3(0.3f, 0.8f, 0.5f)};
  const float intervals[] = {1.0f / 180, 0.02f, 0.08f};
  for (float rate : rates) for (Vec3 axis : axes) for (float dt : intervals) {
    ShakeInput input;
    input.update(Vec3(0, 0, 1), Vec3(), dt);
    const float length = motion::norm(axis);
    const Vec3 gyro(axis.x / length * rate, axis.y / length * rate, axis.z / length * rate);
    for (int i = 1; i <= 250; ++i) {
      const float angle = rate * kPi / 180 * dt * i;
      input.update(rotateGravity(axis, angle), gyro, dt);
      assert(motion::norm(input.latestLinear) < 0.002f);
      assert(input.activity == 0);
      assert(input.consume().agitation == 0);
    }
  }
}

static void verticalAndLateralMotion() {
  ShakeInput normal, lateral;
  normal.update(Vec3(0, 0, 1), Vec3(), 0.005f);
  lateral.update(Vec3(0, 0, 1), Vec3(), 0.005f);
  float normalEnergy = 0, lateralEnergy = 0;
  for (int i = 0; i < 100; ++i) {
    normal.update(Vec3(0, 0, 1.4f), Vec3(), 0.005f);
    lateral.update(Vec3(0.4f, 0, 1), Vec3(), 0.005f);
    normalEnergy += normal.consume().motionEnergy;
    lateralEnergy += lateral.consume().motionEnergy;
    if (i == 0) {
      assert(normal.latestLinear.z > 0.38f);
      assert(lateral.latestLinear.x > 0.38f);
    }
    assert(motion::isFinite(normal.latestLinear) && motion::isFinite(lateral.latestLinear));
  }
  assert(normalEnergy > lateralEnergy * 4.5f);
  // A sustained/DC push is not an endless source of additional shake energy.
  assert(normal.activity == 0 && lateral.activity == 0);
  assert(motion::norm(normal.latestLinear) < 0.003f);
  assert(motion::norm(lateral.latestLinear) < 0.003f);

  ShakeInput oscillation;
  oscillation.update(Vec3(0, 0, 1), Vec3(), 0.005f);
  float energy = 0;
  for (int i = 1; i <= 1000; ++i) {
    const float z = 0.6f * std::sin(2 * kPi * 8 * i * 0.005f);
    oscillation.update(Vec3(0, 0, 1 + z), Vec3(), 0.005f);
    assert(near(oscillation.latestLinear.x, 0) && near(oscillation.latestLinear.y, 0));
    energy += oscillation.consume().motionEnergy;
  }
  assert(energy > 1);
}

static void shortPulseAndNoReleaseTail() {
  ShakeInput input;
  input.update(Vec3(0, 0, 1), Vec3(), 1.0f / 180);
  assert(input.consume().agitation == 0);
  input.update(Vec3(0, 0, 2), Vec3(), 1.0f / 180);
  for (int i = 0; i < 20; ++i) {
    input.update(Vec3(0, 0, 1), Vec3(), 1.0f / 180);
    assert(input.activity == 0);
  }
  const ShakePulse pulse = input.consume();
  assert(pulse.agitation > 5 && pulse.agitation <= 18);
  assert(pulse.motionEnergy > 0 && pulse.durationSeconds > 0.1f);
  assert(input.consume().agitation == 0);
  input.update(Vec3(0, 0, 1), Vec3(), 1.0f / 180);
  assert(input.consume().agitation == 0);
}

static double sampledEnergy(int sampleHz, int consumeHz) {
  ShakeInput input;
  const float dt = 1.0f / sampleHz;
  input.update(Vec3(0, 0, 1), Vec3(), dt);
  double energy = 0;
  int nextConsume = 1;
  for (int i = 1; i <= sampleHz * 4; ++i) {
    const float t = i * dt;
    input.update(Vec3(0, 0, 1 + 0.25f * std::sin(2 * kPi * 5 * t)), Vec3(), dt);
    while (static_cast<double>(i) / sampleHz + 1e-9 >=
           static_cast<double>(nextConsume) / consumeHz) {
      const ShakePulse pulse = input.consume();
      energy += pulse.agitation * pulse.agitation;
      assert(near(pulse.agitation * pulse.agitation, pulse.motionEnergy * 60, 0.00002f));
      ++nextConsume;
    }
  }
  const ShakePulse last = input.consume();
  return energy + last.agitation * last.agitation;
}

static void rateIndependentEnergy() {
  const double reference = sampledEnergy(200, 60);
  assert(reference > 1);
  for (int sampleHz : {100, 180, 200}) for (int consumeHz : {30, 60, 120, 240}) {
    const double energy = sampledEnergy(sampleHz, consumeHz);
    assert(std::fabs(energy - reference) / reference < 0.015);
  }
}

static void staleInvalidAndBoundedInput() {
  ShakeInput input;
  input.update(Vec3(0, 0, 1), Vec3(), 0.005f);
  for (int i = 0; i < 1000; ++i)
    input.update((i & 1) ? Vec3(4, 4, 4) : Vec3(-4, -4, -4), Vec3(), 0.005f);
  assert(near(input.consume().agitation, 18));
  input.update(Vec3(0, 0, 2), Vec3(), 0.005f);
  input.update(Vec3(0, 0, 1), Vec3(), 0.101f);
  assert(input.consume().agitation == 0 && input.activity == 0);
  assert(motion::norm(input.latestLinear) == 0);

  const float bad[] = {0, -0.1f, std::numeric_limits<float>::infinity(),
                       std::numeric_limits<float>::quiet_NaN()};
  for (float dt : bad) {
    input.update(Vec3(0, 0, 2), Vec3(), dt);
    assert(input.consume().agitation == 0 && input.activity == 0);
  }
  input.update(Vec3(NAN, 0, 1), Vec3(), 0.005f);
  assert(input.consume().agitation == 0 && motion::isFinite(input.latestLinear));
  input.update(Vec3(0, 0, 1), Vec3(INFINITY, 0, 0), 0.005f);
  assert(input.consume().agitation == 0 && motion::isFinite(input.latestLinear));
  input.update(Vec3(100, 0, 1), Vec3(), 0.005f);
  assert(input.consume().agitation == 0);
}

static void stableRestRecovery() {
  const Vec3 initial[] = {Vec3(0.5f, 0, 1), Vec3(0, 0, -1)};
  for (Vec3 acceleration : initial) {
    ShakeInput input;
    input.update(acceleration, Vec3(), 0.005f);
    assert(input.consume().agitation == 0); // Initialization never kicks.
    for (int i = 0; i < 1600; ++i) {
      input.update(Vec3(0, 0, 1), Vec3(), 0.005f);
      input.consume();
    }
    assert(input.activity == 0);
    assert(motion::norm(input.latestLinear) < 0.002f);
  }
}

static void coherentImpulseIntegration() {
  ShakeInput input;
  input.update(Vec3(0, 0, 1), Vec3(), 0.005f);
  input.update(Vec3(0.4f, 0, 1), Vec3(), 0.005f);
  // A brief push survives quiet samples before the next physics tick.
  for (int i = 0; i < 2; ++i) input.update(Vec3(0, 0, 1), Vec3(), 0.005f);
  ShakePulse pulse = input.consume();
  assert(pulse.linearImpulseGSeconds.x > 0.0015f && pulse.linearImpulseGSeconds.x < 0.0017f);
  assert(near(pulse.linearImpulseGSeconds.y, 0) && near(pulse.linearImpulseGSeconds.z, 0));
  assert(motion::norm(input.consume().linearImpulseGSeconds) == 0);

  // Equal and opposite brief pushes mostly cancel even with leaky memory.
  input.update(Vec3(0.4f, 0, 1), Vec3(), 0.005f);
  input.update(Vec3(-0.4f, 0, 1), Vec3(), 0.005f);
  assert(motion::norm(input.consume().linearImpulseGSeconds) < 0.0003f);

  input.update(Vec3(0, 0, 1.6f), Vec3(), 0.005f);
  pulse = input.consume();
  assert(pulse.linearImpulseGSeconds.x == 0 && pulse.linearImpulseGSeconds.y == 0);
  assert(pulse.linearImpulseGSeconds.z > 0.0024f && pulse.linearImpulseGSeconds.z < 0.0027f);

  // Summed impulses are in g*s, independent of sampling/consumption rate.
  float referenceImpulse = 0;
  for (int sampleHz : {180, 200}) {
    ShakeInput sampled;
    const float dt = 1.0f / sampleHz;
    sampled.update(Vec3(0, 0, 1), Vec3(), dt);
    float totalX = 0;
    int nextConsume = 1;
    for (int i = 1; i <= sampleHz; ++i) {
      sampled.update(Vec3(0.4f, 0, 1), Vec3(), dt);
      if (static_cast<double>(i) / sampleHz + 1e-9 >= static_cast<double>(nextConsume) / 60) {
        totalX += sampled.consume().linearImpulseGSeconds.x;
        ++nextConsume;
      }
    }
    totalX += sampled.consume().linearImpulseGSeconds.x;
    // The extra effect acts on the onset, then decays; direct acceleration is
    // already handled by the main tilt path. A constant1s push cannot keep
    // injecting additional shake/shove after the100ms memory has decayed.
    assert(totalX > 0.018f && totalX < 0.022f);
    assert(sampled.activity == 0 && motion::norm(sampled.latestLinear) < 0.0001f);
    if (referenceImpulse > 0) assert(near(totalX, referenceImpulse, 0.0001f));
    referenceImpulse = totalX;
  }

  for (int i = 0; i < 1000; ++i) input.update(Vec3(0.4f, 0, 1), Vec3(), 0.005f);
  assert(motion::norm(input.consume().linearImpulseGSeconds) <= 0.100001f);
  input.update(Vec3(0.4f, 0, 1), Vec3(), 0.005f);
  input.update(Vec3(0, 0, 1), Vec3(), 0.11f);
  assert(motion::norm(input.consume().linearImpulseGSeconds) == 0);

  // Exact gyro-described tilting creates neither random nor coherent kicks.
  input.reset();
  input.update(Vec3(0, 0, 1), Vec3(), 0.005f);
  for (int i = 1; i <= 400; ++i) {
    input.update(rotateGravity(Vec3(0, 1, 0), i * 0.005f * kPi * 2),
                 Vec3(0, 360, 0), 0.005f);
    assert(motion::norm(input.consume().linearImpulseGSeconds) == 0);
  }
}

static void stillPoseMagnitudeRecovery() {
  for (float magnitude : {0.9f, 1.0f, 1.1f, 1.15f}) {
    ShakeInput input;
    input.update(Vec3(0, 0, 1), Vec3(), 0.005f);
    const Vec3 pose(-0.62f * magnitude, 0.08f * magnitude,
                    std::sqrt(1 - 0.62f * 0.62f - 0.08f * 0.08f) * magnitude);
    for (int i = 0; i < 1600; ++i) {
      input.update(pose, Vec3(), 0.005f);
      input.consume();
    }
    assert(input.activity == 0 && motion::norm(input.latestLinear) < 0.002f);
    input.update(pose, Vec3(), 0.005f);
    const ShakePulse pulse = input.consume();
    assert(pulse.agitation == 0 && motion::norm(pulse.linearImpulseGSeconds) == 0);
  }
}

int main() {
  restingNoise();
  rotationRejectsShake();
  verticalAndLateralMotion();
  shortPulseAndNoReleaseTail();
  rateIndependentEnergy();
  staleInvalidAndBoundedInput();
  stableRestRecovery();
  coherentImpulseIntegration();
  stillPoseMagnitudeRecovery();
  std::puts("Shake input tests passed: quiet rest, gyro-compensated tilt, transient XYZ motion, pulse capture, rate-independent energy, and bounded invalid/stale recovery.");
}
