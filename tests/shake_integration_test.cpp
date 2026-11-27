#include "../shared/ShakeInput.h"
#include "fluid_test_support.h"

using motion::Vec3;

struct Harness {
  motion::ShakeInput input;
  Simulation simulation;
  double physicsAccumulator = 0;
  float maxAgitation = 0;
  int activeFrames = 0, physicsFrames = 0;
  bool boost;

  explicit Harness(bool enhanced = true) : boost(enhanced) {}
  void clearStats() { maxAgitation = 0; activeFrames = physicsFrames = 0; }

  void sample(Vec3 acceleration, Vec3 gyroDps, float dt) {
    input.update(acceleration, gyroDps, dt);
    physicsAccumulator += dt;
    const double step = 1.0 / sand::config::kPhysicsHz;
    while (physicsAccumulator + 1e-9 >= step) {
      const motion::ShakePulse pulse = input.consume();
      assert(isfinite(pulse.agitation) && pulse.agitation >= 0 && pulse.agitation <= 18.001f);
      if (pulse.agitation > maxAgitation) maxAgitation = pulse.agitation;
      if (pulse.agitation > 0.001f) ++activeFrames;
      ++physicsFrames;
      if (boost) {
        const motion::Vec2 shove = motion::mapGravity(pulse.linearImpulseGSeconds);
        const float scale = sand::config::kAcceleration * sand::config::kShoveGain * sand::config::kShakeGain;
        simulation.kick(shove.x * scale, shove.y * scale,
                        pulse.agitation * sand::config::kShakeGain * sand::config::kShakePixelScale);
      }
      const motion::Vec2 gravity = motion::mapGravity(acceleration);
      simulation.step(gravity.x, gravity.y, fluidDt());
      checkFluid(simulation);
      physicsAccumulator -= step;
    }
  }

  void rest(Vec3 acceleration, int rate, float seconds) {
    const int count = static_cast<int>(rate * seconds + 0.5f);
    for (int i = 0; i < count; ++i) sample(acceleration, Vec3(), 1.0f / rate);
  }
};

static void flatRestHasNoShake(int rate) {
  Harness enhanced, baseline(false);
  enhanced.rest(Vec3(0, 0, 1), rate, 1);
  baseline.rest(Vec3(0, 0, 1), rate, 1);
  assert(enhanced.activeFrames == 0 && enhanced.maxAgitation == 0);
  equalFluid(enhanced.simulation, baseline.simulation);
}

static void normalShakeMovesFluid(int rate) {
  Harness enhanced;
  runFluid(enhanced.simulation, 0, 1, 4);
  enhanced.rest(Vec3(0, 1, 0), rate, 0.8f);
  assert(enhanced.maxAgitation == 0);
  Harness baseline(false);
  baseline.simulation = enhanced.simulation;
  baseline.input = enhanced.input;
  baseline.physicsAccumulator = enhanced.physicsAccumulator;
  enhanced.clearStats();
  float maxSeparation = 0;
  int mostLofted = 0;
  const float pi = 3.14159265358979323846f;
  for (int i = 1; i <= rate; ++i) {
    const float seconds = static_cast<float>(i) / rate;
    // Gravity remains down; normal-Z translation must act through agitation.
    const Vec3 acceleration(0, 1, 0.6f * sinf(2 * pi * 7 * seconds));
    enhanced.sample(acceleration, Vec3(), 1.0f / rate);
    baseline.sample(acceleration, Vec3(), 1.0f / rate);
    const float separation = fluidSeparation(enhanced.simulation, baseline.simulation);
    if (separation > maxSeparation) maxSeparation = separation;
    int lofted = 0;
    for (int p = 0; p < Simulation::kParticleCount; ++p)
      if (enhanced.simulation.particle(p).y < baseline.simulation.particle(p).y - 2) ++lofted;
    if (lofted > mostLofted) mostLofted = lofted;
  }
  printf("%dHz Z shake: mean displacement %.2fpx, %d dots loft2px, input peak %.2f\n",
         rate, maxSeparation, mostLofted, enhanced.maxAgitation);
  assert(enhanced.activeFrames >= 20 && enhanced.maxAgitation > 1);
  assert(maxSeparation > 1 && mostLofted >= 5);
  enhanced.rest(Vec3(0, 1, 0), rate, 0.5f);
  enhanced.clearStats();
  enhanced.rest(Vec3(0, 1, 0), rate, 0.2f);
  assert(enhanced.maxAgitation < 0.01f && enhanced.activeFrames == 0);
}

static void lateralShakeAddsDirectionalMotion(int rate) {
  Harness enhanced;
  runFluid(enhanced.simulation, 0, 1, 4);
  enhanced.rest(Vec3(0, 1, 0), rate, 0.8f);
  enhanced.clearStats();
  Harness baseline(false);
  baseline.simulation = enhanced.simulation;
  baseline.input = enhanced.input;
  baseline.physicsAccumulator = enhanced.physicsAccumulator;
  float enhancedPath = 0, baselinePath = 0;
  float lastEnhanced = fluidCenter(enhanced.simulation, false);
  float lastBaseline = lastEnhanced;
  const float pi = 3.14159265358979323846f;
  for (int i = 1; i <= rate; ++i) {
    const float seconds = static_cast<float>(i) / rate;
    const Vec3 acceleration(0.5f * sinf(2 * pi * 5 * seconds), 1, 0);
    enhanced.sample(acceleration, Vec3(), 1.0f / rate);
    baseline.sample(acceleration, Vec3(), 1.0f / rate);
    const float xEnhanced = fluidCenter(enhanced.simulation, false);
    const float xBaseline = fluidCenter(baseline.simulation, false);
    enhancedPath += fabsf(xEnhanced - lastEnhanced);
    baselinePath += fabsf(xBaseline - lastBaseline);
    lastEnhanced = xEnhanced; lastBaseline = xBaseline;
  }
  printf("%dHz lateral shake: centroid path %.2fpx force-only / %.2fpx enhanced\n",
         rate, baselinePath, enhancedPath);
  assert(enhancedPath > baselinePath * 1.2f);
}

static void slowTiltDoesNotShake(int rate) {
  Harness harness;
  harness.rest(Vec3(0, 0, 1), rate, 0.8f);
  harness.clearStats();
  const float pi = 3.14159265358979323846f;
  for (int i = 1; i <= rate * 3; ++i) {
    const float angle = static_cast<float>(i) / rate * pi / 6;
    harness.sample(Vec3(0, sinf(angle), cosf(angle)), Vec3(30, 0, 0), 1.0f / rate);
  }
  printf("%dHz pure90deg tilt: peak false agitation %.6f; active frames %d\n",
         rate, harness.maxAgitation, harness.activeFrames);
  assert(harness.maxAgitation < 0.01f && harness.activeFrames == 0);
}

int main() {
  const int rates[] = {180, 200};
  for (int rate : rates) {
    flatRestHasNoShake(rate);
    normalShakeMovesFluid(rate);
    lateralShakeAddsDirectionalMotion(rate);
    slowTiltDoesNotShake(rate);
  }
  printf("Shake pipeline,240Hz fluid consumption, containment, and quiet input passed.\n");
}
