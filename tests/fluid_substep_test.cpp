#include "fluid_test_support.h"

#include <float.h>

static Simulation independentParticles(float scale, float speed) {
  Simulation simulation;
  assert(simulation.configurePacked(scale, 3, 1));
  const float points[3][2] = {{119.5f, 119.5f}, {55, 65}, {185, 65}};
  for (int i = 0; i < 3; ++i) {
    sand::Particle &p = const_cast<sand::Particle &>(simulation.particle(i));
    p = {points[i][0], points[i][1], i == 0 ? speed : 0, 0,
         static_cast<uint8_t>(i)};
  }
  return simulation;
}

static void assertOneFullStep(float scale, float speed, float gx = 0) {
  Simulation simulation = independentParticles(scale, speed);
  const float dt = fluidDt();
  const float expectedVelocity = (speed + gx * sand::config::kAcceleration * dt) /
                                (1 + sand::config::kAirDamping * dt);
  const float expectedPosition = simulation.particle(0).x + expectedVelocity * dt;
  simulation.step(gx, 0, dt);
  // Independent bodies with no wall/neighbor interaction have a closed-form
  // full-step result. A gratuitous second solve would change both quantities.
  assert(simulation.particle(0).vx == expectedVelocity);
  assert(simulation.particle(0).x == expectedPosition);
  checkFluid(simulation);
}

static void ordinaryAndScaledMotionKeepsOneStep() {
  assertOneFullStep(1, 0);
  assertOneFullStep(1, 300);
  assertOneFullStep(1, 650);
  assertOneFullStep(1, 1000);
  assertOneFullStep(1, 1000, 2); // Gravity still leaves this below the travel bound.
  assertOneFullStep(1.2f, 1200);
  assertOneFullStep(2, 1200);
  puts("Ordinary small-particle motion and full-speed larger particles retain one240Hz solve.");
}

static void fastBurstMatchesTwoHalfSteps() {
  const float speeds[] = {1040, 1200};
  for (float speed : speeds) {
    Simulation adaptive = independentParticles(1, speed), reference = adaptive;
    adaptive.step(0, 0, fluidDt());
    reference.step(0, 0, fluidDt() * 0.5f);
    reference.step(0, 0, fluidDt() * 0.5f);
    equalFluid(adaptive, reference);
    checkFluid(adaptive);
    const float fullStepVelocity = speed / (1 + sand::config::kAirDamping * fluidDt());
    assert(adaptive.particle(0).vx != fullStepVelocity);
  }

  // Current speed alone is under the threshold; the next gravity increment
  // crosses it. Including acceleration must select the two-step reference.
  Simulation accelerated = independentParticles(1, 1018), reference = accelerated;
  accelerated.step(2, 0, fluidDt());
  reference.step(2, 0, fluidDt() * 0.5f);
  reference.step(2, 0, fluidDt() * 0.5f);
  equalFluid(accelerated, reference);
  checkFluid(accelerated);
  puts("Fast small-particle bursts match two explicit half-steps, including upcoming gravity acceleration.");
}

static void largeFrameSlicesRemainBounded() {
  Simulation whole = independentParticles(1, 1200), explicitSlices = whole, stalled = whole;
  const float duration = sand::config::kMaxFrameSeconds;
  const int slices = static_cast<int>(ceilf(duration / sand::config::kMaxSubstepSeconds - 0.00001f));
  const float dt = duration / slices;
  whole.step(0, 0, duration);
  stalled.step(0, 0, 1000);
  for (int i = 0; i < slices; ++i) explicitSlices.step(0, 0, dt);
  equalFluid(whole, explicitSlices);
  equalFluid(whole, stalled);
  checkFluid(whole);
  const Simulation before = whole;
  whole.step(NAN, 0, dt);
  whole.step(0, INFINITY, dt);
  whole.step(0, 0, NAN);
  whole.step(0, 0, 0);
  equalFluid(whole, before);
  puts("Large frames preserve explicit bounded slices, stall caps and invalid-input behavior.");
}

static void repeatedMaximumBlastsRemainContained() {
  const float scales[] = {1, 1.1f, 1.2f, 3.5f, 10};
  for (float scale : scales) {
    Simulation simulation;
    assert(simulation.configurePacked(scale, Simulation::maxPackedCountForScale(scale), 71));
    for (int tick = 0; tick < 720; ++tick) {
      if (tick % 60 == 0) {
        const int origin = (tick / 60) % simulation.particleCount();
        simulation.explode(simulation.particle(origin).x, simulation.particle(origin).y,
                           sand::config::kExplosionMaxImpulse);
      }
      const int direction = (tick / 180) % 4;
      const float gx[] = {1, 0, -1, 0}, gy[] = {0, 1, 0, -1};
      simulation.step(gx[direction], gy[direction], fluidDt());
      checkFluid(simulation);
    }
  }
  puts("Repeated1200px/s dense blasts stay finite, speed-bounded and inside the round wall at1x-10x sizes.");
}

int main() {
  ordinaryAndScaledMotionKeepsOneStep();
  fastBurstMatchesTwoHalfSteps();
  largeFrameSlicesRemainBounded();
  repeatedMaximumBlastsRemainContained();
}
