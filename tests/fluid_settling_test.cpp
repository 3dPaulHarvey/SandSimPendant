#include "fluid_test_support.h"

static float speedSquared(const Simulation &simulation) {
  float result = 0;
  for (int i = 0; i < Simulation::kParticleCount; ++i) {
    const sand::Particle &p = simulation.particle(i);
    result += p.vx * p.vx + p.vy * p.vy;
  }
  return result / Simulation::kParticleCount;
}

static float meanVelocityX(const Simulation &simulation) {
  float result = 0;
  for (int i = 0; i < Simulation::kParticleCount; ++i)
    result += simulation.particle(i).vx;
  return result / Simulation::kParticleCount;
}

static void smallCoherentMotionMustDissipate() {
  Simulation simulation;
  runFluid(simulation, 0, 0, 10);
  simulation.kick(1, 0, 0);
  const float initial = meanVelocityX(simulation);
  const float damping = 1.0f / (1.0f + sand::config::kAirDamping * fluidDt());
  runFluid(simulation, 0, 0, 5);
  const float expected = initial * powf(damping, 5 * sand::config::kPhysicsHz);
  const float actual = meanVelocityX(simulation);
  printf("Small face-up drift: %.5fpx/s after5s; analytic drag %.5fpx/s\n", actual, expected);
  // Reconstructing all velocity from float positions used to erase drag below
  // the position rounding threshold, keeping low-speed motion alive forever.
  assert(fabsf(actual - expected) < 0.01f);
  runFluid(simulation, 0, 0, 15);
  assert(speedSquared(simulation) < 0.001f);
}

static void excite(Simulation &simulation, int mode) {
  if (mode == 0) {
    simulation.kick(360, 0, 0);
    simulation.kick(360, 0, 0); // Reach the650px/s limit without bypassing the API.
  } else if (mode == 1) {
    simulation.kick(0, 0, sand::config::kMaxKick);
  } else {
    for (int tick = 0; tick < 3 * static_cast<int>(sand::config::kPhysicsHz); ++tick)
      simulation.step(sinf(tick * 0.03f), cosf(tick * 0.03f), fluidDt());
  }
  assert(speedSquared(simulation) > 100);
}

static void stationaryInputMustSettle() {
  const uint32_t seeds[] = {1, 8, 22};
  const float strengths[] = {0, 0.02f, 0.1f, 1};
  float worstBelowFive = 0, worstBelowOne = 0, worstTailRms = 0;
  for (int mode = 0; mode < 3; ++mode) {
    for (float strength : strengths) {
      Simulation simulation;
      simulation.reset(seeds[mode]);
      excite(simulation, mode);
      // Every run holds exactly the same input with NO subsequent shake. One
      // face-up maximum-agitation case continues for2min to catch late bursts.
      const int seconds = mode == 1 && strength == 0 ? 120 : 30;
      int lastAboveFive = 0, lastAboveOne = 0;
      float tailSpeedSquared = 0;
      for (int tick = 1; tick <= seconds * static_cast<int>(sand::config::kPhysicsHz); ++tick) {
        simulation.step(0, strength, fluidDt());
        const float energy = speedSquared(simulation);
        assert(isfinite(energy));
        if (energy > 25) lastAboveFive = tick;
        if (energy > 1) lastAboveOne = tick;
        if (tick >= 20 * static_cast<int>(sand::config::kPhysicsHz) && energy > tailSpeedSquared)
          tailSpeedSquared = energy;
        if (!(tick % sand::config::kPhysicsHz)) checkFluid(simulation);
      }
      const float belowFive = lastAboveFive * fluidDt();
      const float belowOne = lastAboveOne * fluidDt();
      const float tailRms = sqrtf(tailSpeedSquared);
      if (belowFive > 15 || belowOne > 20 || tailRms >= 1) {
        printf("Failed settle seed%lu g=%.2f mode%d: below5=%.3fs below1=%.3fs tail=%.3fpx/s\n",
               static_cast<unsigned long>(seeds[mode]), strength, mode, belowFive, belowOne, tailRms);
        fflush(stdout);
      }
      assert(belowFive <= 15 && belowOne <= 20 && tailRms < 1);
      if (belowFive > worstBelowFive) worstBelowFive = belowFive;
      if (belowOne > worstBelowOne) worstBelowOne = belowOne;
      if (tailRms > worstTailRms) worstTailRms = tailRms;

      // Rest is dissipative, never an input lock or a particle reset.
      if (mode == 0 && strength == 1) {
        Simulation turned = simulation, neutral = simulation;
        runFluid(turned, -1, 0, 0.1f);
        runFluid(neutral, 0, 0, 0.1f);
        const float travel = fluidCenter(neutral, false) - fluidCenter(turned, false);
        assert(travel > 4);
        printf("After long rest, quarter-turn moves %.2fpx in100ms\n", travel);
      }
    }
  }
  printf("12 zero-shake holds: permanently below5px/s by%.2fs, below1 by%.2fs; tail RMS %.3f\n",
         worstBelowFive, worstBelowOne, worstTailRms);
}

int main() {
  smallCoherentMotionMustDissipate();
  stationaryInputMustSettle();
  puts("Velocity drag, shallow/full tilt settling, maximum kicks, post-swirl decay, and2min stability passed.");
}
