#include "../Gravity_Sand/SandSimulation.h"

// Freeze the old engine as a behavioral baseline, not a timing benchmark.
#undef GRAVITY_SAND_SIMULATION_H
#define sand reference_sand
#include "artifacts/SandSimulation.reference.h"
#undef sand

#include <assert.h>
#include <stdio.h>

template <class Engine>
static float center(const Engine &sim, bool y) {
  float sum = 0;
  for (int i = 0; i < Engine::kParticleCount; ++i)
    sum += y ? sim.particle(i).y : sim.particle(i).x;
  return sum / Engine::kParticleCount;
}

template <class Engine>
static void settle(Engine &sim) {
  sim.reset(42);
  for (int frame = 0; frame < 600; ++frame) sim.step(0, 1, 1.0f / 60);
}

int main() {
  sand::Simulation sim;
  reference_sand::Simulation reference;
  settle(sim);
  settle(reference);
  const float start = center(sim, false), oldStart = center(reference, false);
  for (int frame = 0; frame < 6; ++frame) {
    sim.step(-1, 0, 1.0f / 60);
    reference.step(-1, 0, 1.0f / 60);
  }
  const float travel = start - center(sim, false);
  const float oldTravel = oldStart - center(reference, false);
  printf("Quarter-turn, first100ms: %.4f cells travel; old engine %.4f\n", travel, oldTravel);
  assert(travel > oldTravel * 1.2f);
  assert(travel > 0.10f);

  settle(sim);
  settle(reference);
  const float shallowStart = center(sim, false), oldShallowStart = center(reference, false);
  for (int frame = 0; frame < 12; ++frame) {
    sim.step(-0.1f, 0, 1.0f / 60);
    reference.step(-0.1f, 0, 1.0f / 60);
  }
  const float shallowTravel = shallowStart - center(sim, false);
  const float oldShallowTravel = oldShallowStart - center(reference, false);
  printf("Shallow quarter-turn, first200ms: %.4f cells travel; old engine %.4f\n",
         shallowTravel, oldShallowTravel);
  assert(shallowTravel > oldShallowTravel * 1.2f);
  assert(shallowTravel > 0.05f);

  // A settled pile with minor direction noise must not receive turn boosts.
  settle(sim);
  sand::Simulation quiet = sim;
  for (int frame = 0; frame < 120; ++frame) {
    sim.step((frame & 1) ? 0.005f : -0.005f, 1, 1.0f / 60);
    quiet.step(0, 1, 1.0f / 60);
  }
  assert(fabsf(center(sim, false) - center(quiet, false)) < 0.05f);
  puts("Early pile release improves at full and shallow tilt; small noise stays quiet.");
  return 0;
}
