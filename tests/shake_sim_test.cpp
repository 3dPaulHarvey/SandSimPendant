#include "fluid_test_support.h"
#include <float.h>

int main() {
  Simulation quiet, zeroKick;
  for (int i = 0; i < 480; ++i) {
    zeroKick.kick(0, 0, 0);
    zeroKick.kick(0, 0, -FLT_MAX);
    quiet.step(0.3f, 0.8f, fluidDt());
    zeroKick.step(0.3f, 0.8f, fluidDt());
    equalFluid(quiet, zeroKick);
  }

  Simulation positive, negative;
  positive.kick(100, 0, 0);
  negative.kick(-100, 0, 0);
  runFluid(positive, 0, 0, 0.1f);
  runFluid(negative, 0, 0, 0.1f);
  const float separation = fluidCenter(positive, false) - fluidCenter(negative, false);
  printf("Opposite100px/s kicks separate centroids by %.2fpx in100ms\n", separation);
  assert(separation > 10);

  Simulation settled;
  runFluid(settled, 0, 1, 4);
  Simulation jostled = settled, baseline = settled;
  jostled.kick(0, 0, 150);
  float largestSeparation = 0;
  int mostLofted = 0;
  for (int frame = 0; frame < 120; ++frame) {
    jostled.step(0, 1, fluidDt());
    baseline.step(0, 1, fluidDt());
    const float distance = fluidSeparation(jostled, baseline);
    if (distance > largestSeparation) largestSeparation = distance;
    int lofted = 0;
    for (int i = 0; i < Simulation::kParticleCount; ++i)
      if (jostled.particle(i).y < baseline.particle(i).y - 3) ++lofted;
    if (lofted > mostLofted) mostLofted = lofted;
    checkFluid(jostled);
  }
  printf("Single150px/s agitation: mean separation %.2fpx; %d dots loft3px\n",
         largestSeparation, mostLofted);
  assert(largestSeparation > 2 && mostLofted >= 3);

  // Balanced normal-Z impulses must not secretly add directional momentum.
  Simulation balanced;
  balanced.kick(0, 0, 100);
  float sumX = 0, sumY = 0;
  for (int i = 0; i < Simulation::kParticleCount; ++i) {
    sumX += balanced.particle(i).vx;
    sumY += balanced.particle(i).vy;
  }
  assert(fabsf(sumX) < 0.01f && fabsf(sumY) < 0.01f);

  Simulation invalid, unchanged = invalid;
  invalid.kick(NAN, 0, 1);
  invalid.kick(0, INFINITY, 1);
  invalid.kick(1, 0, NAN);
  equalFluid(invalid, unchanged);
  invalid.step(0, 1, fluidDt());
  unchanged.step(0, 1, fluidDt());
  equalFluid(invalid, unchanged);

  for (int frame = 0; frame < 2400; ++frame) {
    const float angle = frame * 0.037f;
    jostled.kick((frame & 1) ? FLT_MAX : -FLT_MAX,
                 (frame & 2) ? FLT_MAX : -FLT_MAX, FLT_MAX);
    checkFluid(jostled);
    jostled.step(cosf(angle) * 2, sinf(angle) * 2,
                 (frame % 31) ? fluidDt() : 0.2f);
    checkFluid(jostled);
  }
  puts("Zero impulse equivalence, directional kicks, agitation, balanced momentum,");
  puts("invalid input rejection, and extreme impulse containment passed.");
}
