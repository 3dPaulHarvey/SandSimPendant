#include "fluid_test_support.h"

int main() {
  Simulation settled;
  settled.reset(42);
  runFluid(settled, 0, 1, 4);
  const float bottom = fluidCenter(settled, true);
  printf("Downward rest: centroid=(%.2f, %.2f)px, mean speed=%.2fpx/s\n",
         fluidCenter(settled, false), bottom, fluidSpeed(settled));
  assert(bottom > 180);

  // Compare with a neutral continuation to exclude residual settling motion.
  Simulation full = settled, shallow = settled, neutral = settled;
  runFluid(full, -1, 0, 0.1f);
  runFluid(neutral, 0, 0, 0.1f);
  const float fullTravel = fluidCenter(neutral, false) - fluidCenter(full, false);
  printf("Full quarter-turn response: %.2fpx left in100ms\n", fullTravel);
  assert(fullTravel > 4);

  neutral = settled;
  runFluid(shallow, -0.12f, 0, 0.2f);
  runFluid(neutral, 0, 0, 0.2f);
  const float shallowTravel = fluidCenter(neutral, false) - fluidCenter(shallow, false);
  printf("Shallow0.12g turn: %.2fpx left in200ms\n", shallowTravel);
  assert(shallowTravel > 1);

  Simulation reverse = settled;
  runFluid(reverse, 0, -1, 0.1f);
  const float reverseTravel = bottom - fluidCenter(reverse, true);
  printf("Gravity reversal: %.2fpx up in100ms\n", reverseTravel);
  assert(reverseTravel > 4);

  runFluid(full, -1, 0, 3);
  assert(fluidCenter(full, false) < 60);
  runFluid(full, 0, -1, 3);
  assert(fluidCenter(full, true) < 60);
  runFluid(full, 1, 0, 3);
  assert(fluidCenter(full, false) > 180);

  // A lateral shove into the curved wall must loft fluid above the resting pool.
  Simulation splash = settled;
  splash.kick(350, 0, 0);
  int maxLofted = 0;
  float top = 240;
  for (int i = 0; i < static_cast<int>(sand::config::kPhysicsHz); ++i) {
    splash.step(1, 0, fluidDt());
    checkFluid(splash);
    int lofted = 0;
    for (int p = 0; p < Simulation::kParticleCount; ++p) {
      if (splash.particle(p).y < settled.particle(p).y - 25) ++lofted;
      if (splash.particle(p).y < top) top = splash.particle(p).y;
    }
    if (lofted > maxLofted) maxLofted = lofted;
  }
  printf("Curved-wall splash: %d dots rise25px, highest y=%.2fpx\n", maxLofted, top);
  assert(maxLofted >= 20);
  assert(top < 140);
  puts("Fast full/shallow tilt, gravity reversal, four-wall flow, and curved-wall splash passed.");
}
