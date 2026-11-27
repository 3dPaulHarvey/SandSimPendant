#include "fluid_test_support.h"
#include <float.h>

int main() {
  static_assert(Simulation::kParticleCount == 100, "The initial fluid experiment uses 100 dots");
  Simulation simulation;
  checkFluid(simulation);

  // A repeatable seed reproduces every particle, including colors and impulses.
  Simulation a, b;
  a.reset(42);
  b.reset(42);
  equalFluid(a, b);
  for (int frame = 0; frame < 1200; ++frame) {
    const float angle = frame * 0.013f;
    if (!(frame % 47)) {
      a.kick(70, -80, 25);
      b.kick(70, -80, 25);
    }
    a.step(cosf(angle), sinf(angle), fluidDt());
    b.step(cosf(angle), sinf(angle), fluidDt());
    equalFluid(a, b);
    checkFluid(a);
  }
  a.reset(42);
  b.reset(42);
  equalFluid(a, b);

  const Simulation unchanged = a;
  a.step(NAN, 1, 0.1f);
  a.step(1, INFINITY, 0.1f);
  a.step(1, 1, NAN);
  a.step(1, 1, INFINITY);
  a.step(1, 1, 0);
  a.step(1, 1, -0.1f);
  a.step(1, 1, 1e-30f);
  equalFluid(a, unchanged);

  // Long frame stalls advance only the documented bounded amount of time.
  a.step(0.6f, 0.8f, 1000);
  b.step(0.6f, 0.8f, sand::config::kMaxFrameSeconds);
  equalFluid(a, b);

  const float directions[8][2] = {
    {-1, -1}, {1, -1}, {1, 1}, {-1, 1},
    {0, -1}, {1, 0}, {0, 1}, {-1, 0}
  };
  for (int frame = 0; frame < 12000; ++frame) {
    const float angle = frame * 0.017f;
    const int direction = (frame / 23) % 8;
    if (!(frame % 29)) simulation.kick(cosf(angle) * 350, sinf(angle) * 350, 80);
    if (!(frame % 151)) {
      simulation.step(directions[direction][0] * FLT_MAX,
                      directions[direction][1] * FLT_MAX, 10);
    } else {
      simulation.step(cosf(angle) * 2, sinf(angle) * 2, fluidDt());
    }
    checkFluid(simulation);
  }
  printf("%d fluid steps: all 100 dots finite, speed bounded, and fully inside the circular wall.\n", 12000);
  puts("Deterministic reset, rejected invalid input, and capped stall duration passed.");
}
