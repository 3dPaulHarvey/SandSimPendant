#include "../Gravity_Sand/SandSimulation.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

using sand::Simulation;

static void check(const Simulation &sim) {
  int count = 0;
  bool seen[Simulation::kParticleCount] = {};
  for (int row = 0; row < Simulation::kGridSize; ++row) {
    for (int col = 0; col < Simulation::kGridSize; ++col) {
      const uint16_t id = sim.particleAt(col, row);
      if (!Simulation::inside(col, row)) {
        assert(id == Simulation::kWall);
      } else if (id != Simulation::kEmpty) {
        assert(id < Simulation::kParticleCount);
        assert(!seen[id]);
        seen[id] = true;
        ++count;
        const sand::Particle &p = sim.particle(id);
        assert(p.cell == row * Simulation::kGridSize + col);
        assert(static_cast<int>(floorf(p.x)) == col);
        assert(static_cast<int>(floorf(p.y)) == row);
        // Prove the nonnegative proposal range used by optimized integer
        // conversions, even at the solver's maximum speed and substep.
        assert(col >= 1 && col <= 78 && row >= 1 && row <= 78);
        assert(p.x - 0.751f > 0 && p.x + 0.751f < 80);
        assert(p.y - 0.751f > 0 && p.y + 0.751f < 80);
        assert(isfinite(p.x) && isfinite(p.y));
        assert(isfinite(p.vx) && isfinite(p.vy));
        assert(p.color < 8);
        assert(sim.pixelX(id) == col * 3);
        assert(sim.pixelY(id) == row * 3);
      }
    }
  }
  assert(count == Simulation::kParticleCount);
  assert(sim.particleAt(-1, 0) == Simulation::kWall);
  assert(sim.particleAt(80, 0) == Simulation::kWall);
}

static float centroid(const Simulation &sim, bool y) {
  float sum = 0;
  for (int i = 0; i < Simulation::kParticleCount; ++i) {
    sum += y ? sim.particle(i).y : sim.particle(i).x;
  }
  return sum / Simulation::kParticleCount;
}

static void run(Simulation &sim, float gx, float gy, int frames) {
  for (int i = 0; i < frames; ++i) {
    sim.step(gx, gy, 1.0f / 60.0f);
    if (!(i % 60)) check(sim);
  }
  check(sim);
}

static void snapshot(const Simulation &sim, const char *filename) {
  FILE *out = fopen(filename, "wb");
  assert(out);
  fprintf(out, "P6\n240 240\n255\n");
  for (int y = 0; y < 240; ++y) {
    for (int x = 0; x < 240; ++x) {
      const uint16_t id = sim.particleAt(x / 3, y / 3);
      unsigned char rgb[3] = {0, 0, 0};
      if (id < Simulation::kParticleCount) {
        const uint8_t shade = sim.particle(id).color;
        rgb[0] = 255;
        rgb[1] = static_cast<unsigned char>(shade * 30);
        rgb[2] = static_cast<unsigned char>(shade == 7 ? 64 : 0);
      }
      fwrite(rgb, 1, sizeof(rgb), out);
    }
  }
  fclose(out);
}

int main(int argc, char **) {
  Simulation sim;
  check(sim);
  uint16_t original[Simulation::kParticleCount];
  for (int i = 0; i < Simulation::kParticleCount; ++i) original[i] = sim.particle(i).cell;
  run(sim, 0, 0, 120);
  for (int i = 0; i < Simulation::kParticleCount; ++i) assert(sim.particle(i).cell == original[i]);

  run(sim, 0, 1, 600);
  const float downY = centroid(sim, true);
  assert(downY > 58.0f);
  printf("Down settles: centroid=(%.2f, %.2f) cells\n", centroid(sim, false), downY);
  if (argc > 1) snapshot(sim, "tests/artifacts/sand_down.ppm");
  run(sim, -1, 0, 600);
  assert(centroid(sim, false) < 22.0f);
  printf("Left settles: centroid=(%.2f, %.2f) cells\n", centroid(sim, false), centroid(sim, true));
  if (argc > 1) snapshot(sim, "tests/artifacts/sand_left_after_turn.ppm");
  run(sim, 0, -1, 600);
  assert(centroid(sim, true) < 22.0f);
  printf("Up settles: centroid=(%.2f, %.2f) cells\n", centroid(sim, false), centroid(sim, true));
  run(sim, 0.70710678f, 0.70710678f, 600);
  assert(centroid(sim, false) > 50.0f && centroid(sim, true) > 50.0f);
  printf("Diagonal settles: centroid=(%.2f, %.2f) cells\n", centroid(sim, false), centroid(sim, true));
  if (argc > 1) snapshot(sim, "tests/artifacts/sand_diagonal.ppm");

  for (int frame = 0; frame < 6000; ++frame) {
    const float a = frame * 0.013f;
    const float strength = 0.6f + 0.5f * sinf(frame * 0.017f);
    sim.step(cosf(a) * strength, sinf(a) * strength, (frame % 151 == 0) ? 0.2f : 1.0f / 60.0f);
    if (!(frame % 20)) check(sim);
  }
  check(sim);
  printf("100 seconds of rotating gravity: all %d grains conserved and contained\n", Simulation::kParticleCount);

  Simulation a, b;
  a.reset(42);
  b.reset(42);
  for (int i = 0; i < 100; ++i) {
    a.step(0.3f, -0.8f, 1.0f / 60.0f);
    b.step(0.3f, -0.8f, 1.0f / 60.0f);
  }
  for (int i = 0; i < Simulation::kParticleCount; ++i) {
    assert(a.particle(i).cell == b.particle(i).cell);
    assert(a.particle(i).x == b.particle(i).x);
    assert(a.particle(i).y == b.particle(i).y);
  }
  a.step(NAN, 1, 0.1f);
  a.step(1, NAN, 0.1f);
  a.step(1, 1, NAN);
  a.step(1, 1, -0.1f);
  check(a);
  for (int frame = 0; frame < 600; ++frame) {
    const float directions[8][2] = {
      {-2, -2}, {2, -2}, {2, 2}, {-2, 2},
      {0, -10}, {10, 0}, {0, 10}, {-10, 0}
    };
    const int direction = (frame / 23) % 8;
    a.step(directions[direction][0], directions[direction][1], 0.2f);
    check(a);
  }
  puts("Large-dt/high-gravity turns preserve cast bounds and all grains.");
  puts("Determinism, invalid input, zero-gravity, boundaries, and conservation passed.");
  return 0;
}
