#include "../Gravity_Sand/SandSimulation.h"

#include <assert.h>
#include <stdio.h>

using sand::Simulation;

static void check(const Simulation &sim) {
  bool seen[Simulation::kParticleCount] = {};
  int count = 0;
  for (int row = 0; row < Simulation::kGridSize; ++row) {
    for (int col = 0; col < Simulation::kGridSize; ++col) {
      const uint16_t id = sim.particleAt(col, row);
      if (!Simulation::inside(col, row)) assert(id == Simulation::kWall);
      else if (id != Simulation::kEmpty) {
        assert(id < Simulation::kParticleCount && !seen[id]);
        seen[id] = true;
        ++count;
        const sand::Particle &p = sim.particle(id);
        assert(p.cell == row * Simulation::kGridSize + col);
        assert(isfinite(p.x) && isfinite(p.y) && isfinite(p.vx) && isfinite(p.vy));
        assert(static_cast<int>(floorf(p.x)) == col);
        assert(static_cast<int>(floorf(p.y)) == row);
        assert(p.vx * p.vx + p.vy * p.vy < 90.01f * 90.01f);
        assert(p.x - 0.751f > 0 && p.x + 0.751f < 80);
        assert(p.y - 0.751f > 0 && p.y + 0.751f < 80);
      }
    }
  }
  assert(count == Simulation::kParticleCount);
}

static float centerX(const Simulation &sim) {
  float sum = 0;
  for (int i = 0; i < Simulation::kParticleCount; ++i) sum += sim.particle(i).x;
  return sum / Simulation::kParticleCount;
}

static void equal(const Simulation &a, const Simulation &b) {
  for (int i = 0; i < Simulation::kParticleCount; ++i) {
    const auto &p = a.particle(i), &q = b.particle(i);
    assert(p.x == q.x && p.y == q.y && p.vx == q.vx && p.vy == q.vy);
    assert(p.cell == q.cell && p.color == q.color && p.slide == q.slide);
  }
}

int main() {
  Simulation quiet, zeroKick;
  for (int i = 0; i < 240; ++i) {
    zeroKick.kick(0, 0, 0);
    quiet.step(0.3f, 0.8f, 1.0f / 60);
    zeroKick.step(0.3f, 0.8f, 1.0f / 60);
    equal(quiet, zeroKick); // Includes RNG order, not only no visible drift.
  }
  Simulation positive, negative;
  const float initialX = centerX(positive);
  positive.kick(25, 0, 0);
  negative.kick(-25, 0, 0);
  for (int i = 0; i < 12; ++i) {
    positive.step(0, 0, 1.0f / 60);
    negative.step(0, 0, 1.0f / 60);
  }
  assert(centerX(positive) > initialX + 0.5f);
  assert(centerX(negative) < initialX - 0.5f);
  check(positive);
  check(negative);

  Simulation settled;
  for (int i = 0; i < 600; ++i) settled.step(0, 1, 1.0f / 60);
  Simulation jostled = settled;
  jostled.kick(0, 0, 18);
  int mostChanged = 0, mostLofted = 0;
  for (int frame = 0; frame < 30; ++frame) {
    jostled.step(0, 1, 1.0f / 60);
    int changed = 0, lofted = 0;
    for (int i = 0; i < Simulation::kParticleCount; ++i) {
      if (jostled.particle(i).cell != settled.particle(i).cell) ++changed;
      if (jostled.pixelY(i) < settled.pixelY(i)) ++lofted;
    }
    if (changed > mostChanged) mostChanged = changed;
    if (lofted > mostLofted) mostLofted = lofted;
    check(jostled);
  }
  assert(mostChanged >= 10 && mostLofted >= 3);
  printf("Single Z agitation=18 moves %d settled grains; %d lift above their old row.\n", mostChanged, mostLofted);

  Simulation repeated = settled;
  int repeatedChanged = 0;
  for (int frame = 0; frame < 12; ++frame) {
    repeated.kick(0, 0, 18);
    repeated.step(0, 1, 1.0f / 60);
    check(repeated);
  }
  for (int i = 0; i < Simulation::kParticleCount; ++i)
    if (repeated.particle(i).cell != settled.particle(i).cell) ++repeatedChanged;
  assert(repeatedChanged >= 25);
  printf("Twelve Z agitation=18 frames move %d settled grains.\n", repeatedChanged);

  // Pair-balanced Z jitter injects no coherent XY momentum in a fresh field.
  Simulation balanced;
  balanced.kick(0, 0, 20);
  float sumX = 0, sumY = 0;
  for (int i = 0; i < Simulation::kParticleCount; ++i) {
    sumX += balanced.particle(i).vx;
    sumY += balanced.particle(i).vy;
  }
  assert(fabsf(sumX) < 0.001f && fabsf(sumY) < 0.001f);

  Simulation invalid, unchanged = invalid;
  invalid.kick(NAN, 0, 1);
  invalid.kick(0, INFINITY, 1);
  invalid.kick(1, 0, NAN);
  equal(invalid, unchanged);
  invalid.step(0, 1, 1.0f / 60);
  unchanged.step(0, 1, 1.0f / 60);
  equal(invalid, unchanged);
  for (int frame = 0; frame < 1200; ++frame) {
    const float angle = frame * 0.037f;
    jostled.kick((frame & 1) ? 1e30f : -1e30f, (frame & 2) ? 1e30f : -1e30f, 1e30f);
    check(jostled);
    jostled.step(cosf(angle) * 2, sinf(angle) * 2, (frame % 31) ? 1.0f / 60 : 0.2f);
    check(jostled);
  }
  puts("Shake impulses: zero activity, directional response, visible Z jostling, paired momentum,");
  puts("nonfinite rejection, velocity limits, conservation, and long harsh-input containment passed.");
  return 0;
}
