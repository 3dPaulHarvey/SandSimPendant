#include "../Gravity_Sand/SandSimulation.h"

#include <assert.h>
#include <initializer_list>
#include <stdio.h>

using sand::Simulation;

static void check(const Simulation &simulation) {
  assert(simulation.packed());
  assert(simulation.sizeScale() >= sand::config::kMinVisualVariation);
  assert(simulation.sizeScale() <= sand::config::kMaxPackedScale);
  assert(simulation.particleCount() >= sand::config::kMinParticleCount);
  assert(simulation.particleCount() <= Simulation::maxPackedCountForScale(simulation.sizeScale()));
  const float radius = simulation.dotRadius();
  assert(fabsf(radius - sand::config::kDotRadius * simulation.sizeScale()) < 0.0001f);
  assert(fabsf(simulation.boundaryRadius() + radius - Simulation::kCircleRadius) < 0.0001f);
  const float boundary = simulation.boundaryRadius() + 0.002f;
  const float maximum = sand::config::kMaxSpeed + 0.02f;
  for (int i = 0; i < simulation.particleCount(); ++i) {
    const sand::Particle &p = simulation.particle(i);
    assert(isfinite(p.x) && isfinite(p.y) && isfinite(p.vx) && isfinite(p.vy));
    const float x = p.x - Simulation::kCenter, y = p.y - Simulation::kCenter;
    assert(x * x + y * y <= boundary * boundary);
    assert(p.vx * p.vx + p.vy * p.vy <= maximum * maximum);
    assert(p.color < 8);
  }
}

static void same(const Simulation &first, const Simulation &second) {
  assert(first.packed() == second.packed());
  assert(first.sizeScale() == second.sizeScale());
  assert(first.particleCount() == second.particleCount());
  assert(first.dotRadius() == second.dotRadius());
  assert(first.boundaryRadius() == second.boundaryRadius());
  for (int i = 0; i < first.particleCount(); ++i) {
    const sand::Particle &a = first.particle(i), &b = second.particle(i);
    assert(a.x == b.x && a.y == b.y && a.vx == b.vx && a.vy == b.vy && a.color == b.color);
  }
}

static float minimumDistance(const Simulation &simulation) {
  float minimum2 = 1e9f;
  for (int i = 0; i < simulation.particleCount(); ++i)
    for (int j = 0; j < i; ++j) {
      const float dx = simulation.particle(i).x - simulation.particle(j).x;
      const float dy = simulation.particle(i).y - simulation.particle(j).y;
      const float distance2 = dx * dx + dy * dy;
      if (distance2 < minimum2) minimum2 = distance2;
    }
  return sqrtf(minimum2);
}

static float speed(const Simulation &simulation) {
  float squared = 0;
  for (int i = 0; i < simulation.particleCount(); ++i) {
    const sand::Particle &p = simulation.particle(i);
    squared += p.vx * p.vx + p.vy * p.vy;
  }
  return sqrtf(squared / simulation.particleCount());
}

static float centerX(const Simulation &simulation) {
  float sum = 0;
  for (int i = 0; i < simulation.particleCount(); ++i) sum += simulation.particle(i).x;
  return sum / simulation.particleCount();
}

static void advance(Simulation &simulation, float gx, float gy, int steps) {
  for (int i = 0; i < steps; ++i) {
    simulation.step(gx, gy, 1.0f / sand::config::kPhysicsHz);
    check(simulation);
  }
}

static void packedLimitsAndReset() {
  assert(Simulation::maxPackedCountForScale(1) == 120);
  assert(Simulation::maxPackedCountForScale(2) == 120);
  assert(Simulation::maxPackedCountForScale(4) == 120);
  assert(Simulation::maxPackedCountForScale(6) == 67);
  assert(Simulation::maxPackedCountForScale(10) == 24);
  assert(Simulation::maxPackedCountForScale(NAN) == 0);
  assert(Simulation::maxPackedCountForScale(INFINITY) == 0);
  assert(Simulation::maxPackedCountForScale(0) == 0);
  assert(Simulation::maxPackedCountForScale(0.99f) == 0);
  assert(Simulation::maxPackedCountForScale(10.01f) == 0);
  const float scales[] = {1, 2, 4, 7, 10};
  for (float scale : scales) {
    for (uint32_t seed : {1u, 0xABCD1234u}) {
      Simulation simulation, initial;
      const int count = Simulation::maxPackedCountForScale(scale);
      assert(simulation.configurePacked(scale, count, seed));
      assert(initial.configurePacked(scale, count, seed));
      same(simulation, initial);
      check(simulation);
      assert(minimumDistance(simulation) > simulation.dotRadius());
      int inside = 0, outside = 0;
      for (int i = 0; i < count; ++i) {
        const sand::Particle &p = simulation.particle(i);
        const float x = p.x - Simulation::kCenter, y = p.y - Simulation::kCenter;
        const float distance = sqrtf(x * x + y * y);
        // The new layout fills a disk instead of collapsing a large square
        // patch onto its wall. Both its interior and outer half are populated.
        assert(distance < simulation.boundaryRadius());
        if (distance < simulation.boundaryRadius() * 0.5f) ++inside;
        if (distance > simulation.boundaryRadius() * 0.75f) ++outside;
      }
      assert(inside >= count / 5 && outside >= count / 3);
      assert(!simulation.configurePacked(scale, count + 1, seed));
      same(simulation, initial);
      for (float invalid : {NAN, INFINITY, -1.0f, 0.99f, 10.01f}) {
        assert(!simulation.configurePacked(invalid, 10, seed));
        same(simulation, initial);
      }
      assert(!simulation.configurePacked(scale, 0, seed));
      same(simulation, initial);
      for (int direction = 0; direction < 4; ++direction) {
        simulation.explode(Simulation::kCenter, Simulation::kCenter, sand::config::kExplosionMaxImpulse);
        const float gx = direction == 0 ? 1.0f : (direction == 2 ? -1.0f : 0.0f);
        const float gy = direction == 1 ? 1.0f : (direction == 3 ? -1.0f : 0.0f);
        advance(simulation, gx, gy, 120);
      }
      simulation.reset(seed);
      same(simulation, initial);
      assert(simulation.configure(1, 100, seed));
      assert(!simulation.packed());
      assert(initial.configure(1, 100, seed));
      same(simulation, initial);
    }
  }
  puts("Packed scales 1x-10x remain finite, contained and deterministic under dense explosions and four-way tilt.");
}

static void crowdedMotion() {
  Simulation sparse, dense;
  assert(sparse.configurePacked(10, 8, 1234));
  assert(dense.configurePacked(10, 24, 1234));
  const float initialSparseDistance = minimumDistance(sparse);
  const float initialDenseDistance = minimumDistance(dense);
  assert(initialDenseDistance < initialSparseDistance);
  assert(initialDenseDistance < dense.dotRadius() * 2); // Intentional physical crowding.
  advance(sparse, 0, 1, 2400);
  advance(dense, 0, 1, 2400);
  const float sparseStart = centerX(sparse), denseStart = centerX(dense);
  const float denseQuietSpeed = speed(dense);
  advance(sparse, 1, 0, 120);
  advance(dense, 1, 0, 120);
  const float sparseTravel = fabsf(centerX(sparse) - sparseStart);
  const float denseTravel = fabsf(centerX(dense) - denseStart);
  assert(denseQuietSpeed < 1.0f);
  assert(sparseTravel > 5.0f && denseTravel < 1.0f);
  printf("1000%% spacing: %.2fpx sparse / %.2fpx dense against 50px physical diameter; quiet dense RMS %.2fpx/s.\n",
         initialSparseDistance, initialDenseDistance, denseQuietSpeed);
  printf("Half-second side tilt: sparse center travel %.2fpx; dense %.2fpx.\n",
         sparseTravel, denseTravel);
}

int main() {
  packedLimitsAndReset();
  crowdedMotion();
}
