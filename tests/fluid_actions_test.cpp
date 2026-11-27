#include "fluid_test_support.h"

#include <float.h>
#include <limits.h>

static void explosionIsAnOutwardImpulse() {
  const float scales[] = {1, 2.5f, 5};
  for (float scale : scales) {
    Simulation simulation;
    assert(simulation.configure(scale, Simulation::maxCountForScale(scale), 73));
    const Simulation before = simulation;
    const float ox = Simulation::kCenter, oy = Simulation::kCenter;
    simulation.explode(ox, oy, 500);
    float nearestDistance = FLT_MAX, furthestDistance = 0;
    float nearestSpeed = 0, furthestSpeed = 0;
    for (int i = 0; i < simulation.particleCount(); ++i) {
      const auto &p = simulation.particle(i), &q = before.particle(i);
      assert(p.x == q.x && p.y == q.y && p.color == q.color);
      const float dx = p.x - ox, dy = p.y - oy;
      const float distance = sqrtf(dx * dx + dy * dy);
      const float speed = sqrtf(p.vx * p.vx + p.vy * p.vy);
      if (distance > 0.001f) {
        assert(dx * p.vx + dy * p.vy > 0);
        assert(fabsf(dx * p.vy - dy * p.vx) < distance * speed * 0.00001f);
        assert(speed > 0);
        if (distance < nearestDistance) { nearestDistance = distance; nearestSpeed = speed; }
        if (distance > furthestDistance) { furthestDistance = distance; furthestSpeed = speed; }
      }
    }
    assert(nearestDistance < furthestDistance);
    assert(nearestSpeed > furthestSpeed); // Actual distance falloff, not uniform random agitation.
    checkFluid(simulation);
  }
  puts("Explosion impulse is radial with distance falloff; positions and colors are unchanged.");
}

static void maximumExplosionDoublesTheOldMaximum() {
  assert(sand::config::kExplosionMinImpulse == 600);
  assert(sand::config::kExplosionMaxImpulse == 1200);
  const float scales[] = {1, 2.5f, 3.5f, 10};
  for (int packed = 0; packed < 2; ++packed) {
    for (float scale : scales) {
      if (!packed && scale > sand::config::kMaxSizeScale) continue;
      Simulation minimum, maximum;
      if (packed)
        assert(minimum.configurePacked(scale, Simulation::maxPackedCountForScale(scale), 73));
      else
        assert(minimum.configure(scale, Simulation::maxCountForScale(scale), 73));
      maximum = minimum;
      const Simulation before = minimum;
      const float ox = before.particle(0).x, oy = before.particle(0).y;
      minimum.explode(ox, oy, 600);
      maximum.explode(ox, oy, 1200);
      assert(maximum.particleCount() == before.particleCount());
      assert(maximum.dotRadius() == before.dotRadius());
      assert(maximum.sizeScale() == before.sizeScale());
      for (int i = 0; i < maximum.particleCount(); ++i) {
        const auto &old = before.particle(i), &low = minimum.particle(i), &high = maximum.particle(i);
        assert(high.x == old.x && high.y == old.y && high.color == old.color);
        assert(low.x == old.x && low.y == old.y && low.color == old.color);
        assert(fabsf(high.vx - 2 * low.vx) < 0.002f);
        assert(fabsf(high.vy - 2 * low.vy) < 0.002f);
      }
      const auto &nearest = maximum.particle(0);
      assert(sqrtf(nearest.vx * nearest.vx + nearest.vy * nearest.vy) > 1199);
      checkFluid(minimum);
      checkFluid(maximum);
    }
  }
  // The full impulse must survive integration, not just the explode() call.
  Simulation isolated;
  assert(isolated.configurePacked(1, 3, 7));
  isolated.explode(isolated.particle(0).x, isolated.particle(0).y, 1200);
  isolated.step(0, 0, fluidDt());
  const auto &moving = isolated.particle(0);
  assert(sqrtf(moving.vx * moving.vx + moving.vy * moving.vy) > 1100);
  checkFluid(isolated);
  puts("200% explosions double 100% velocities from rest across sparse/dense sizes; near-origin 1200px/s survives integration.");
}

static void edgeCoincidenceAndInvalidActions() {
  const float scale = 5;
  Simulation a, b;
  assert(a.configure(scale, Simulation::maxCountForScale(scale), 42));
  b = a;
  a.explode(NAN, 0, 500);
  a.explode(0, INFINITY, 500);
  a.explode(0, 0, NAN);
  a.explode(0, 0, INFINITY);
  a.explode(0, 0, 0);
  a.explode(0, 0, -100);
  equalFluid(a, b);
  const float ox = a.particle(0).x, oy = a.particle(0).y;
  a.explode(ox, oy, 500);
  b.explode(ox, oy, 500);
  equalFluid(a, b);
  assert(a.particle(0).vx * a.particle(0).vx + a.particle(0).vy * a.particle(0).vy > 0);
  checkFluid(a);

  const float origins[5][2] = {
    {Simulation::kCenter - 117, Simulation::kCenter},
    {Simulation::kCenter + 117, Simulation::kCenter},
    {Simulation::kCenter, Simulation::kCenter - 117},
    {Simulation::kCenter, Simulation::kCenter + 117},
    {FLT_MAX, -FLT_MAX}
  };
  for (const auto &origin : origins) {
    a.explode(origin[0], origin[1], FLT_MAX);
    checkFluid(a);
    runFluid(a, 0.5f, 0.5f, 0.05f);
  }

  const Simulation original = a;
  assert(!a.configure(NAN, 5, 1));
  assert(!a.configure(INFINITY, 5, 1));
  assert(!a.configure(0.99f, 100, 1));
  assert(!a.configure(5.01f, 3, 1));
  assert(!a.configure(1, 2, 1));
  assert(!a.configure(1, 121, 1));
  assert(!a.configure(1, INT_MAX, 1));
  assert(!a.configure(5, 6, 1));
  equalFluid(a, original);
  puts("Coincident and wall origins remain finite; invalid actions preserve the complete active state.");
}

static uint32_t randomBits(uint32_t &state) {
  state ^= state << 13; state ^= state >> 17; state ^= state << 5;
  return state;
}

static void repeatedResizeAndExplode() {
  Simulation a, b;
  uint32_t rng = 391;
  for (int action = 0; action < 1000; ++action) {
    const uint32_t seed = randomBits(rng);
    const float scale = 1 + 4 * (randomBits(rng) & 65535) / 65535.0f;
    const int count = Simulation::maxCountForScale(scale);
    switch (action % 8) {
      case 0:
        assert(a.configure(1, 120, seed)); assert(b.configure(1, 120, seed));
        break;
      case 1:
        a.explode(Simulation::kCenter, Simulation::kCenter, sand::config::kExplosionMaxImpulse);
        b.explode(Simulation::kCenter, Simulation::kCenter, sand::config::kExplosionMaxImpulse);
        break;
      case 2:
        assert(a.configure(5, 3, seed)); assert(b.configure(5, 3, seed));
        break;
      case 3:
        a.explode(a.particle(0).x, a.particle(0).y, sand::config::kExplosionMaxImpulse);
        b.explode(b.particle(0).x, b.particle(0).y, sand::config::kExplosionMaxImpulse);
        break;
      case 4:
        assert(a.configure(scale, count, seed)); assert(b.configure(scale, count, seed));
        break;
      case 5:
        a.explode(2.5f, Simulation::kCenter, sand::config::kExplosionMaxImpulse);
        b.explode(2.5f, Simulation::kCenter, sand::config::kExplosionMaxImpulse);
        break;
      case 6:
        a.kick(FLT_MAX, -FLT_MAX, FLT_MAX);
        b.kick(FLT_MAX, -FLT_MAX, FLT_MAX);
        break;
      default:
        a.reset(seed); b.reset(seed);
        break;
    }
    equalFluid(a, b);
    checkFluid(a);
    uint8_t colors[Simulation::kMaxParticleCount];
    for (int i = 0; i < a.particleCount(); ++i) colors[i] = a.particle(i).color;
    for (int step = 0; step < 6; ++step) {
      const float angle = action * 0.07f + step * 0.01f;
      a.step(sinf(angle), cosf(angle), fluidDt());
      b.step(sinf(angle), cosf(angle), fluidDt());
      equalFluid(a, b);
      checkFluid(a);
      for (int i = 0; i < a.particleCount(); ++i) assert(a.particle(i).color == colors[i]);
    }
  }
  puts("1,000 mixed resize/reset/explosion actions preserve active counts, colors, containment, and determinism.");
}

static void oneExplosionDissipates() {
  const float scales[] = {1, 3, 5};
  for (float scale : scales) {
    Simulation simulation;
    assert(simulation.configure(scale, Simulation::maxCountForScale(scale), 42));
    simulation.explode(Simulation::kCenter, Simulation::kCenter, sand::config::kExplosionMaxImpulse);
    assert(fluidSpeed(simulation) > 100);
    runFluid(simulation, 0, 0, 20);
    const float speed = fluidSpeed(simulation);
    printf("Single explosion at %.1fx size: %.4fpx/s after 20s without another action.\n", scale, speed);
    assert(speed < 1);
  }
}

int main() {
  explosionIsAnOutwardImpulse();
  maximumExplosionDoublesTheOldMaximum();
  edgeCoincidenceAndInvalidActions();
  repeatedResizeAndExplode();
  oneExplosionDissipates();
}
