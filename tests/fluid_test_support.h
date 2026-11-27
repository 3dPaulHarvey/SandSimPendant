#ifndef SIMCIRCLE_FLUID_TEST_SUPPORT_H
#define SIMCIRCLE_FLUID_TEST_SUPPORT_H

#include "../Gravity_Sand/SandSimulation.h"
#include <assert.h>
#include <stdio.h>

using sand::Simulation;
inline float fluidDt() { return 1.0f / sand::config::kPhysicsHz; }

inline void checkFluid(const Simulation &simulation) {
  const float center = (Simulation::kDisplaySize - 1) * 0.5f;
  const float radius = simulation.dotRadius();
  const float limit = simulation.boundaryRadius() + 0.002f;
  assert(simulation.particleCount() >= 3 && simulation.particleCount() <= Simulation::kMaxParticleCount);
  const float minimumScale = simulation.packed() ? sand::config::kMinVisualVariation : sand::config::kMinSizeScale;
  const float maximumScale = simulation.packed() ? sand::config::kMaxPackedScale : sand::config::kMaxSizeScale;
  assert(isfinite(simulation.sizeScale()) && simulation.sizeScale() >= minimumScale &&
         simulation.sizeScale() <= maximumScale);
  assert(fabsf(radius - sand::config::kDotRadius * simulation.sizeScale()) < 0.0001f);
  assert(fabsf(simulation.boundaryRadius() + radius - Simulation::kCircleRadius) < 0.0001f);
  for (int i = 0; i < simulation.particleCount(); ++i) {
    const sand::Particle &p = simulation.particle(i);
    assert(isfinite(p.x) && isfinite(p.y) && isfinite(p.vx) && isfinite(p.vy));
    const float dx = p.x - center, dy = p.y - center;
    assert(dx * dx + dy * dy <= limit * limit);
    const float speedLimit = sand::config::kMaxSpeed + 0.02f;
    assert(p.vx * p.vx + p.vy * p.vy <= speedLimit * speedLimit);
    assert(p.color < 8);
  }
}

inline void equalFluid(const Simulation &a, const Simulation &b) {
  assert(a.particleCount() == b.particleCount());
  assert(a.packed() == b.packed());
  assert(a.sizeScale() == b.sizeScale() && a.dotRadius() == b.dotRadius());
  assert(a.boundaryRadius() == b.boundaryRadius());
  for (int i = 0; i < a.particleCount(); ++i) {
    const sand::Particle &p = a.particle(i), &q = b.particle(i);
    assert(p.x == q.x && p.y == q.y && p.vx == q.vx && p.vy == q.vy);
    assert(p.color == q.color);
  }
}

inline float fluidCenter(const Simulation &simulation, bool y) {
  float sum = 0;
  for (int i = 0; i < simulation.particleCount(); ++i)
    sum += y ? simulation.particle(i).y : simulation.particle(i).x;
  return sum / simulation.particleCount();
}

inline float fluidSpeed(const Simulation &simulation) {
  float sum = 0;
  for (int i = 0; i < simulation.particleCount(); ++i) {
    const sand::Particle &p = simulation.particle(i);
    sum += sqrtf(p.vx * p.vx + p.vy * p.vy);
  }
  return sum / simulation.particleCount();
}

inline void runFluid(Simulation &simulation, float gx, float gy, float seconds) {
  const int steps = static_cast<int>(seconds * sand::config::kPhysicsHz + 0.5f);
  for (int i = 0; i < steps; ++i) {
    simulation.step(gx, gy, fluidDt());
    checkFluid(simulation);
  }
}

inline float fluidSeparation(const Simulation &a, const Simulation &b) {
  assert(a.particleCount() == b.particleCount());
  float sum = 0;
  for (int i = 0; i < a.particleCount(); ++i) {
    const float dx = a.particle(i).x - b.particle(i).x;
    const float dy = a.particle(i).y - b.particle(i).y;
    sum += sqrtf(dx * dx + dy * dy);
  }
  return sum / a.particleCount();
}
#endif
