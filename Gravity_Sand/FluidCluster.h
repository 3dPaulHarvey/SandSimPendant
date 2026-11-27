#ifndef GRAVITY_SAND_FLUID_CLUSTER_H
#define GRAVITY_SAND_FLUID_CLUSTER_H

#include <math.h>
#include "SandConfig.h"

namespace sand {
namespace cluster {

struct Point { float x, y; };

inline float neighborhoodWeight(const Point &a, const Point &b, float inverseSupport) {
  const float dx = (a.x - b.x) * inverseSupport;
  const float dy = (a.y - b.y) * inverseSupport;
  const float distanceSquared = dx * dx + dy * dy;
  if (!(distanceSquared < 1.0f)) return 0;
  const float remaining = 1.0f - distanceSquared;
  return remaining * remaining;
}

// Called only for an explosion, not every physics step. The accessor can return
// any particle type exposing x/y; no dynamic allocation or simulation coupling.
template <class ParticleAt>
Point explosionOrigin(int count, float dotRadius, ParticleAt particleAt) {
  const float screenCenter = (hardware::kDisplaySize - 1) * 0.5f;
  const Point fallback = {screenCenter, screenCenter};
  if (count <= 0) return fallback;
  if (count > config::kMaxParticleCount) count = config::kMaxParticleCount;
  if (!isfinite(dotRadius) || dotRadius <= 0 || !isfinite(8.0f * dotRadius)) {
    dotRadius = config::kDotRadius;
  }
  // A 20px neighborhood at the default radius follows the selected dot size.
  const float inverseSupport = 1.0f / (8.0f * dotRadius);
  Point points[config::kMaxParticleCount];
  int validCount = 0;
  for (int i = 0; i < count; ++i) {
    const auto &particle = particleAt(i);
    if (isfinite(particle.x) && isfinite(particle.y)) {
      points[validCount++] = {particle.x, particle.y};
    }
  }
  if (validCount == 0) return fallback;

  int peak = 0;
  float largestDensity = -1;
  for (int i = 0; i < validCount; ++i) {
    float density = 0;
    for (int j = 0; j < validCount; ++j) {
      density += neighborhoodWeight(points[i], points[j], inverseSupport);
    }
    // Equal disconnected clumps pick the first peak, never their midpoint.
    if (density > largestDensity) { largestDensity = density; peak = i; }
  }

  // Center just the winning neighborhood. Positive weights keep the origin
  // inside the particles' convex hull, including groups against the round wall.
  double weightedX = 0, weightedY = 0, totalWeight = 0;
  for (int i = 0; i < validCount; ++i) {
    const float weight = neighborhoodWeight(points[peak], points[i], inverseSupport);
    weightedX += static_cast<double>(weight) * points[i].x;
    weightedY += static_cast<double>(weight) * points[i].y;
    totalWeight += weight;
  }
  if (!(totalWeight > 0)) return points[peak];
  const Point center = {static_cast<float>(weightedX / totalWeight),
                        static_cast<float>(weightedY / totalWeight)};
  if (!isfinite(center.x) || !isfinite(center.y)) return points[peak];

  // A crescent or hollow group must not put the blast in an unoccupied gap.
  // Require fluid within one dot diameter; otherwise keep the actual peak dot.
  const float inverseDiameter = 0.5f / dotRadius;
  for (int i = 0; i < validCount; ++i) {
    const float dx = (center.x - points[i].x) * inverseDiameter;
    const float dy = (center.y - points[i].y) * inverseDiameter;
    if (dx * dx + dy * dy <= 1.0f) return center;
  }
  return points[peak];
}

} // namespace cluster
} // namespace sand
#endif
