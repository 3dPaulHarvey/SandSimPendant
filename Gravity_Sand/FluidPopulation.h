#ifndef GRAVITY_SAND_FLUID_POPULATION_H
#define GRAVITY_SAND_FLUID_POPULATION_H

#include "SandSimulation.h"
#include "FluidCoverage.h"

namespace sand {
namespace variation {

struct FittedPopulation {
  float scale;
  int count;
  float coverage;
  int measurements;
};
constexpr int kMaxPopulationMeasurements = 20;
constexpr float kPopulationCoverageTolerance = 0.005f;

// Population fitting runs only on a new layout, never in the frame/physics
// loops. Measure the clipped union, then tune continuous size at a fixed count
// so adding one giant shape cannot jump a normal roll into a crowded layout.
inline FittedPopulation fitPopulation(Simulation &simulation,
                                     render::SpawnCoverage &coverage,
                                     float preferredScale, float targetCoverage,
                                     uint32_t seed, render::Shape shape,
                                     const OddShape &odd = OddShape()) {
  if (!validVisualScale(preferredScale)) preferredScale = config::kMinVisualVariation;
  if (!isfinite(targetCoverage)) targetCoverage = 0.22f;
  if (targetCoverage < config::kMinSpawnCoverage) targetCoverage = config::kMinSpawnCoverage;
  if (targetCoverage > config::kMaxSpawnCoverage) targetCoverage = config::kMaxSpawnCoverage;
  if (!render::validShape(shape)) shape = render::Shape::Circle;
  constexpr float baseRadius = config::kDotRadius * config::kVisualSizeScale;
  const float areaPerScaleSquared = shapeAreaFactor(shape) * baseRadius * baseRadius;
  const float oddAreaDelta = render::validShape(odd.shape) ?
      (shapeAreaFactor(odd.shape) - shapeAreaFactor(shape)) * baseRadius * baseRadius : 0;
  const float targetArea = targetCoverage * render::SpawnCoverage::kScreenArea;
  const auto clampCount = [](int count) -> int {
    if (count < palette::kForegroundCount) return palette::kForegroundCount;
    return count > config::kMaxParticleCount ? config::kMaxParticleCount : count;
  };
  int count = clampCount(static_cast<int>(floorf(
      (targetArea / (preferredScale * preferredScale) - oddAreaDelta) / areaPerScaleSquared)));
  FittedPopulation best = {config::kMinVisualVariation, count, 0, 0};
  float bestError = 2;
  const auto measure = [&](float scale) -> float {
    simulation.configurePacked(scale, count, seed);
    const float fill = coverage.measure(count, simulation.dotRadius() * config::kVisualSizeScale,
      shape, [&](int index) -> const Particle & { return simulation.particle(index); },
      odd.index(count), odd.shape);
    ++best.measurements;
    const float error = fabsf(fill - targetCoverage);
    if (error < bestError) {
      bestError = error;
      best.scale = scale;
      best.count = count;
      best.coverage = fill;
    }
    return fill;
  };
  const auto finish = [&]() -> FittedPopulation {
    if (simulation.sizeScale() != best.scale || simulation.particleCount() != best.count)
      simulation.configurePacked(best.scale, best.count, seed);
    return best;
  };
  // Start close using the known outline area. Sparse layouts usually need
  // only this check and one proportional correction for clipping/overlap.
  const float maximumScale = fminf(config::kMaxVisualVariation,
                                  sqrtf(config::kPackedCountBudget / count));
  float estimate = sqrtf(targetArea / (areaPerScaleSquared * count + oddAreaDelta));
  estimate = fmaxf(config::kMinVisualVariation, fminf(maximumScale, estimate));
  const float estimatedFill = measure(estimate);
  if (bestError <= kPopulationCoverageTolerance) return finish();
  if (estimatedFill > 0) {
    const float corrected = fmaxf(config::kMinVisualVariation,
        fminf(maximumScale, estimate * sqrtf(targetCoverage / estimatedFill)));
    if (corrected != estimate) measure(corrected);
    if (bestError <= kPopulationCoverageTolerance) return finish();
  }
  float low = config::kMinVisualVariation, high = config::kMaxVisualVariation;
  bool bracketed = false;
  // Usually the area estimate brackets immediately. Slender shapes near the
  // largest size can need a few more bodies to compensate for overlap.
  for (int attempt = 0; attempt < 4; ++attempt) {
    low = config::kMinVisualVariation;
    high = fminf(config::kMaxVisualVariation, sqrtf(config::kPackedCountBudget / count));
    const float lowFill = measure(low);
    const float highFill = measure(high);
    if (bestError <= kPopulationCoverageTolerance) return finish();
    if (lowFill <= targetCoverage && highFill >= targetCoverage) { bracketed = true; break; }
    if (lowFill > targetCoverage) {
      count = clampCount(static_cast<int>(floorf(count * targetCoverage / lowFill)) - 1);
    } else {
      count = clampCount(static_cast<int>(ceilf(count * targetCoverage / highFill * 1.2f)));
    }
  }
  if (!bracketed) {
    // A bounded fallback starts below the target even at the minimum size.
    // Its large-size endpoint has ample coverage for every supported shape.
    count = clampCount(static_cast<int>(floorf((targetArea /
        (config::kMinVisualVariation * config::kMinVisualVariation) - oddAreaDelta) /
        areaPerScaleSquared)) - 1);
    low = config::kMinVisualVariation;
    high = fminf(config::kMaxVisualVariation, sqrtf(config::kPackedCountBudget / count));
    const float lowFill = measure(low), highFill = measure(high);
    bracketed = lowFill <= targetCoverage && highFill >= targetCoverage;
  }
  if (bracketed) {
    // At most 20 measurements including the analytic start and fallback.
    // Half a percentage point is enough; do not spend time chasing pixels.
    for (int iteration = 0; iteration < 8 && bestError > kPopulationCoverageTolerance; ++iteration) {
      const float middle = (low + high) * 0.5f;
      const float fill = measure(middle);
      if (fill < targetCoverage) low = middle;
      else high = middle;
    }
  }
  return finish();
}

} // namespace variation
} // namespace sand
#endif
