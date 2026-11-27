#ifndef GRAVITY_SAND_FLUID_VARIATION_H
#define GRAVITY_SAND_FLUID_VARIATION_H

#include <math.h>
#include <stdint.h>
#include "SandConfig.h"
#include "FluidShapes.h"
#include "FluidPalettes.h"

namespace sand {
namespace variation {
struct CountRange { int baseline, low, high; };
struct Population { float scale, targetCoverage; }; // A request, loosely fitted to visible area.

// One optional visual exception, selected once and kept stable for this pile.
// A slot key maps uniformly across the fitted count without storing an array.
struct OddShape {
  render::Shape shape;
  uint32_t slot;

  OddShape(render::Shape shape = render::Shape::Count, uint32_t slot = 0)
      : shape(shape), slot(slot) {}

  int index(int count) const {
    if (!render::validShape(shape) || count < 2 || count > config::kMaxParticleCount) return -1;
    return static_cast<int>((static_cast<uint64_t>(slot) * static_cast<uint32_t>(count)) >> 32);
  }

  render::Shape shapeAt(int particle, int count, render::Shape majority) const {
    if (!render::validShape(majority)) majority = render::Shape::Circle;
    return particle >= 0 && particle == index(count) ? shape : majority;
  }
};

inline bool validVisualScale(float scale) {
  return isfinite(scale) && scale >= config::kMinVisualVariation &&
         scale <= config::kMaxVisualVariation;
}

inline bool validScale(float scale) {
  return isfinite(scale) && scale >= config::kMinSizeScale && scale <= config::kMaxSizeScale;
}

// Constant occupied area: diameter grows by s, therefore count falls by s^2.
// This capacity guard also bounds solver work and the initial patch's size.
inline int maxCountForScale(float scale) {
  if (!validScale(scale)) return 0;
  const int maximum = static_cast<int>(ceilf(config::kMaxParticleCount / (scale * scale)));
  return maximum < config::kMinParticleCount ? config::kMinParticleCount : maximum;
}

inline CountRange countRange(float scale) {
  if (!validScale(scale)) return {0, 0, 0};
  int baseline = static_cast<int>(config::kParticleCount / (scale * scale) + 0.5f);
  if (baseline < config::kMinParticleCount) baseline = config::kMinParticleCount;
  int low = static_cast<int>(floorf(baseline * (1.0f - config::kCountJitterFraction)));
  int high = static_cast<int>(ceilf(baseline * (1.0f + config::kCountJitterFraction)));
  if (low < config::kMinParticleCount) low = config::kMinParticleCount;
  const int maximum = maxCountForScale(scale);
  if (high > maximum) high = maximum;
  return {baseline, low, high};
}

inline float scaleFromUnit(float unit) {
  if (!isfinite(unit) || unit < 0) unit = 0;
  if (unit > 1) unit = 1;
  // Uniform diameter draw: every equal-width size interval has equal odds.
  return config::kMinVisualVariation + (config::kMaxVisualVariation - config::kMinVisualVariation) *
         unit;
}

inline float shapeAreaFactor(render::Shape shape) {
  switch (shape) {
    case render::Shape::Triangle: return 1.299038106f;
    case render::Shape::Square: return 2.0f;
    case render::Shape::Star: return 1.469463131f;
    default: return 3.141592654f;
  }
}

inline bool validPackedScale(float scale) {
  return isfinite(scale) && scale >= config::kMinVisualVariation && scale <= config::kMaxPackedScale;
}

inline int maxPackedCountForScale(float scale) {
  if (!validPackedScale(scale)) return 0;
  const int maximum = static_cast<int>(ceilf(config::kPackedCountBudget / (scale * scale)));
  return maximum < config::kMaxParticleCount ? maximum : config::kMaxParticleCount;
}

inline CountRange visualCountRange(float scale, render::Shape shape = render::Shape::Circle) {
  if (!validVisualScale(scale)) return {0, 0, 0};
  constexpr float screenArea = 3.141592654f * 120.0f * 120.0f;
  const float radius = config::kDotRadius * config::kVisualSizeScale * scale;
  const int high = maxPackedCountForScale(scale);
  // Explicit serial size/stress previews only. Randomize uses a coverage
  // target and a fitted population instead of sampling this crowded range.
  int low = static_cast<int>(ceilf(0.33f * screenArea / (shapeAreaFactor(shape) * radius * radius)));
  if (low < palette::kForegroundCount) low = palette::kForegroundCount;
  if (low > high) low = high;
  return {(low + high) / 2, low, high};
}

inline float unitFromBits(uint32_t bits) {
  // 24 uniform bits match float precision, including both size endpoints.
  return static_cast<float>(bits >> 8) * (1.0f / 16777215.0f);
}

template <class Random>
uint32_t bounded(Random &random, uint32_t count) {
  if (count < 2) return 0;
  const uint32_t threshold = (uint32_t(0) - count) % count;
  uint32_t bits = 0;
  for (int attempt = 0; attempt < 64; ++attempt) {
    bits = random();
    if (bits >= threshold) return bits % count;
  }
  // A broken/stuck random source must not block input and physics indefinitely.
  return bits % count;
}

inline float coverageFromUnit(float unit) {
  if (!isfinite(unit) || unit < 0) unit = 0;
  if (unit > 1) unit = 1;
  // Favor open layouts throughout one bounded band; no crowded exception.
  return config::kMinSpawnCoverage +
      (config::kMaxSpawnCoverage - config::kMinSpawnCoverage) * unit * unit;
}

template <class Random>
Population samplePopulation(Random &random) {
  const float scale = scaleFromUnit(unitFromBits(random()));
  const float unit = unitFromBits(random());
  return {scale, coverageFromUnit(unit)};
}

// Pick once per population; no random work is added to rendering or physics.
template <class Random>
float sampleStretch(Random &random) {
  return config::kDotStretchMin +
      (config::kDotStretchMax - config::kDotStretchMin) * unitFromBits(random());
}

// Change the whole population's visual shape once per button press. Avoid an
// immediate repeat, with no random work or shape state in the physics solver.
template <class Random>
render::Shape sampleShape(Random &random, render::Shape current) {
  const uint32_t count = static_cast<uint32_t>(render::Shape::Count);
  if (!render::validShape(current)) current = render::Shape::Circle;
  uint32_t choice = bounded(random, count - 1);
  if (choice >= static_cast<uint32_t>(current)) ++choice;
  return static_cast<render::Shape>(choice);
}

// Every pile contains exactly one other shape. Randomness is consumed only
// on boot/Randomize, never while animating or simulating particles.
template <class Random>
OddShape sampleOddShape(Random &random, render::Shape majority) {
  const render::Shape alternate = sampleShape(random, majority);
  return {alternate, random()};
}

template <class Random>
float sampleExplosionStrength(Random &random) {
  return config::kExplosionMinImpulse + unitFromBits(random()) *
         (config::kExplosionMaxImpulse - config::kExplosionMinImpulse);
}

// Every other locked swatch is equally likely; a press visibly changes it.
template <class Random>
uint8_t samplePalette(Random &random, uint8_t current) {
  if (current >= palette::kCount) current = 0;
  uint32_t choice = bounded(random, palette::kCount - 1);
  if (choice >= current) ++choice;
  return static_cast<uint8_t>(choice);
}

// Balance the five foreground colors, then shuffle once per new population.
// Color assignment is render-only and never consumes the solver's RNG.
template <class Random>
void assignColorSlots(uint8_t *slots, int count, Random &random) {
  if (!slots || count < 0 || count > config::kMaxParticleCount) return;
  for (int i = 0; i < count; ++i) slots[i] = static_cast<uint8_t>(i % palette::kForegroundCount);
  for (int i = count - 1; i > 0; --i) {
    const int j = static_cast<int>(bounded(random, i + 1));
    const uint8_t temporary = slots[i];
    slots[i] = slots[j];
    slots[j] = temporary;
  }
}
} // namespace variation
} // namespace sand
#endif
