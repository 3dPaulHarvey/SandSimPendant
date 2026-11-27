#ifndef GRAVITY_SAND_FLUID_COVERAGE_H
#define GRAVITY_SAND_FLUID_COVERAGE_H

#include <math.h>
#include <stdint.h>
#include <string.h>
#include "SandConfig.h"
#include "FluidShapes.h"

// Coverage fitting is bounded, one-shot work. Speed up its scanline/bitmap
// loops on the ESP32 without weakening finite checks or changing geometry.
#if defined(ARDUINO_ARCH_ESP32) && defined(__GNUC__) && !defined(__clang__)
#pragma GCC push_options
#pragma GCC optimize("O3")
#endif

namespace sand {
namespace render {

// One bit per pixel measures the clipped UNION of stationary spawn outlines.
// This belongs to the input/physics core and is used only on population changes;
// the LCD core keeps exclusive ownership of its independent rendering buffers.
class SpawnCoverage {
 public:
  static constexpr int kSize = 240;
  static constexpr float kScreenArea = 3.141592654f * (kSize / 2) * (kSize / 2);
  static_assert(kSize % 8 == 0, "Coverage rows must align to bitmap bytes");

  SpawnCoverage() {
    constexpr float center = (kSize - 1) * 0.5f;
    constexpr float radius = config::kCircleRadius + 0.5f;
    for (int y = 0; y < kSize; ++y) {
      const float dy = y - center;
      const float square = radius * radius - dy * dy;
      if (square < 0) { left_[y] = 1; right_[y] = 0; continue; }
      const float extent = sqrtf(square);
      left_[y] = static_cast<int16_t>(ceilf(center - extent));
      right_[y] = static_cast<int16_t>(floorf(center + extent));
    }
  }

  template <class ParticleAt>
  float measure(int count, float radius, Shape shape, ParticleAt particleAt,
                int oddIndex = -1, Shape oddShape = Shape::Count) {
    memset(mask_, 0, sizeof(mask_));
    pixels_ = 0;
    if (count < 1 || count > config::kMaxParticleCount || !isfinite(radius) ||
        radius <= 0 || radius > config::kDotRadius * config::kVisualSizeScale * config::kMaxPackedScale)
      return 0;
    const Outline main = outline(shape, radius);
    if (oddIndex < 0 || oddIndex >= count || !validShape(oddShape) || oddShape == shape) oddIndex = -1;
    const Outline odd = oddIndex >= 0 ? outline(oddShape, radius) : Outline{};
    for (int dot = 0; dot < count; ++dot) {
      const Outline &polygon = dot == oddIndex ? odd : main;
      const auto &particle = particleAt(dot);
      if (!isfinite(particle.x) || !isfinite(particle.y)) continue;
      if (particle.x < -radius || particle.x > kSize - 1 + radius ||
          particle.y < -radius || particle.y > kSize - 1 + radius) continue;
      int top = static_cast<int>(ceilf(particle.y - radius));
      int bottom = static_cast<int>(floorf(particle.y + radius));
      if (top < 0) top = 0;
      if (bottom >= kSize) bottom = kSize - 1;
      for (int y = top; y <= bottom; ++y) {
        const float localY = y - particle.y;
        if (!polygon.count) {
          const float square = radius * radius - localY * localY;
          if (square < 0) continue;
          const float extent = sqrtf(square);
          span(y, static_cast<int>(ceilf(particle.x - extent)),
               static_cast<int>(floorf(particle.x + extent)));
          continue;
        }
        float crossings[10];
        int found = 0;
        for (int edge = 0; edge < polygon.count; ++edge) {
          const ShapeVertex &a = polygon.vertices[edge], &b = polygon.vertices[(edge + 1) % polygon.count];
          if (!((a.y <= localY && localY < b.y) || (b.y <= localY && localY < a.y))) continue;
          const float x = particle.x + a.x + (localY - a.y) * polygon.slopes[edge];
          int insertion = found;
          while (insertion > 0 && crossings[insertion - 1] > x) {
            crossings[insertion] = crossings[insertion - 1];
            --insertion;
          }
          crossings[insertion] = x;
          ++found;
        }
        for (int edge = 0; edge + 1 < found; edge += 2)
          span(y, static_cast<int>(ceilf(crossings[edge])),
               static_cast<int>(floorf(crossings[edge + 1])));
      }
    }
    return pixels_ / kScreenArea;
  }

 private:
  struct Outline {
    ShapeVertex vertices[10];
    float slopes[10];
    int count;
  };

  static Outline outline(Shape shape, float radius) {
    const ShapePolygon polygon = shapePolygon(shape);
    Outline result = {};
    result.count = polygon.count;
    for (int i = 0; i < polygon.count; ++i) {
      const ShapeVertex &a = polygon.vertices[i], &b = polygon.vertices[(i + 1) % polygon.count];
      result.vertices[i] = {a.x * radius, a.y * radius};
      if (b.y != a.y) result.slopes[i] = (b.x - a.x) / (b.y - a.y);
    }
    return result;
  }

  uint8_t mask_[kSize * kSize / 8] = {};
  int16_t left_[kSize] = {}, right_[kSize] = {};
  int pixels_ = 0;

  static int countBits(uint8_t bits) {
    unsigned value = bits;
    value -= (value >> 1) & 0x55u;
    value = (value & 0x33u) + ((value >> 2) & 0x33u);
    return static_cast<int>((value + (value >> 4)) & 0x0fu);
  }

  void markByte(int index, uint8_t bits) {
    uint8_t &value = mask_[index];
    pixels_ += countBits(static_cast<uint8_t>(bits & ~value));
    value |= bits;
  }

  void span(int y, int left, int right) {
    if (left < left_[y]) left = left_[y];
    if (right > right_[y]) right = right_[y];
    if (left > right) return;
    const int first = y * (kSize / 8) + (left >> 3);
    const int last = y * (kSize / 8) + (right >> 3);
    const uint8_t firstBits = static_cast<uint8_t>(0xffu << (left & 7));
    const uint8_t lastBits = static_cast<uint8_t>(0xffu >> (7 - (right & 7)));
    if (first == last) {
      markByte(first, static_cast<uint8_t>(firstBits & lastBits));
      return;
    }
    markByte(first, firstBits);
    // Whole interior bytes cover eight pixels at a time. Empty bytes are the
    // common case in ordinary rolls; already covered pixels never count twice.
    for (int index = first + 1; index < last; ++index) {
      uint8_t &value = mask_[index];
      if (value == 0xffu) continue;
      pixels_ += value ? 8 - countBits(value) : 8;
      value = 0xffu;
    }
    markByte(last, lastBits);
  }
};
} // namespace render
} // namespace sand

#if defined(ARDUINO_ARCH_ESP32) && defined(__GNUC__) && !defined(__clang__)
#pragma GCC pop_options
#endif

#endif
