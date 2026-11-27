#ifndef GRAVITY_SAND_FLUID_BACKGROUND_H
#define GRAVITY_SAND_FLUID_BACKGROUND_H

#include <math.h>
#include <stdint.h>
#include <string.h>
#include "FluidPalettes.h"

// Bounded grid shading benefits from loop optimization on the pendant. Keep
// normal floating-point behavior; the renderer never enables fast-math.
#if defined(ARDUINO_ARCH_ESP32) && defined(__GNUC__) && !defined(__clang__)
#pragma GCC push_options
#pragma GCC optimize("O3")
#endif

namespace sand {
namespace render {

// Three overlapping, softly shaded shapes. The 4px color grid uses 14.4KB for
// current/target colors, rather than a full RGB framebuffer. Only changed,
// visible cells consume the per-frame transfer budget; hidden colors can catch
// up immediately so moving particles uncover the correct backdrop.
class FluidBackground {
 public:
  static constexpr int kSize = 240;
  static constexpr int kCellSize = 4;
  static constexpr int kCells = kSize / kCellSize;
  static constexpr int kCellCount = kCells * kCells;
  static constexpr int kRasterColors = 33;
  static constexpr uint32_t kTickMs = 100;

  template <class IsVisible, class Invalidate>
  void prepare(uint8_t paletteIndex, uint32_t effectSeed, uint32_t nowMs,
               int visibleTileBudget, IsVisible visible, Invalidate invalidate) {
    if (paletteIndex >= palette::kCount) paletteIndex = 0;
    if (paletteIndex != palette_ || effectSeed != effectSeed_) {
      palette_ = paletteIndex;
      effectSeed_ = effectSeed;
      initialize();
      lastTickMs_ = nowMs;
      shadeTarget();
      memcpy(current_, target_, sizeof(current_));
      cursor_ = 0;
      invalidate(0, 0, kSize, kSize);
      return;
    }
    // A deliberately paused effect neither advances nor accumulates a backlog.
    if (visibleTileBudget <= 0) { lastTickMs_ = nowMs; return; }
    if (visibleTileBudget > kCellCount) visibleTileBudget = kCellCount;
    const uint32_t elapsed = nowMs - lastTickMs_;
    if (elapsed >= kTickMs) {
      lastTickMs_ = nowMs;
      // Time subtraction is wrap-safe; a late frame cannot teleport a shape.
      const float seconds = (elapsed > 200 ? 200 : elapsed) * 0.001f;
      advance(seconds);
      shadeTarget();
    }

    int nextCursor = cursor_;
    int remaining = visibleTileBudget;
    for (int scanned = 0; scanned < kCellCount; ++scanned) {
      int cell = cursor_ + scanned;
      if (cell >= kCellCount) cell -= kCellCount;
      if (current_[cell] == target_[cell]) continue;
      const int x = (cell % kCells) * kCellSize;
      const int y = (cell / kCells) * kCellSize;
      if (!visible(x, y)) {
        current_[cell] = target_[cell];
      } else if (remaining > 0) {
        current_[cell] = target_[cell];
        invalidate(x, y, kCellSize, kCellSize);
        --remaining;
        nextCursor = cell + 1;
        if (nextCursor == kCellCount) nextCursor = 0;
      }
    }
    // Resume after the last visible update, even when a newer target field
    // arrives before the previous one has drained. No corner can starve.
    cursor_ = nextCursor;
  }

  inline __attribute__((always_inline)) uint16_t colorAt(int x, int y) const {
    if (static_cast<unsigned>(x) >= kSize || static_cast<unsigned>(y) >= kSize) return 0;
    return current_[(y / kCellSize) * kCells + x / kCellSize];
  }

  inline __attribute__((always_inline))
  uint16_t composite(uint8_t encoded, int x, int y) const {
    if (!encoded || encoded >= kRasterColors) return colorAt(x, y);
    const unsigned coverage = ((encoded - 1) & 3) + 1;
    const uint16_t foreground = foreground_[(encoded - 1) >> 2];
    if (coverage == 4) return foreground;
    return blend(foreground, colorAt(x, y), coverage, 4);
  }

  uint16_t rimColor() const { return palette::get(palette_).background[2]; }
  uint32_t effectSeed() const { return effectSeed_; }
  uint32_t revision() const { return revision_; }

  static inline __attribute__((always_inline))
  uint16_t blend(uint16_t foreground, uint16_t background,
                 unsigned amount, unsigned denominator) {
    if (!denominator || amount >= denominator) return foreground;
    const unsigned rest = denominator - amount;
    const unsigned red = (((foreground >> 11) & 31) * amount +
                          ((background >> 11) & 31) * rest) / denominator;
    const unsigned green = (((foreground >> 5) & 63) * amount +
                            ((background >> 5) & 63) * rest) / denominator;
    const unsigned blue = ((foreground & 31) * amount +
                           (background & 31) * rest) / denominator;
    return static_cast<uint16_t>((red << 11) | (green << 5) | blue);
  }

 private:
  struct Color { float r, g, b; };
  struct Layer {
    float x, y, heading, speed;
    float angle, rotationRate, inverseX, inverseY;
    float turnPhase, turnRate, turnFrequency;
    float brightnessPhase, brightnessFrequency;
    float saturationPhase, saturationFrequency;
    float hue, saturation, value, opacity;
    float cosine, sine;
    Color color;
    uint8_t shape;
  };
  uint16_t current_[kCellCount] = {}, target_[kCellCount] = {};
  uint16_t foreground_[8] = {};
  Layer layers_[3] = {};
  Color baseTop_ = {}, baseBottom_ = {};
  uint8_t palette_ = 0xFF;
  uint32_t effectSeed_ = 0, rng_ = 1, lastTickMs_ = 0, revision_ = 0;
  int cursor_ = 0;

  static float minimum(float a, float b) { return a < b ? a : b; }
  static float maximum(float a, float b) { return a > b ? a : b; }
  static float absolute(float value) { return value < 0 ? -value : value; }
  static float wrap(float angle) {
    constexpr float tau = 6.283185307f;
    if (angle >= tau) angle -= tau;
    if (angle < 0) angle += tau;
    return angle;
  }

  uint32_t randomBits() {
    rng_ ^= rng_ << 13;
    rng_ ^= rng_ >> 17;
    rng_ ^= rng_ << 5;
    return rng_;
  }
  float random(float low, float high) {
    return low + (high - low) * static_cast<float>(randomBits() >> 8) * (1.0f / 16777216.0f);
  }

  static Color unpack(uint16_t color) {
    const unsigned r = (color >> 11) & 31, g = (color >> 5) & 63, b = color & 31;
    return {static_cast<float>((r << 3) | (r >> 2)),
            static_cast<float>((g << 2) | (g >> 4)),
            static_cast<float>((b << 3) | (b >> 2))};
  }

  static uint16_t pack(const Color &color) {
    const unsigned r = static_cast<unsigned>(color.r + 0.5f);
    const unsigned g = static_cast<unsigned>(color.g + 0.5f);
    const unsigned b = static_cast<unsigned>(color.b + 0.5f);
    return static_cast<uint16_t>(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
  }

  static void setHue(Layer &layer, uint16_t color) {
    const Color rgb = unpack(color);
    const float high = maximum(rgb.r, maximum(rgb.g, rgb.b));
    const float low = minimum(rgb.r, minimum(rgb.g, rgb.b));
    const float delta = high - low;
    layer.value = high * (1.0f / 255.0f);
    layer.saturation = high > 0 ? delta / high : 0;
    float hue = 0;
    if (delta > 0) {
      if (high == rgb.r) hue = (rgb.g - rgb.b) / delta;
      else if (high == rgb.g) hue = 2 + (rgb.b - rgb.r) / delta;
      else hue = 4 + (rgb.r - rgb.g) / delta;
      if (hue < 0) hue += 6;
    }
    layer.hue = hue;
  }

  static Color modulatedColor(const Layer &layer) {
    const float brightness = 1.075f + 0.375f * sinf(layer.brightnessPhase);
    const float saturation = minimum(1.0f, layer.saturation *
                                    (1.0f + 0.30f * sinf(layer.saturationPhase)));
    const float value = minimum(110.0f, layer.value * brightness * 255.0f);
    const int sector = static_cast<int>(layer.hue);
    const float fraction = layer.hue - sector;
    const float p = value * (1 - saturation);
    const float q = value * (1 - saturation * fraction);
    const float t = value * (1 - saturation * (1 - fraction));
    switch (sector) {
      case 0: return {value, t, p};
      case 1: return {q, value, p};
      case 2: return {p, value, t};
      case 3: return {p, q, value};
      case 4: return {t, p, value};
      default: return {value, p, q};
    }
  }

  void initialize() {
    constexpr float tau = 6.283185307f;
    rng_ = effectSeed_ ^ (0x9E3779B9u * (palette_ + 1u));
    if (!rng_) rng_ = 0xA341316Cu;
    const palette::Swatch &swatch = palette::get(palette_);
    for (int i = 0; i < 8; ++i) foreground_[i] = swatch.foreground[i % palette::kForegroundCount];
    const Color a = unpack(swatch.background[0]);
    const Color b = unpack(swatch.background[1]);
    const Color c = unpack(swatch.background[2]);
    baseTop_ = {a.r * 0.62f + c.r * 0.08f, a.g * 0.62f + c.g * 0.08f, a.b * 0.62f + c.b * 0.08f};
    baseBottom_ = {a.r * 0.50f + b.r * 0.18f, a.g * 0.50f + b.g * 0.18f, a.b * 0.50f + b.b * 0.18f};
    const int colorOffset = static_cast<int>(randomBits() % 3);
    const int shapeOffset = static_cast<int>(randomBits() % 3);
    const int direction = (randomBits() & 1u) ? 1 : 2;
    for (int i = 0; i < 3; ++i) {
      Layer &layer = layers_[i];
      layer.x = random(35, 205);
      layer.y = random(35, 205);
      layer.heading = random(0, tau);
      layer.speed = random(8, 18);
      layer.angle = random(0, tau);
      layer.rotationRate = random(-0.14f, 0.14f);
      layer.inverseX = 1.0f / random(62, 108);
      layer.inverseY = 1.0f / random(54, 100);
      layer.turnPhase = random(0, tau);
      layer.turnRate = random(-0.38f, 0.38f);
      layer.turnFrequency = random(0.25f, 0.65f);
      layer.brightnessPhase = random(0, tau);
      layer.brightnessFrequency = tau / random(6, 14);
      layer.saturationPhase = random(0, tau);
      layer.saturationFrequency = tau / random(7, 15);
      layer.opacity = random(0.65f, 0.88f);
      layer.shape = static_cast<uint8_t>((shapeOffset + direction * i) % 3);
      setHue(layer, swatch.background[(colorOffset + i) % 3]);
      layer.cosine = cosf(layer.angle);
      layer.sine = sinf(layer.angle);
      layer.color = modulatedColor(layer);
    }
  }

  void advance(float seconds) {
    constexpr float pi = 3.141592654f;
    for (Layer &layer : layers_) {
      layer.turnPhase = wrap(layer.turnPhase + layer.turnFrequency * seconds);
      layer.heading = wrap(layer.heading + layer.turnRate * sinf(layer.turnPhase) * seconds);
      layer.x += cosf(layer.heading) * layer.speed * seconds;
      layer.y += sinf(layer.heading) * layer.speed * seconds;
      if (layer.x < 24) { layer.x = 48 - layer.x; layer.heading = wrap(pi - layer.heading); }
      if (layer.x > 216) { layer.x = 432 - layer.x; layer.heading = wrap(pi - layer.heading); }
      if (layer.y < 24) { layer.y = 48 - layer.y; layer.heading = wrap(-layer.heading); }
      if (layer.y > 216) { layer.y = 432 - layer.y; layer.heading = wrap(-layer.heading); }
      layer.angle = wrap(layer.angle + layer.rotationRate * seconds);
      layer.cosine = cosf(layer.angle);
      layer.sine = sinf(layer.angle);
      layer.brightnessPhase = wrap(layer.brightnessPhase + layer.brightnessFrequency * seconds);
      layer.saturationPhase = wrap(layer.saturationPhase + layer.saturationFrequency * seconds);
      layer.color = modulatedColor(layer);
    }
  }

  void shadeTarget() {
    for (int y = 0; y < kCells; ++y) {
      const float pixelY = y * kCellSize + (kCellSize - 1) * 0.5f;
      const float vertical = pixelY * (1.0f / (kSize - 1));
      const Color base = {baseTop_.r + (baseBottom_.r - baseTop_.r) * vertical,
                          baseTop_.g + (baseBottom_.g - baseTop_.g) * vertical,
                          baseTop_.b + (baseBottom_.b - baseTop_.b) * vertical};
      for (int x = 0; x < kCells; ++x) {
        const float pixelX = x * kCellSize + (kCellSize - 1) * 0.5f;
        Color color = base;
        for (const Layer &layer : layers_) {
          const float dx = pixelX - layer.x, dy = pixelY - layer.y;
          const float localX = (dx * layer.cosine + dy * layer.sine) * layer.inverseX;
          const float localY = (dy * layer.cosine - dx * layer.sine) * layer.inverseY;
          const float xx = localX * localX, yy = localY * localY;
          float distance;
          if (layer.shape == 0) distance = xx + yy; // Ellipse.
          else if (layer.shape == 1) distance = xx * xx + yy * yy; // Rounded rectangle.
          else distance = absolute(localX) + absolute(localY); // Diamond.
          if (distance >= 1) continue;
          const float falloff = 1 - distance;
          const float alpha = layer.opacity * falloff * falloff * (3 - 2 * falloff);
          color.r += (layer.color.r - color.r) * alpha;
          color.g += (layer.color.g - color.g) * alpha;
          color.b += (layer.color.b - color.b) * alpha;
        }
        target_[y * kCells + x] = pack(color);
      }
    }
    ++revision_;
  }
};
} // namespace render
} // namespace sand

#if defined(ARDUINO_ARCH_ESP32) && defined(__GNUC__) && !defined(__clang__)
#pragma GCC pop_options
#endif

#endif
