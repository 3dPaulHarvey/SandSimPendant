#ifndef GRAVITY_SAND_FLUID_RENDERER_H
#define GRAVITY_SAND_FLUID_RENDERER_H

#include <math.h>
#include <stdint.h>
#include <string.h>
#include "DirtyRenderer.h"
#include "SandConfig.h"
#include "FluidShapes.h"

// Large overlapping polygons spend their time in bounded coverage/copy loops.
// Optimize those loops on the ESP32 while retaining finite checks (no fast-math).
#if defined(ARDUINO_ARCH_ESP32) && defined(__GNUC__) && !defined(__clang__)
#pragma GCC push_options
#pragma GCC optimize("O3")
#endif

namespace sand {
namespace render {

// An immutable render snapshot contains just the visible particle state.
// Coordinates remain fractional pixels all the way to the dot rasterizer.
struct Dot {
  float x, y;
  uint8_t color;
  Shape shape; // Count means use the population's shape; one dot may override it.
  float vx, vy;
  float impactX, impactY, impactScale, impactWeight;

  Dot(float x = 0, float y = 0, uint8_t color = 0, float vx = 0, float vy = 0)
      : x(x), y(y), color(color), shape(Shape::Count), vx(vx), vy(vy),
        impactX(0), impactY(0), impactScale(1), impactWeight(0) {}
};

struct PixelRect {
  uint8_t x, y, w, h;
};

// Two byte-per-pixel rasters cost 115.2 KB, rather than 230.4 KB for RGB565.
// 0 is transparent background; 1..32 encode eight colors at four coverages.
// Dirty tiles only batch transfers: they never quantize particle positions.
class FluidRaster {
 public:
  static constexpr int kSize = 240;
  static constexpr int kTileSize = 4;
  static constexpr int kTiles = kSize / kTileSize;
  static constexpr int kPixels = kSize * kSize;
  static constexpr float kMaxDotRadius =
      config::kDotRadius * config::kMaxPackedScale * config::kVisualSizeScale;

  // Cache the pendant's circular viewport once, not once per particle/frame.
  // Generic raster callers retain rectangular clipping unless they opt in.
  void setClipCircle(float radius) {
    cachedSnapshot_ = false;
    clipCircle_ = isfinite(radius) && radius > 0 && radius <= kSize;
    if (!clipCircle_) return;
    const float center = (kSize - 1) * 0.5f;
    for (int y = 0; y < kSize; ++y) {
      const float dy = y - center;
      const float width2 = radius * radius - dy * dy;
      if (width2 < 0) { clipLeft_[y] = 1; clipRight_[y] = 0; continue; }
      const float width = sqrtf(width2);
      clipLeft_[y] = static_cast<uint8_t>(maximum(0, static_cast<int>(ceilf(center - width))));
      clipRight_[y] = static_cast<uint8_t>(minimum(kSize - 1, static_cast<int>(floorf(center + width))));
    }
  }

  void draw(const Dot *dots, int count, float radius, Shape shape = Shape::Circle,
            float stretchMax = config::kDotStretchMax) {
    // The population supplies one stable deformation limit. Sanitize once per
    // draw; wall impacts supply a brief visual-only override per particle.
    if (!isfinite(stretchMax) || stretchMax < 1) stretchMax = 1;
    if (stretchMax > config::kDotStretchMax) stretchMax = config::kDotStretchMax;
    const bool cacheable = dots && count >= 0 && count <= config::kMaxParticleCount &&
                           radius > 0 && isfinite(radius) && radius <= kMaxDotRadius;
    if (cacheable && sameSnapshot(dots, count, radius, shape, stretchMax)) return;
    cachedSnapshot_ = cacheable;
    if (cacheable) {
      cachedCount_ = count;
      cachedRadius_ = radius;
      cachedShape_ = shape;
      cachedStretchMax_ = stretchMax;
      for (int i = 0; i < count; ++i) cachedDots_[i] = dots[i];
    }
    // Keep the committed picture untouched. Swap roles at the next draw,
    // avoiding a complete 57.6KB copy after every SPI update.
    drawing_ = 1 - committed_;
    uint8_t *current = images_[drawing_];
    memset(current, 0, kPixels);
    memset(occupied_[drawing_], 0, sizeof(occupied_[drawing_]));
    if (!dots || count < 0 || count > config::kMaxParticleCount ||
        !(radius > 0.0f) || !isfinite(radius) || radius > kMaxDotRadius) return;
    if (!validShape(shape)) shape = Shape::Circle;
    const ShapePolygon populationPolygon = shapePolygon(shape);
    const float outer = radius + 0.5f;
    const float inner = radius > 0.5f ? radius - 0.5f : 0;
    const float outer2 = outer * outer, inner2 = inner * inner;
    const float edgeScale = 4.0f / (outer2 - inner2);
    constexpr float inverseStretchSpeedRange =
        1.0f / (config::kDotStretchFullSpeed - config::kDotStretchStartSpeed);
    for (int i = 0; i < count; ++i) {
      const Dot &dot = dots[i];
      const Shape dotShape = validShape(dot.shape) ? dot.shape : shape;
      if (dotShape != Shape::Circle) {
        const ShapePolygon polygon = dotShape == shape ? populationPolygon : shapePolygon(dotShape);
        drawPolygon(current, dot, radius, polygon, stretchMax);
        continue;
      }
      const float speed2 = dot.vx * dot.vx + dot.vy * dot.vy;
      if (dot.impactWeight > 0) {
        float xx, xy, yy;
        velocityTransform(dot, speed2, stretchMax, xx, xy, yy);
        if (blendImpact(dot, xx, xy, yy)) {
          // Inverse quadratic of the area-preserving symmetric transform.
          const float a = yy * yy + xy * xy;
          const float b = -xy * (xx + yy);
          const float c = xx * xx + xy * xy;
          drawDot<true>(current, dot, outer * sqrtf(c), outer * sqrtf(a),
                        inner2, outer2, edgeScale, a, b, c);
          continue;
        }
      }
      if (stretchMax > 1 && isfinite(speed2) &&
          speed2 > config::kDotStretchStartSpeed * config::kDotStretchStartSpeed) {
        float amount = (sqrtf(speed2) - config::kDotStretchStartSpeed) * inverseStretchSpeedRange;
        if (amount > 1) amount = 1;
        const float stretch = 1 + (stretchMax - 1) * amount;
        const float across = stretch * stretch;
        const float along = 1.0f / across;
        const float directional = (along - across) / speed2;
        // Ellipse quadratic: reciprocal axes keep area constant, and velocity
        // supplies orientation without atan2/sin/cos or a neighbor search.
        const float a = across + directional * dot.vx * dot.vx;
        const float b = directional * dot.vx * dot.vy;
        const float c = across + directional * dot.vy * dot.vy;
        // The quadratic has determinant one, so these are its tight extents.
        drawDot<true>(current, dot, outer * sqrtf(c), outer * sqrtf(a),
                      inner2, outer2, edgeScale, a, b, c);
      } else {
        drawDot<false>(current, dot, outer, outer, inner2, outer2, edgeScale, 1, 0, 1);
      }
    }
  }

  void invalidate(int x, int y, int width, int height) {
    const int left = maximum(0, x), top = maximum(0, y);
    const int right = minimum(kSize - 1, x + width - 1);
    const int bottom = minimum(kSize - 1, y + height - 1);
    if (left <= right && top <= bottom) mark(forced_, left, top, right, bottom);
  }

  // Changing RGB palette only needs repainting occupied old/new pixels.
  void invalidateColors() {
    for (int y = 0; y < kTiles; ++y)
      for (int word = 0; word < 2; ++word)
        forced_[y][word] |= occupied_[drawing_][y][word] | occupied_[committed_][y][word];
  }

  // Tile batching is followed by exact dirty-pixel bounds. Trimming the outer
  // rows/columns of each transfer saves SPI traffic without changing the image.
  template <class Emit>
  void planPixels(const Options &options, Emit emit) {
    memset(changed_, 0, sizeof(changed_));
    const uint8_t *current = images_[drawing_], *previous = images_[committed_];
    for (int tileY = 0; tileY < kTiles; ++tileY) {
      for (int word = 0; word < 2; ++word) {
        uint32_t candidates = forced_[tileY][word];
        // An unchanged snapshot reuses the committed image itself. Palette,
        // animated background and overlay invalidations still repaint normally.
        if (drawing_ != committed_)
          candidates |= occupied_[drawing_][tileY][word] | occupied_[committed_][tileY][word];
        while (candidates) {
          const int bit = __builtin_ctz(candidates);
          candidates &= candidates - 1;
          const int tileX = word * 32 + bit;
          const int tile = tileY * kTiles + tileX;
          if (forced_[tileY][word] & (uint32_t(1) << bit)) {
            changed_[tile] = 1;
            bounds_[tile] = 0xCC; // x=0..3, y=0..3.
            continue;
          }
          const int offset = tileY * kTileSize * kSize + tileX * kTileSize;
          // Most settled tiles compare with four aligned 32-bit loads.
          bool equal = true;
          for (int y = 0; y < kTileSize; ++y)
            if (memcmp(current + offset + y * kSize,
                       previous + offset + y * kSize, kTileSize) != 0) {
              equal = false;
              break;
            }
          if (equal) continue;
          int left = 3, right = 0, top = 3, bottom = 0;
          for (int y = 0; y < kTileSize; ++y)
            for (int x = 0; x < kTileSize; ++x)
              if (current[offset + y * kSize + x] != previous[offset + y * kSize + x]) {
                if (x < left) left = x;
                if (x > right) right = x;
                if (y < top) top = y;
                if (y > bottom) bottom = y;
              }
          changed_[tile] = 1;
          bounds_[tile] = static_cast<uint8_t>(left | (right << 2) | (top << 4) | (bottom << 6));
        }
      }
    }
    render::plan(changed_, unchanged_, kTiles, kTiles, options, [&](const Rect &tiles) {
      int left = kSize, right = 0, top = kSize, bottom = 0;
      for (int y = tiles.y; y < tiles.y + tiles.h; ++y)
        for (int x = tiles.x; x < tiles.x + tiles.w; ++x) {
          const int tile = y * kTiles + x;
          if (!changed_[tile]) continue;
          const uint8_t bounds = bounds_[tile];
          left = minimum(left, x * kTileSize + (bounds & 3));
          right = maximum(right, x * kTileSize + ((bounds >> 2) & 3));
          top = minimum(top, y * kTileSize + ((bounds >> 4) & 3));
          bottom = maximum(bottom, y * kTileSize + (bounds >> 6));
        }
      const PixelRect pixels = {static_cast<uint8_t>(left), static_cast<uint8_t>(top),
        static_cast<uint8_t>(right - left + 1), static_cast<uint8_t>(bottom - top + 1)};
      emit(pixels);
    });
  }

  void commit() {
    committed_ = drawing_;
    memset(forced_, 0, sizeof(forced_));
  }
  uint8_t pixel(int x, int y) const { return images_[drawing_][y * kSize + x]; }
  const uint8_t *row(int y) const { return images_[drawing_] + y * kSize; }

  static uint16_t shade(uint16_t rgb, int coverage) {
    const unsigned red = ((rgb >> 11) & 31) * coverage / 4;
    const unsigned green = ((rgb >> 5) & 63) * coverage / 4;
    const unsigned blue = (rgb & 31) * coverage / 4;
    return static_cast<uint16_t>((red << 11) | (green << 5) | blue);
  }

 private:
  Dot cachedDots_[config::kMaxParticleCount];
  int cachedCount_ = 0;
  float cachedRadius_ = 0;
  float cachedStretchMax_ = 0;
  Shape cachedShape_ = Shape::Circle;
  bool cachedSnapshot_ = false;
  bool clipCircle_ = false;
  uint8_t clipLeft_[kSize] = {}, clipRight_[kSize] = {};
  alignas(4) uint8_t images_[2][kPixels] = {};
  uint32_t occupied_[2][kTiles][2] = {}, forced_[kTiles][2] = {};
  uint8_t drawing_ = 0, committed_ = 1;
  uint8_t changed_[kTiles * kTiles] = {}, unchanged_[kTiles * kTiles] = {};
  uint8_t bounds_[kTiles * kTiles] = {};

  bool sameSnapshot(const Dot *dots, int count, float radius, Shape shape, float stretchMax) const {
    if (!cachedSnapshot_ || count != cachedCount_ || radius != cachedRadius_ ||
        shape != cachedShape_ || stretchMax != cachedStretchMax_)
      return false;
    // Compare fields, not struct padding. No rounding or motion threshold may
    // suppress visible subpixel changes, and NaNs naturally force a redraw.
    for (int i = 0; i < count; ++i) {
      const Dot &a = dots[i], &b = cachedDots_[i];
      if (a.x != b.x || a.y != b.y || a.color != b.color || a.shape != b.shape ||
          a.vx != b.vx || a.vy != b.vy || a.impactX != b.impactX ||
          a.impactY != b.impactY || a.impactScale != b.impactScale ||
          a.impactWeight != b.impactWeight)
        return false;
    }
    return true;
  }

  static constexpr int kUnclippedShapeRowPixels =
      static_cast<int>(2 * kMaxDotRadius * config::kDotStretchMax) + 5;
  // Spans are clipped before indexing this row. Even a 300px stretched shape
  // therefore needs at most one visible screen row of scratch storage.
  static constexpr int kShapeRowPixels =
      kUnclippedShapeRowPixels < kSize ? kUnclippedShapeRowPixels : kSize;
  struct ShapeEdge {
    int first, last; // Half-pixel scanlines; the upper endpoint is excluded.
    float x, step;
  };
  struct ShapeCoverage {
    uint16_t pixels[kShapeRowPixels];
    int first, last; // Only this interval is initialized for the current row.
  };

  // Polygon coordinates were bounded before these conversions. Unlike the
  // general libm functions, these need not handle huge values or infinities.
  static int shapeFloor(float value) {
    const int integer = static_cast<int>(value);
    return integer - (static_cast<float>(integer) > value);
  }
  static int shapeCeil(float value) {
    const int integer = static_cast<int>(value);
    return integer + (static_cast<float>(integer) < value);
  }

  // Each sample represents half a pixel row. Whole interior pixels need only
  // an integer addition; fractions are calculated at the two span ends.
  static inline __attribute__((always_inline))
  void addShapeSpan(ShapeCoverage &row, int origin, int left, int right,
                    float firstX, float lastX) {
    if (left > right) return;
    const float clipLeft = left - 0.5f, clipRight = right + 0.5f;
    if (firstX < clipLeft) firstX = clipLeft;
    if (lastX > clipRight) lastX = clipRight;
    if (!(firstX < lastX)) return;
    // Clipping makes both values nonnegative before integer conversion.
    const int first = maximum(left, static_cast<int>(firstX + 0.5f));
    const int last = minimum(right, static_cast<int>(lastX + 0.5f));
    uint16_t *coverage = row.pixels;
    if (row.first > row.last) {
      // The first span writes its own interval directly. A long diagonal
      // outline no longer clears or scans its entire bounding box per row.
      row.first = first;
      row.last = last;
      if (first == last) {
        coverage[first - origin] = static_cast<uint16_t>((lastX - firstX) * 128 + 0.5f);
        return;
      }
      coverage[first - origin] = static_cast<uint16_t>((first + 0.5f - firstX) * 128 + 0.5f);
      uint16_t *interior = coverage + (first - origin + 1);
      const uint16_t *end = coverage + (last - origin);
      for (; interior < end; ++interior) *interior = 128;
      coverage[last - origin] = static_cast<uint16_t>((lastX - last + 0.5f) * 128 + 0.5f);
      return;
    }
    // Later half-row samples may extend either side or create a disjoint
    // star span. Initialize the whole extension, including any intervening gap.
    if (first < row.first) {
      for (int x = first; x < row.first; ++x) coverage[x - origin] = 0;
      row.first = first;
    }
    if (last > row.last) {
      for (int x = row.last + 1; x <= last; ++x) coverage[x - origin] = 0;
      row.last = last;
    }
    if (first == last) {
      coverage[first - origin] += static_cast<uint16_t>((lastX - firstX) * 128 + 0.5f);
      return;
    }
    coverage[first - origin] += static_cast<uint16_t>((first + 0.5f - firstX) * 128 + 0.5f);
    uint16_t *interior = coverage + (first - origin + 1);
    const uint16_t *end = coverage + (last - origin);
    for (; interior < end; ++interior) *interior += 128;
    coverage[last - origin] += static_cast<uint16_t>((lastX - last + 0.5f) * 128 + 0.5f);
  }

  static void velocityTransform(const Dot &dot, float speed2, float stretchMax,
                                float &xx, float &xy, float &yy) {
    xx = yy = 1;
    xy = 0;
    if (stretchMax > 1 && isfinite(speed2) &&
        speed2 > config::kDotStretchStartSpeed * config::kDotStretchStartSpeed) {
      constexpr float inverseRange =
          1.0f / (config::kDotStretchFullSpeed - config::kDotStretchStartSpeed);
      float amount = (sqrtf(speed2) - config::kDotStretchStartSpeed) * inverseRange;
      if (amount > 1) amount = 1;
      const float stretch = 1 + (stretchMax - 1) * amount;
      const float squeeze = 1.0f / stretch;
      const float directional = (stretch - squeeze) / speed2;
      // Symmetric stretch is continuous to identity at low speed. Fixed
      // upright shapes deform along travel without a sudden orientation snap.
      xx = squeeze + directional * dot.vx * dot.vx;
      xy = directional * dot.vx * dot.vy;
      yy = squeeze + directional * dot.vy * dot.vy;
    }
  }

  static bool blendImpact(const Dot &dot, float &xx, float &xy, float &yy) {
    if (!(dot.impactWeight > 0) || !isfinite(dot.impactWeight) ||
        !isfinite(dot.impactScale)) return false;
    const float normal2 = dot.impactX * dot.impactX + dot.impactY * dot.impactY;
    if (!isfinite(normal2) || normal2 < 0.000001f) return false;
    const float weight = dot.impactWeight > 1 ? 1 : dot.impactWeight;
    const float scale = fmaxf(0.65f, fminf(1.12f, dot.impactScale));
    const float across = 1.0f / scale;
    const float directional = (scale - across) / normal2;
    const float impactXX = across + directional * dot.impactX * dot.impactX;
    const float impactXY = directional * dot.impactX * dot.impactY;
    const float impactYY = across + directional * dot.impactY * dot.impactY;
    xx += (impactXX - xx) * weight;
    xy += (impactXY - xy) * weight;
    yy += (impactYY - yy) * weight;
    // Blending positive symmetric matrices keeps the outline bounded; restoring
    // determinant one preserves area throughout the squash/rebound transition.
    const float normalize = 1.0f / sqrtf(xx * yy - xy * xy);
    xx *= normalize;
    xy *= normalize;
    yy *= normalize;
    return true;
  }

  void drawPolygon(uint8_t *current, const Dot &dot, float radius,
                   const ShapePolygon &polygon, float stretchMax) {
    const float extentScale = dot.impactWeight > 0 ? fmaxf(stretchMax, 1.0f / 0.65f) : stretchMax;
    const float maximumExtent = radius * extentScale + 0.5f;
    if (!isfinite(dot.x) || !isfinite(dot.y) ||
        dot.x < -maximumExtent || dot.y < -maximumExtent ||
        dot.x > kSize - 1 + maximumExtent || dot.y > kSize - 1 + maximumExtent) return;

    float xx, xy, yy;
    velocityTransform(dot, dot.vx * dot.vx + dot.vy * dot.vy, stretchMax, xx, xy, yy);
    if (dot.impactWeight > 0) blendImpact(dot, xx, xy, yy);
    xx *= radius;
    xy *= radius;
    yy *= radius;

    ShapeVertex vertices[10];
    float minX = kSize + maximumExtent, maxX = -maximumExtent;
    float minY = minX, maxY = maxX;
    for (int i = 0; i < polygon.count; ++i) {
      const ShapeVertex &unit = polygon.vertices[i];
      ShapeVertex &vertex = vertices[i];
      vertex.x = dot.x + xx * unit.x + xy * unit.y;
      vertex.y = dot.y + xy * unit.x + yy * unit.y;
      if (vertex.x < minX) minX = vertex.x;
      if (vertex.x > maxX) maxX = vertex.x;
      if (vertex.y < minY) minY = vertex.y;
      if (vertex.y > maxY) maxY = vertex.y;
    }
    const int left = maximum(0, shapeFloor(minX - 0.5f));
    const int right = minimum(kSize - 1, shapeCeil(maxX + 0.5f));
    const int top = maximum(0, shapeFloor(minY - 0.5f));
    const int bottom = minimum(kSize - 1, shapeCeil(maxY + 0.5f));
    if (left > right || top > bottom) return;
    mark(occupied_[drawing_], left, top, right, bottom);

    ShapeEdge edges[10];
    int edgeCount = 0;
    for (int i = 0; i < polygon.count; ++i) {
      ShapeVertex from = vertices[i], to = vertices[i + 1 == polygon.count ? 0 : i + 1];
      if (from.y > to.y) { const ShapeVertex swap = from; from = to; to = swap; }
      const int first = maximum(0, shapeCeil(2 * from.y + 0.5f));
      const int last = minimum(2 * kSize, shapeCeil(2 * to.y + 0.5f));
      if (first >= last) continue; // Includes horizontal and sub-sample edges.
      const float slope = (to.x - from.x) / (to.y - from.y);
      ShapeEdge &edge = edges[edgeCount++];
      edge.first = first;
      edge.last = last;
      edge.x = from.x + (first * 0.5f - 0.25f - from.y) * slope;
      edge.step = slope * 0.5f;
    }

    ShapeCoverage coverage;
    const uint8_t solid = static_cast<uint8_t>(4 + (dot.color & 7) * 4);
    for (int y = top; y <= bottom; ++y) {
      coverage.first = kSize;
      coverage.last = -1;
      const int rowLeft = clipCircle_ ? maximum(left, clipLeft_[y]) : left;
      const int rowRight = clipCircle_ ? minimum(right, clipRight_[y]) : right;
      for (int sample = 0; sample < 2; ++sample) {
        const int scanline = 2 * y + sample;
        float crossings[10];
        int crossingCount = 0;
        for (int edgeIndex = 0; edgeIndex < edgeCount; ++edgeIndex) {
          ShapeEdge &edge = edges[edgeIndex];
          if (scanline < edge.first || scanline >= edge.last) continue;
          const float x = edge.x;
          edge.x += edge.step;
          int index = crossingCount++;
          while (index && crossings[index - 1] > x) {
            crossings[index] = crossings[index - 1];
            --index;
          }
          crossings[index] = x;
        }
        // Even/odd pairs correctly leave the five-point star's notches empty.
        for (int i = 0; i + 1 < crossingCount; i += 2)
          addShapeSpan(coverage, left, rowLeft, rowRight, crossings[i], crossings[i + 1]);
      }
      if (coverage.first > coverage.last) continue;
      const uint16_t *source = coverage.pixels + (coverage.first - left);
      uint8_t *destination = current + y * kSize + coverage.first;
      const uint8_t *end = current + y * kSize + coverage.last + 1;
      for (; destination < end; ++destination, ++source) {
        int amount = (*source + 32) >> 6;
        if (!amount) continue;
        if (amount >= 4) { *destination = solid; continue; }
        uint8_t &pixel = *destination;
        const int oldCoverage = pixel ? ((pixel - 1) & 3) + 1 : 0;
        if (amount >= oldCoverage) pixel = static_cast<uint8_t>(solid - 4 + amount);
      }
    }
  }

  template <bool Stretched>
  void drawDot(uint8_t *current, const Dot &dot, float extentX, float extentY,
               float inner2, float outer2, float edgeScale, float a, float b, float c) {
    // Bound before float-to-int conversion, including corrupt snapshots.
    if (!isfinite(dot.x) || !isfinite(dot.y) ||
        dot.x < -extentX || dot.y < -extentY ||
        dot.x > kSize - 1 + extentX || dot.y > kSize - 1 + extentY) return;
    const int left = maximum(0, static_cast<int>(floorf(dot.x - extentX)));
    const int top = maximum(0, static_cast<int>(floorf(dot.y - extentY)));
    const int right = minimum(kSize - 1, static_cast<int>(ceilf(dot.x + extentX)));
    const int bottom = minimum(kSize - 1, static_cast<int>(ceilf(dot.y + extentY)));
    mark(occupied_[drawing_], left, top, right, bottom);
    const uint8_t solid = static_cast<uint8_t>(4 + (dot.color & 7) * 4);
    if (outer2 >= 400.0f) {
      // Large ellipses have broad solid interiors. Solve two horizontal spans
      // per row, fill the interior in bytes, and shade only the narrow edges.
      // This also avoids scanning the empty corners of diagonal ellipses.
      drawLargeDot<Stretched>(current, dot, left, top, right, bottom,
                             inner2, outer2, edgeScale, a, b, c, solid);
      return;
    }
    for (int y = top; y <= bottom; ++y) {
      const int rowLeft = clipCircle_ ? maximum(left, clipLeft_[y]) : left;
      const int rowRight = clipCircle_ ? minimum(right, clipRight_[y]) : right;
      const float dy = y - dot.y;
      const float firstX = rowLeft - dot.x;
      float ellipseDistance2 = a * firstX * firstX + 2 * b * firstX * dy + c * dy * dy;
      float distanceStep = a * (2 * firstX + 1) + 2 * b * dy;
      for (int x = rowLeft; x <= rowRight; ++x) {
        const float dx = x - dot.x;
        const float distance2 = Stretched ? ellipseDistance2 : dx * dx + dy * dy;
        // Only two additions per ellipse pixel. The circle instantiation
        // compiles out this recurrence and keeps the original distance math.
        if (Stretched) { ellipseDistance2 += distanceStep; distanceStep += 2 * a; }
        if (distance2 >= outer2) continue;
        if (distance2 <= inner2) {
          current[y * kSize + x] = solid;
          continue;
        }
        int coverage = static_cast<int>((outer2 - distance2) * edgeScale + 0.5f);
        if (coverage > 4) coverage = 4;
        if (coverage < 1) continue;
        uint8_t &pixel = current[y * kSize + x];
        const int oldCoverage = pixel ? ((pixel - 1) & 3) + 1 : 0;
        // A translucent edge must not erase the solid center of a neighbor.
        if (coverage >= oldCoverage)
          pixel = static_cast<uint8_t>(solid - 4 + coverage);
      }
    }
  }

  template <bool Stretched>
  void drawLargeDot(uint8_t *current, const Dot &dot,
                    int left, int top, int right, int bottom,
                    float inner2, float outer2, float edgeScale,
                    float a, float b, float c, uint8_t solid) {
    const float inverseA = Stretched ? 1.0f / a : 1.0f;
    const float rowScale = Stretched ? c - b * b * inverseA : 1.0f;
    for (int y = top; y <= bottom; ++y) {
      const int rowLeft = clipCircle_ ? maximum(left, clipLeft_[y]) : left;
      const int rowRight = clipCircle_ ? minimum(right, clipRight_[y]) : right;
      if (rowLeft > rowRight) continue;
      const float dy = y - dot.y;
      const float rowDistance = rowScale * dy * dy;
      if (rowDistance >= outer2) continue;
      const float center = Stretched ? dot.x - b * dy * inverseA : dot.x;
      const float outerSpan = sqrtf((outer2 - rowDistance) * inverseA);
      const int first = maximum(rowLeft, shapeFloor(center - outerSpan));
      const int last = minimum(rowRight, shapeCeil(center + outerSpan));
      if (first > last) continue;
      int solidFirst = last + 1, solidLast = last;
      if (rowDistance < inner2) {
        const float innerSpan = sqrtf((inner2 - rowDistance) * inverseA);
        solidFirst = maximum(first, shapeCeil(center - innerSpan));
        solidLast = minimum(last, shapeFloor(center + innerSpan));
        if (solidFirst <= solidLast)
          memset(current + y * kSize + solidFirst, solid, solidLast - solidFirst + 1);
        else { solidFirst = last + 1; solidLast = last; }
      }
      for (int side = 0; side < 2; ++side) {
        const int edgeFirst = side ? solidLast + 1 : first;
        const int edgeLast = side ? last : solidFirst - 1;
        for (int x = edgeFirst; x <= edgeLast; ++x) {
          const float dx = x - dot.x;
          const float distance2 = Stretched ?
              a * dx * dx + 2 * b * dx * dy + c * dy * dy : dx * dx + dy * dy;
          if (distance2 >= outer2) continue;
          int coverage = static_cast<int>((outer2 - distance2) * edgeScale + 0.5f);
          if (coverage > 4) coverage = 4;
          if (coverage < 1) continue;
          uint8_t &pixel = current[y * kSize + x];
          const int oldCoverage = pixel ? ((pixel - 1) & 3) + 1 : 0;
          if (coverage >= oldCoverage)
            pixel = static_cast<uint8_t>(solid - 4 + coverage);
        }
      }
    }
  }
  static void mark(uint32_t (&mask)[kTiles][2], int left, int top, int right, int bottom) {
    for (int y = top / kTileSize; y <= bottom / kTileSize; ++y)
      for (int x = left / kTileSize; x <= right / kTileSize; ++x)
        mask[y][x / 32] |= uint32_t(1) << (x & 31);
  }
  static int minimum(int a, int b) { return a < b ? a : b; }
  static int maximum(int a, int b) { return a > b ? a : b; }
};

}  // namespace render
}  // namespace sand

#if defined(ARDUINO_ARCH_ESP32) && defined(__GNUC__) && !defined(__clang__)
#pragma GCC pop_options
#endif

#endif
