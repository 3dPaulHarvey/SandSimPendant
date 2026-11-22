#ifndef GRAVITY_SAND_DIRTY_RENDERER_H
#define GRAVITY_SAND_DIRTY_RENDERER_H

#include <stdint.h>

namespace sand {
namespace render {

// Coordinates are simulation cells, not display pixels. The caller expands
// each cell to its pixel square and draws colors from the immutable new frame.
struct Rect {
  uint8_t x, y, w, h;
};

struct Options {
  int mergeGapCells;       // Redraw short unchanged gaps to save SPI commands.
  int maxBandRows;         // Maximum rectangle height; bounds the pixel buffer.
  int mergeOverdrawCells;  // Extra cells allowed per vertical rectangle merge.

  Options(int gap = 4, int rows = 4, int overdraw = 8)
      : mergeGapCells(gap), maxBandRows(rows), mergeOverdrawCells(overdraw) {}
};

// Visit dirty rectangles without storing a frame-sized list or allocating.
// Horizontal spans bridge small gaps; adjacent rows join when the extra pixel
// traffic is cheaper than another display window. At most 40 disjoint spans
// fit on an 80-cell row, so storage stays bounded even for a checkerboard.
// Rectangles may overlap, but always repaint the exact colors from current.
// Do not mutate current or previous from emit; update previous after plan().
template <class Emit>
bool plan(const uint8_t *current, const uint8_t *previous, int width,
          int height, const Options &options, Emit emit) {
  if (!current || !previous || width < 1 || width > 80 ||
      height < 1 || height > 80) return false;

  const int gap = options.mergeGapCells < 0 ? 0 : options.mergeGapCells;
  const int rows = options.maxBandRows < 1 ? 1 :
                   (options.maxBandRows > 80 ? 80 : options.maxBandRows);
  const int overdraw = options.mergeOverdrawCells < 0 ? 0 :
                       options.mergeOverdrawCells;
  Rect active[40], next[40];
  int activeCount = 0;

  for (int y = 0; y < height; ++y) {
    // Flush at band boundaries so h never exceeds the caller's strip buffer.
    if (y % rows == 0) {
      for (int i = 0; i < activeCount; ++i) emit(active[i]);
      activeCount = 0;
    }
    bool used[40] = {};
    int nextCount = 0;
    const int row = y * width;
    int x = 0;
    while (x < width) {
      while (x < width && current[row + x] == previous[row + x]) ++x;
      if (x == width) break;
      const int first = x;
      int last = x;
      for (++x; x < width; ++x) {
        if (current[row + x] != previous[row + x]) last = x;
        else if (x - last > gap) break;
      }
      const int spanWidth = last - first + 1;

      int best = -1, bestExtra = 6401;
      int bestLeft = first, bestRight = last + 1;
      for (int i = 0; i < activeCount; ++i) {
        if (used[i]) continue;
        const Rect &r = active[i];
        const int left = first < r.x ? first : r.x;
        const int oldRight = r.x + r.w;
        const int right = last + 1 > oldRight ? last + 1 : oldRight;
        const int extra = (right - left) * (r.h + 1) - r.w * r.h - spanWidth;
        if (extra <= overdraw && extra < bestExtra) {
          best = i;
          bestExtra = extra;
          bestLeft = left;
          bestRight = right;
        }
      }
      Rect &result = next[nextCount++];
      if (best >= 0) {
        const Rect &r = active[best];
        used[best] = true;
        result.x = static_cast<uint8_t>(bestLeft);
        result.y = r.y;
        result.w = static_cast<uint8_t>(bestRight - bestLeft);
        result.h = static_cast<uint8_t>(r.h + 1);
      } else {
        result.x = static_cast<uint8_t>(first);
        result.y = static_cast<uint8_t>(y);
        result.w = static_cast<uint8_t>(spanWidth);
        result.h = 1;
      }
    }
    for (int i = 0; i < activeCount; ++i)
      if (!used[i]) emit(active[i]);
    activeCount = nextCount;
    for (int i = 0; i < activeCount; ++i) active[i] = next[i];
  }
  for (int i = 0; i < activeCount; ++i) emit(active[i]);
  return true;
}

}  // namespace render
}  // namespace sand

#endif
