#include "../Gravity_Sand/DirtyRenderer.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

using sand::render::Options;
using sand::render::Rect;

static const int kSize = 80;
static uint8_t current[kSize * kSize], previous[kSize * kSize];

struct Result {
  int windows, cells;
};

// Apply the planned paint operations to the old image. Every case must produce
// the entire requested new image, including erased grains and bridged gaps.
static Result repaint(const Options &options, int width = kSize,
                      int height = kSize) {
  uint8_t painted[kSize * kSize];
  memcpy(painted, previous, sizeof(painted));
  Result result = {};
  const int maximumRows = options.maxBandRows < 1 ? 1 :
                          (options.maxBandRows > 80 ? 80 : options.maxBandRows);
  assert(sand::render::plan(current, previous, width, height, options,
    [&](const Rect &r) {
      assert(r.w > 0 && r.h > 0);
      assert(r.x + r.w <= width && r.y + r.h <= height);
      assert(r.h <= maximumRows);
      assert(r.y / maximumRows == (r.y + r.h - 1) / maximumRows);
      ++result.windows;
      result.cells += r.w * r.h;
      for (int y = r.y; y < r.y + r.h; ++y)
        for (int x = r.x; x < r.x + r.w; ++x)
          painted[y * width + x] = current[y * width + x];
    }));
  assert(memcmp(painted, current, width * height) == 0);
  return result;
}

static uint32_t rng = 0x12345678;
static uint32_t randomValue() {
  rng ^= rng << 13;
  rng ^= rng >> 17;
  rng ^= rng << 5;
  return rng;
}

int main() {
  const Options defaults;
  assert(repaint(defaults).windows == 0);

  // A moving grain requires both the old position to erase and new to paint.
  previous[20 * kSize + 20] = 4;
  current[20 * kSize + 23] = 4;
  Result moved = repaint(defaults);
  assert(moved.windows == 1 && moved.cells == 4);

  memset(previous, 0, sizeof(previous));
  memset(current, 0, sizeof(current));
  // Sixteen separate row transfers collapse into one compact rectangle.
  for (int y = 12; y < 16; ++y)
    for (int x = 10; x <= 22; x += 4) current[y * kSize + x] = 2;
  Result individual = repaint(Options(0, 1, 0));
  Result batched = repaint(defaults);
  assert(individual.windows == 16 && batched.windows == 1);
  printf("Clustered changes: %d windows -> %d windows, %d cells repainted.\n",
         individual.windows, batched.windows, batched.cells);

  // Distant changes are not combined into a giant rectangle just because
  // they occupy adjacent rows. The budget bounds added vertical overdraw.
  memset(current, 0, sizeof(current));
  current[10 * kSize + 1] = 1;
  current[11 * kSize + 78] = 1;
  assert(repaint(defaults).windows == 2);

  // Full invalidation paints every cell, split only by bounded band height.
  memset(previous, 0xFF, sizeof(previous));
  memset(current, 0, sizeof(current));
  Result full = repaint(defaults);
  assert(full.windows == 20 && full.cells == kSize * kSize);

  // Overlay/status removal forces a small region to repaint even if logical
  // sand colors have not changed. No sentinels may leak into the new image.
  memset(previous, 0, sizeof(previous));
  for (int y = 8; y < 16; ++y)
    for (int x = 24; x < 56; ++x) previous[y * kSize + x] = 0xFF;
  Result status = repaint(defaults);
  assert(status.windows == 2 && status.cells == 32 * 8);

  // Worst-case span count, all four corners, and complete palette changes.
  memset(previous, 0, sizeof(previous));
  for (int y = 0; y < kSize; ++y)
    for (int x = 0; x < kSize; ++x)
      current[y * kSize + x] = ((x + y) & 1) ? 0 : 8;
  repaint(Options(0, 4, 0));
  repaint(defaults);

  // Random successive frames cover old-cell erasure, color-only changes,
  // arbitrary sparse movement and every legal coordinate and band edge.
  for (int frame = 0; frame < 250; ++frame) {
    memcpy(previous, current, sizeof(previous));
    for (int i = 0; i < kSize * kSize; ++i)
      if (randomValue() % 7 == 0) current[i] = randomValue() % 9;
    repaint(Options(frame % 7, 1 + frame % 9, frame % 30));
  }

  // Odd dimensions and defensive option normalization retain full coverage.
  memset(previous, 0xFF, sizeof(previous));
  memset(current, 0, sizeof(current));
  repaint(Options(-1, 0, -1), 79, 77);
  repaint(Options(100, 100, 100000), 1, 1);
  int callbacks = 0;
  auto ignored = [&](const Rect &) { ++callbacks; };
  assert(!sand::render::plan(current, previous, 81, 80, defaults, ignored));
  assert(!sand::render::plan(current, previous, 80, 0, defaults, ignored));
  assert(!sand::render::plan(nullptr, previous, 80, 80, defaults, ignored));
  assert(callbacks == 0);
  puts("Dirty rectangles repaint complete frames with bounded bands and fewer windows.");
  return 0;
}
