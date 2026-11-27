#include "../Gravity_Sand/FluidRenderer.h"

#include <assert.h>
#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using sand::render::Dot;
using sand::render::FluidRaster;
using sand::render::Options;
using sand::render::PixelRect;

static FluidRaster raster;
static uint8_t panel[240 * 240];

struct Transfers { int windows = 0, pixels = 0; };

// Apply exactly the scheduled SPI windows to a model of the old display.
// This catches trails, missed subpixel changes, and incorrect merge coverage.
static Transfers repaint(const Options &options = Options(1, 4, 3)) {
  Transfers result;
  raster.planPixels(options, [&](const PixelRect &r) {
    assert(r.w && r.h && r.x + r.w <= 240 && r.y + r.h <= 240);
    assert(r.h <= options.maxBandRows * 4);
    ++result.windows;
    result.pixels += r.w * r.h;
    for (int y = r.y; y < r.y + r.h; ++y)
      for (int x = r.x; x < r.x + r.w; ++x)
        panel[y * 240 + x] = raster.pixel(x, y);
  });
  for (int y = 0; y < 240; ++y)
    for (int x = 0; x < 240; ++x)
      assert(panel[y * 240 + x] == raster.pixel(x, y));
  raster.commit();
  return result;
}

static int litPixels() {
  int count = 0;
  for (int y = 0; y < 240; ++y)
    for (int x = 0; x < 240; ++x) count += raster.pixel(x, y) != 0;
  return count;
}

struct Ink {
  double coverage = 0, xx = 0, yy = 0, xy = 0;
};

// Coverage-weighted image measurements test the visible shape, independently
// of the renderer's ellipse coefficients and scanline implementation.
static Ink measureInk(float centerX, float centerY) {
  Ink ink;
  for (int y = 0; y < 240; ++y)
    for (int x = 0; x < 240; ++x) {
      const uint8_t pixel = raster.pixel(x, y);
      if (!pixel) continue;
      const int coverage = ((pixel - 1) & 3) + 1;
      const double dx = x - centerX, dy = y - centerY;
      ink.coverage += coverage;
      ink.xx += coverage * dx * dx;
      ink.yy += coverage * dy * dy;
      ink.xy += coverage * dx * dy;
    }
  return ink;
}

static void assertPicture(const uint8_t *picture) {
  for (int y = 0; y < 240; ++y)
    assert(memcmp(raster.row(y), picture + y * 240, 240) == 0);
}

static void velocityShapes() {
  static uint8_t roundPicture[240 * 240], stretchedPicture[240 * 240];
  Dot dot(100.25f, 100.5f, 3);
  const float fullSpeed = sand::config::kDotStretchFullSpeed;

  // Check both tiny everyday dots and the largest selectable dots. Their
  // centers stay fixed, so these redraws exercise changes in shape alone.
  const float radii[] = {2.5f, 12.5f};
  for (float radius : radii) {
    // At 3x stretch a 5px dot is only 1.67px wide: pixel sampling varies
    // with subpixel phase. Guard each frame, then check average ink below.
    const double areaTolerance = radius <= 2.5f ? 0.30 : 0.12;
    dot.vx = dot.vy = 0;
    raster.draw(&dot, 1, radius);
    repaint();
    memcpy(roundPicture, panel, sizeof(panel));
    const Ink round = measureInk(dot.x, dot.y);

    dot.vx = sand::config::kDotStretchStartSpeed * 0.5f;
    raster.draw(&dot, 1, radius);
    assertPicture(roundPicture);
    assert(repaint().windows == 0); // Quiet motion cannot cause shape chatter.

    dot.vx = fullSpeed;
    raster.draw(&dot, 1, radius);
    const Ink horizontal = measureInk(dot.x, dot.y);
    assert(horizontal.xx > 3 * horizontal.yy);
    assert(fabs(horizontal.coverage - round.coverage) < round.coverage * areaTolerance);
    assert(repaint().windows > 0);
    memcpy(stretchedPicture, panel, sizeof(panel));

    dot.vx = fullSpeed * 2;
    raster.draw(&dot, 1, radius);
    assertPicture(stretchedPicture); // Extra speed cannot grow the footprint.
    assert(repaint().windows == 0);

    dot.vx = 0;
    dot.vy = fullSpeed;
    raster.draw(&dot, 1, radius);
    const Ink vertical = measureInk(dot.x, dot.y);
    assert(vertical.yy > 3 * vertical.xx);
    assert(fabs(vertical.coverage - round.coverage) < round.coverage * areaTolerance);
    assert(repaint(Options(0, 1, 0)).windows > 0);

    dot.vx = dot.vy = fullSpeed;
    raster.draw(&dot, 1, radius);
    const Ink diagonal = measureInk(dot.x, dot.y);
    assert(diagonal.xy > 0.25 * (diagonal.xx + diagonal.yy));
    assert(fabs(diagonal.coverage - round.coverage) < round.coverage * areaTolerance);
    assert(repaint().windows > 0);

    dot.vy = -fullSpeed;
    raster.draw(&dot, 1, radius);
    const Ink reverseDiagonal = measureInk(dot.x, dot.y);
    assert(reverseDiagonal.xy < -0.25 * (reverseDiagonal.xx + reverseDiagonal.yy));
    assert(repaint().windows > 0);

    dot.vx = dot.vy = 0;
    raster.draw(&dot, 1, radius);
    assertPicture(roundPicture);
    assert(repaint().windows > 0); // Retracted tips must be erased.

    const float invalidVelocity[][2] = {
      {NAN, 0}, {0, INFINITY}, {-INFINITY, 1}, {FLT_MAX, FLT_MAX}
    };
    for (const auto &velocity : invalidVelocity) {
      dot.vx = velocity[0];
      dot.vy = velocity[1];
      raster.draw(&dot, 1, radius);
      assertPicture(roundPicture);
      assert(repaint().windows == 0);
    }
  }

  // Average the tiny dot over a 4x4 quarter-pixel grid, so conserved area
  // cannot pass merely because one favorable sample lands on a pixel row.
  double roundTotal = 0, stretchedTotal[4] = {};
  const float directions[][2] = {{1, 0}, {0, 1}, {1, 1}, {1, -1}};
  for (int phaseY = 0; phaseY < 4; ++phaseY)
    for (int phaseX = 0; phaseX < 4; ++phaseX) {
      Dot sample(100 + phaseX * 0.25f, 100 + phaseY * 0.25f, 3);
      raster.draw(&sample, 1, 2.5f);
      const double roundInk = measureInk(sample.x, sample.y).coverage;
      roundTotal += roundInk;
      for (int direction = 0; direction < 4; ++direction) {
        sample.vx = directions[direction][0] * fullSpeed;
        sample.vy = directions[direction][1] * fullSpeed;
        raster.draw(&sample, 1, 2.5f);
        const double stretchedInk = measureInk(sample.x, sample.y).coverage;
        assert(fabs(stretchedInk - roundInk) < roundInk * 0.30);
        stretchedTotal[direction] += stretchedInk;
      }
    }
  for (double total : stretchedTotal)
    assert(fabs(total - roundTotal) < roundTotal * 0.05);

  // A fast maximum-size ellipse may extend beyond any panel edge. Repainting
  // and removing it must remain bounded and restore every previously lit pixel.
  Dot edges[] = {{0, 0, 0, fullSpeed, fullSpeed},
                 {239, 0, 1, fullSpeed, -fullSpeed},
                 {0, 239, 2, fullSpeed, -fullSpeed},
                 {239, 239, 3, fullSpeed, fullSpeed},
                 {-15, 120, 4, fullSpeed, 0}};
  raster.draw(edges, 5, 12.5f);
  assert(raster.pixel(0, 0) && raster.pixel(239, 239));
  assert(raster.pixel(0, 120)); // Offscreen centers can have visible tips.
  repaint(Options(0, 1, 0));
  raster.draw(nullptr, 0, 12.5f);
  assert(repaint().windows > 0);
  assert(litPixels() == 0);
  puts("Velocity shapes: direction, preserved ink, speed cap and exact shape-only redraws passed.");
}

static void roundPanelClip() {
  const float fullSpeed = sand::config::kDotStretchFullSpeed;
  Dot dots[] = {{224, 119.5f, 0, fullSpeed, 0},
                {15, 119.5f, 1, -fullSpeed, 0},
                {119.5f, 224, 2, 0, fullSpeed},
                {119.5f, 15, 3, 0, -fullSpeed},
                {193, 193, 4, fullSpeed, fullSpeed},
                {46, 46, 5, -fullSpeed, -fullSpeed}};
  raster.draw(dots, 6, 12.5f);
  repaint();
  const int unclippedPixels = litPixels();

  const float clipRadius = 117.5f;
  raster.setClipCircle(clipRadius);
  raster.draw(dots, 6, 12.5f);
  assert(litPixels() > 0 && litPixels() < unclippedPixels);
  for (int y = 0; y < 240; ++y)
    for (int x = 0; x < 240; ++x) {
      const float dx = x - 119.5f, dy = y - 119.5f;
      if (dx * dx + dy * dy > clipRadius * clipRadius)
        assert(raster.pixel(x, y) == 0);
    }
  assert(repaint(Options(0, 1, 0)).windows > 0);
  raster.draw(dots, 6, 12.5f);
  assert(repaint().windows == 0);

  raster.setClipCircle(0); // Restoring rectangular rendering also restores tips.
  raster.draw(dots, 6, 12.5f);
  assert(litPixels() == unclippedPixels);
  assert(repaint().windows > 0);
  raster.draw(nullptr, 0, 12.5f);
  repaint();
  puts("Round-panel clipping contains maximum-size stretched dots and repaints exactly.");
}

static void populationStretchLimits() {
  using sand::render::Shape;
  static uint8_t restingPicture[240 * 240], fullPicture[240 * 240];
  const float speed = sand::config::kDotStretchFullSpeed;
  for (int index = 0; index < static_cast<int>(Shape::Count); ++index) {
    const Shape shape = static_cast<Shape>(index);
    Dot dot(119.25f, 119.5f, 3);
    raster.draw(&dot, 1, 12.5f, shape, 1);
    repaint();
    memcpy(restingPicture, panel, sizeof(panel));
    const Ink resting = measureInk(dot.x, dot.y);
    assert(resting.coverage > 0);

    dot.vx = speed;
    raster.draw(&dot, 1, 12.5f, shape, 1);
    assertPicture(restingPicture); // A 1x cap disables deformation at any speed.
    assert(repaint().windows == 0);

    // Only the cap changes: a snapshot cache must not hide this redraw.
    raster.draw(&dot, 1, 12.5f, shape, 2);
    const Ink medium = measureInk(dot.x, dot.y);
    assert(medium.xx > resting.xx * 3 && medium.yy < resting.yy * 0.4);
    assert(fabs(medium.coverage - resting.coverage) < resting.coverage * 0.12);
    assert(repaint().windows > 0);
    raster.draw(&dot, 1, 12.5f, shape, 2);
    assert(repaint().windows == 0);

    raster.draw(&dot, 1, 12.5f, shape, 3);
    const Ink full = measureInk(dot.x, dot.y);
    assert(full.xx > medium.xx * 1.7 && full.yy < medium.yy * 0.65);
    assert(fabs(full.coverage - resting.coverage) < resting.coverage * 0.12);
    assert(repaint().windows > 0);
    memcpy(fullPicture, panel, sizeof(panel));
    raster.draw(&dot, 1, 12.5f, shape); // Preserve the old default 3x rendering.
    assertPicture(fullPicture);
    assert(repaint().windows == 0);

    dot.vx = 0;
    dot.vy = speed;
    raster.draw(&dot, 1, 12.5f, shape, 2);
    const Ink vertical = measureInk(dot.x, dot.y);
    assert(vertical.yy > resting.yy * 3 && vertical.xx < resting.xx * 0.4);
    assert(fabs(vertical.coverage - resting.coverage) < resting.coverage * 0.12);
    assert(repaint().windows > 0);
    dot.vx = speed;
    raster.draw(&dot, 1, 12.5f, shape, 2);
    const Ink diagonal = measureInk(dot.x, dot.y);
    assert(diagonal.xy > 0.2 * (diagonal.xx + diagonal.yy));
    assert(repaint().windows > 0);

    dot.vy = 0;
    const float disabled[] = {NAN, INFINITY, -INFINITY, -1, 0, 1};
    for (float cap : disabled) {
      raster.draw(&dot, 1, 12.5f, shape, cap);
      assertPicture(restingPicture);
      repaint();
      raster.draw(&dot, 1, 12.5f, shape, cap);
      assert(repaint().windows == 0);
    }
    const float oversized[] = {4.0f, FLT_MAX};
    for (float cap : oversized) {
      raster.draw(&dot, 1, 12.5f, shape, cap);
      assertPicture(fullPicture); // Finite over-range values cannot exceed 3x.
      repaint();
    }
    raster.draw(&dot, 1, 12.5f, shape, 1);
    assertPicture(restingPicture);
    assert(repaint().windows > 0); // The former stretched tips are erased.
  }
  raster.draw(nullptr, 0, 12.5f);
  repaint();
  puts("Per-population stretch caps preserve area/direction for every shape, repaint cap-only changes, and bound invalid values.");
}

static void changingPopulationAndSize() {
  Dot population[120];
  for (int i = 0; i < 120; ++i)
    population[i] = {10.0f + (i % 12) * 19.5f, 10.0f + (i / 12) * 23.0f,
                     static_cast<uint8_t>(i & 7)};
  const int counts[] = {120, 4, 100, 3, 80, 5, 120};
  const float radii[] = {2.5f, 12.5f, 2.5f, 12.5f, 2.5f, 11.5f, 2.5f};
  for (int phase = 0; phase < 7; ++phase) {
    raster.draw(population, counts[phase], radii[phase]);
    assert(litPixels() > 0);
    assert(repaint(Options(1, 1 + phase % 8, 3)).windows > 0);
    // No stale pixels from removed particles or the larger previous diameter.
    raster.draw(population, counts[phase], radii[phase]);
    assert(repaint().windows == 0);
  }

  // Four well-separated maximum-size dots keep about the original total ink.
  Dot large[] = {{40.25f, 40.5f, 0}, {199.75f, 40.5f, 2},
                 {40.25f, 199.5f, 4}, {199.75f, 199.5f, 7}};
  raster.draw(large, 4, 12.5f);
  const int fullInk = litPixels();
  assert(fullInk > 1800 && fullInk < 2200);
  assert(raster.pixel(40, 40) == 4 && raster.pixel(52, 52) == 0);
  repaint();
  large[0].x += 0.25f;
  raster.draw(large, 4, 12.5f);
  assert(repaint().windows > 0); // Large circles retain fractional edge motion.
  raster.draw(large, 3, 12.5f);
  assert(repaint().windows > 0);
  assert(!raster.pixel(199, 199)); // The removed fourth dot is erased completely.

  Dot corners[] = {{0, 0, 0}, {239, 0, 1}, {0, 239, 2}, {239, 239, 3}};
  raster.draw(corners, 4, 12.5f);
  repaint(Options(0, 1, 0)); // Every strip remains at most four pixels tall.
  raster.draw(corners, 4, 2.5f);
  repaint();
  raster.draw(nullptr, 0, 12.5f);
  repaint();
  assert(litPixels() == 0);
  puts("Changing 3..120 dots and 5..25px diameters repaint exactly without trails.");
}

static void giantCircleSpans() {
  // Independent rotation/axis geometry checks the optimized wide-row fill,
  // including ellipses whose full footprint exceeds the entire screen.
  const float radii[] = {20, 32.5f, 50};
  const float velocities[][2] = {{0, 0}, {400, 0}, {0, 400}, {400, 400}, {175, -365}};
  const float centers[][2] = {{119.25f, 119.5f}, {-23.25f, 140.5f}, {238.25f, 22.5f}};
  int boundaryRoundoff = 0;
  for (int clip = 0; clip < 2; ++clip) {
    raster.setClipCircle(clip ? 117.5f : 0);
    for (float radius : radii)
      for (const auto &velocity : velocities)
        for (const auto &center : centers) {
          const Dot dot(center[0], center[1], 2, velocity[0], velocity[1]);
          raster.draw(&dot, 1, radius);
          const double speed = sqrt(double(dot.vx) * dot.vx + double(dot.vy) * dot.vy);
          const double amount = speed > sand::config::kDotStretchStartSpeed ?
              fmin(1.0, (speed - sand::config::kDotStretchStartSpeed) /
                   (sand::config::kDotStretchFullSpeed - sand::config::kDotStretchStartSpeed)) : 0;
          const double stretch = 1 + (sand::config::kDotStretchMax - 1) * amount;
          const double nx = speed ? dot.vx / speed : 1, ny = speed ? dot.vy / speed : 0;
          const double outer2 = (radius + 0.5) * (radius + 0.5);
          const double inner2 = (radius - 0.5) * (radius - 0.5);
          for (int y = 0; y < 240; ++y)
            for (int x = 0; x < 240; ++x) {
              const uint8_t pixel = raster.pixel(x, y);
              const int coverage = pixel ? ((pixel - 1) & 3) + 1 : 0;
              const double viewX = x - 119.5, viewY = y - 119.5;
              if (clip && viewX * viewX + viewY * viewY > 117.5 * 117.5) {
                assert(!pixel);
                continue;
              }
              const double dx = x - double(dot.x), dy = y - double(dot.y);
              const double along = (dx * nx + dy * ny) / stretch;
              const double across = (-dx * ny + dy * nx) * stretch;
              const double raw = (outer2 - along * along - across * across) * 4 / (outer2 - inner2);
              const int expected = static_cast<int>(fmin(4.0, fmax(0.0, floor(raw + 0.5))));
              assert(!pixel || (pixel - 1) / 4 == 2);
              if (coverage != expected) {
                // Single-precision coefficients can round an AA threshold to
                // its neighboring coverage level, but never move the edge.
                assert(abs(coverage - expected) == 1);
                assert(fabs((raw - floor(raw)) - 0.5) < 0.025);
                ++boundaryRoundoff;
              }
            }
          repaint(Options(1, 2, 3));
          raster.draw(&dot, 1, radius);
          assert(repaint().windows == 0);
        }
  }
  raster.draw(nullptr, 0, 50);
  assert(repaint().windows > 0);
  assert(litPixels() == 0);
  raster.setClipCircle(0);
  printf("40-100px circle spans match independent ellipse geometry; %d AA threshold roundoffs, exact clipping/repainting.\n",
         boundaryRoundoff);
}

static void unchangedSnapshotReuse() {
  static FluidRaster reference;
  Dot dots[] = {{90.25f, 119.5f, 1, 190, 120}, {145.5f, 135.25f, 4, -110, 200}};
  float radius = 40;
  sand::render::Shape shape = sand::render::Shape::Star;
  raster.setClipCircle(117.5f);
  const auto checkReference = [&]() {
    // Resetting the clip deliberately prevents reuse in the independent draw.
    reference.setClipCircle(117.5f);
    reference.draw(dots, 2, radius, shape);
    for (int y = 0; y < 240; ++y)
      assert(memcmp(raster.row(y), reference.row(y), 240) == 0);
  };
  raster.draw(dots, 2, radius, shape);
  raster.draw(dots, 2, radius, shape); // Repeat before committing the new image.
  checkReference();
  assert(repaint().windows > 0);
  raster.draw(dots, 2, radius, shape);
  assert(repaint().windows == 0);

  for (int field = 0; field < 7; ++field) {
    if (field == 0) dots[0].x += 0.0625f;
    if (field == 1) dots[0].y -= 0.0625f;
    if (field == 2) ++dots[0].color;
    if (field == 3) dots[0].vx += 50;
    if (field == 4) dots[0].vy -= 50;
    if (field == 5) radius += 0.25f;
    if (field == 6) shape = sand::render::Shape::Triangle;
    raster.draw(dots, 2, radius, shape);
    raster.draw(dots, 2, radius, shape);
    checkReference();
    assert(repaint().windows > 0);
    raster.draw(dots, 2, radius, shape);
    assert(repaint().windows == 0);
  }

  // A discarded intermediate draw cannot replace the last committed image.
  dots[0].x += 40;
  raster.draw(dots, 2, radius, shape);
  dots[0].x -= 40;
  raster.draw(dots, 2, radius, shape);
  checkReference();
  assert(repaint().windows == 0);
  raster.draw(dots, 2, radius, shape);
  raster.invalidateColors();
  assert(repaint().windows > 0);
  for (int y = 12; y < 20; ++y)
    for (int x = 40; x < 100; ++x) panel[y * 240 + x] = 0xEE;
  raster.invalidate(40, 12, 60, 8); // Background/overlay work survives reuse.
  raster.draw(dots, 2, radius, shape);
  assert(repaint().windows > 0);

  raster.setClipCircle(40);
  raster.draw(dots, 2, radius, shape);
  assert(!raster.pixel(90, 70));
  assert(repaint().windows > 0);
  raster.setClipCircle(117.5f);
  raster.draw(dots, 2, radius, shape);
  checkReference();
  assert(repaint().windows > 0);
  raster.draw(dots, 1, radius, shape);
  assert(repaint().windows > 0);
  raster.draw(dots, 2, radius, shape);
  checkReference();
  assert(repaint().windows > 0);
  raster.draw(dots, 2, NAN, shape);
  assert(litPixels() == 0 && repaint().windows > 0);
  raster.draw(dots, 2, radius, shape);
  checkReference();
  assert(repaint().windows > 0);
  raster.draw(nullptr, 0, radius);
  assert(litPixels() == 0 && repaint().windows > 0);
  raster.setClipCircle(0);
  puts("Exact snapshot reuse preserves subpixels, pending draws, palette/background invalidations, and clip changes.");
}

int main() {
  assert(repaint().windows == 0);
  Dot dot = {100.0f, 100.0f, 0};
  raster.draw(&dot, 1, 2.5f);
  assert(raster.pixel(100, 100) == 4);
  assert(!raster.pixel(97, 97)); // Round silhouette, rather than a solid square.
  assert(repaint().windows > 0);
  raster.draw(&dot, 1, 2.5f);
  assert(repaint().windows == 0);

  // A quarter-pixel move changes edge coverage before the center rounds.
  dot.x += 0.25f;
  raster.draw(&dot, 1, 2.5f);
  assert(repaint().windows > 0);

  Dot dots[100];
  for (int i = 0; i < 100; ++i) {
    dots[i] = {12.0f + (i % 10) * 23.2f, 12.0f + (i / 10) * 23.1f,
               static_cast<uint8_t>(i & 7)};
  }
  int windows = 0, pixels = 0;
  for (int frame = 0; frame < 180; ++frame) {
    for (int i = 0; i < 100; ++i) {
      dots[i].x += (i & 1) ? -0.27f : 0.31f;
      dots[i].y += (i & 2) ? -0.23f : 0.29f;
    }
    raster.draw(dots, 100, 2.5f);
    const Transfers t = repaint(Options(frame % 3, 1 + frame % 8, frame % 5));
    windows += t.windows;
    pixels += t.pixels;
  }
  printf("Scattered moving dots: %.1f windows, %.1f pixels/frame (varied batching).\n",
         windows / 180.0, pixels / 180.0);

  // RGB changes invalidate occupied pixels even with identical color indices.
  raster.invalidateColors();
  assert(repaint().windows > 0);
  assert(repaint().windows == 0);
  // Overlay removal must restore exactly what the raster says beneath it.
  for (int y = 24; y < 48; ++y)
    for (int x = 72; x < 168; ++x) panel[y * 240 + x] = 0xEE;
  raster.invalidate(72, 24, 96, 24);
  assert(repaint().windows > 0);

  // Overlapping translucent edges never erase an opaque neighbor's center.
  Dot pair[] = {{100, 100, 7}, {102.5f, 100, 0}};
  raster.draw(pair, 2, 2.5f);
  assert(raster.pixel(100, 100) == 32);
  repaint();

  Dot extremes[] = {{0, 0, 1}, {239, 239, 2}, {NAN, 0, 3},
                    {FLT_MAX, 0, 4}, {-FLT_MAX, 0, 5}, {0, INFINITY, 6}};
  raster.draw(extremes, 6, 2.5f);
  assert(raster.pixel(0, 0) && raster.pixel(239, 239));
  repaint();
  raster.draw(nullptr, 0, 2.5f);
  assert(repaint().windows > 0);
  assert(repaint().windows == 0);
  assert(FluidRaster::shade(0xFFFF, 4) == 0xFFFF);
  assert(FluidRaster::shade(0xFFFF, 1) == 0x39E7);
  changingPopulationAndSize();
  velocityShapes();
  populationStretchLimits();
  roundPanelClip();
  giantCircleSpans();
  unchangedSnapshotReuse();
  puts("Fluid dots: subpixel coverage, exact repaint, overlaps and overlay erasure passed.");
}
