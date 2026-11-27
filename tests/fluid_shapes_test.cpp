#include "../Gravity_Sand/FluidRenderer.h"
#include "../Gravity_Sand/FluidVariation.h"

#include <assert.h>
#include <float.h>
#include <stdio.h>
#include <string.h>

using sand::render::Dot;
using sand::render::FluidRaster;
using sand::render::Options;
using sand::render::PixelRect;
using sand::render::Shape;

static FluidRaster raster;
static uint8_t panel[240 * 240];

// Model exactly what the LCD receives, including erasure of old silhouettes.
static int repaint(const Options &options = Options(1, 4, 3)) {
  int windows = 0;
  raster.planPixels(options, [&](const PixelRect &rect) {
    assert(rect.w && rect.h && rect.x + rect.w <= 240 && rect.y + rect.h <= 240);
    assert(rect.h <= options.maxBandRows * 4);
    ++windows;
    for (int y = rect.y; y < rect.y + rect.h; ++y)
      for (int x = rect.x; x < rect.x + rect.w; ++x)
        panel[y * 240 + x] = raster.pixel(x, y);
  });
  for (int y = 0; y < 240; ++y)
    assert(memcmp(panel + y * 240, raster.row(y), 240) == 0);
  raster.commit();
  return windows;
}

static int rowInk(int y) {
  int count = 0;
  for (int x = 0; x < 240; ++x) count += raster.pixel(x, y) != 0;
  return count;
}

static int partialPixels() {
  int count = 0;
  for (int y = 0; y < 240; ++y)
    for (int x = 0; x < 240; ++x) {
      const uint8_t pixel = raster.pixel(x, y);
      if (pixel && ((pixel - 1) & 3) != 3) ++count;
    }
  return count;
}

struct Moments { double xx, yy, xy; };

static Moments moments(float centerX, float centerY) {
  Moments result = {};
  for (int y = 0; y < 240; ++y)
    for (int x = 0; x < 240; ++x) {
      const uint8_t pixel = raster.pixel(x, y);
      if (!pixel) continue;
      const int coverage = ((pixel - 1) & 3) + 1;
      const double dx = x - centerX, dy = y - centerY;
      result.xx += coverage * dx * dx;
      result.yy += coverage * dy * dy;
      result.xy += coverage * dx * dy;
    }
  return result;
}

static void visibleSilhouettes() {
  Dot dot(120.25f, 120.5f, 2);
  raster.draw(&dot, 1, 12.5f, Shape::Triangle);
  assert(raster.pixel(120, 120));
  assert(rowInk(124) > rowInk(114) * 2); // Upright triangle widens toward its base.
  repaint();

  raster.draw(&dot, 1, 12.5f, Shape::Square);
  const int squareWidth = rowInk(120);
  assert(rowInk(114) == squareWidth && rowInk(124) == squareWidth);
  assert(raster.pixel(127, 127)); // Square has filled corners.
  repaint();

  raster.draw(&dot, 1, 12.5f, Shape::Star);
  assert(raster.pixel(120, 111)); // Upper point.
  assert(raster.pixel(114, 128) && raster.pixel(126, 128)); // Lower points.
  assert(!raster.pixel(120, 128)); // Real concave valley, not a convex hull.
  repaint();

  for (int index = 0; index < static_cast<int>(Shape::Count); ++index) {
    const Shape shape = static_cast<Shape>(index);
    assert(sand::render::validShape(shape));
    assert(sand::render::shapeName(shape)[0]);
    raster.draw(&dot, 1, 12.5f, shape);
    assert(partialPixels() > 0);
    repaint();
    dot.x += 0.25f;
    raster.draw(&dot, 1, 12.5f, shape);
    assert(repaint() > 0); // Subpixel movement changes the visible edge.
    raster.draw(&dot, 1, 12.5f, shape);
    assert(repaint() == 0);
  }
  puts("Shapes have distinct silhouettes, a concave five-point star, and subpixel edge coverage.");
}

static void deformationAndInvalidVelocity() {
  const float speed = sand::config::kDotStretchFullSpeed;
  Dot dot(120.25f, 120.5f, 2);
  for (int index = 1; index < static_cast<int>(Shape::Count); ++index) {
    const Shape shape = static_cast<Shape>(index);
    dot.vx = speed; dot.vy = 0;
    raster.draw(&dot, 1, 12.5f, shape);
    const Moments horizontal = moments(dot.x, dot.y);
    assert(horizontal.xx > 8 * horizontal.yy);
    repaint();
    dot.vx = 0; dot.vy = speed;
    raster.draw(&dot, 1, 12.5f, shape);
    const Moments vertical = moments(dot.x, dot.y);
    assert(vertical.yy > 8 * vertical.xx);
    assert(repaint() > 0);
    dot.vx = speed; dot.vy = -speed;
    raster.draw(&dot, 1, 12.5f, shape);
    const Moments diagonal = moments(dot.x, dot.y);
    assert(diagonal.xy < -0.3 * (diagonal.xx + diagonal.yy));
    assert(repaint() > 0);

    dot.vx = dot.vy = 0;
    raster.draw(&dot, 1, 12.5f, shape);
    repaint();
    const float invalid[][2] = {{NAN, 0}, {0, INFINITY}, {FLT_MAX, FLT_MAX}};
    for (const auto &velocity : invalid) {
      dot.vx = velocity[0]; dot.vy = velocity[1];
      raster.draw(&dot, 1, 12.5f, shape);
      assert(repaint() == 0);
    }
  }
  puts("Every polygon stretches along travel and safely ignores invalid velocities.");
}

static void populationAndShapeChanges() {
  Dot dots[120];
  const float fullSpeed = sand::config::kDotStretchFullSpeed;
  for (int i = 0; i < 120; ++i)
    dots[i] = Dot(10.25f + (i % 12) * 19.5f, 10.5f + (i / 12) * 23,
                  static_cast<uint8_t>(i & 7));

  // A stationary population changes its shape without moving its centers.
  for (int index = 0; index < static_cast<int>(Shape::Count); ++index) {
    const Shape shape = static_cast<Shape>(index);
    raster.draw(dots, 120, 2.5f, shape);
    assert(repaint() > 0);
    raster.draw(dots, 120, 2.5f, shape);
    assert(repaint() == 0);
  }

  // Change silhouette, stretch direction, size and population together. Every
  // old tip and removed particle must still be erased by bounded transfers.
  for (int phase = 0; phase < 20; ++phase) {
    const Shape shape = static_cast<Shape>(phase % static_cast<int>(Shape::Count));
    const bool large = phase & 1;
    const int count = large ? 4 : 120;
    for (int i = 0; i < 120; ++i) {
      dots[i].x += (i & 1) ? 0.125f : -0.25f;
      dots[i].y += (i & 2) ? -0.25f : 0.125f;
      dots[i].vx = (phase % 3 == 0) ? 0 : ((i & 1) ? fullSpeed : -fullSpeed);
      dots[i].vy = (phase % 3 == 1) ? 0 : ((i & 2) ? fullSpeed : -fullSpeed);
    }
    raster.draw(dots, count, large ? 12.5f : 2.5f, shape);
    assert(repaint(Options(phase % 3, 1 + phase % 8, phase % 5)) > 0);
    raster.draw(dots, count, large ? 12.5f : 2.5f, shape);
    assert(repaint() == 0);
  }
  raster.draw(nullptr, 0, 12.5f, Shape::Star);
  assert(repaint() > 0);
  assert(repaint() == 0);
  puts("Shape, size, population and deformation changes repaint without trails.");
}

static void clippingAndFallbacks() {
  const float speed = sand::config::kDotStretchFullSpeed;
  Dot edges[] = {{0, 0, 0, speed, speed}, {239, 239, 1, -speed, speed},
                 {-15, 120, 2, speed, 0}, {224, 120, 3, speed, 0},
                 {120, 224, 4, 0, speed}, {NAN, 0, 5}, {FLT_MAX, 0, 6}};
  for (int clip = 0; clip < 2; ++clip) {
    raster.setClipCircle(clip ? 117.5f : 0);
    for (int index = 0; index < static_cast<int>(Shape::Count); ++index) {
      raster.draw(edges, 7, 12.5f, static_cast<Shape>(index));
      assert(rowInk(120) > 0);
      if (clip) {
        for (int y = 0; y < 240; ++y)
          for (int x = 0; x < 240; ++x) {
            const float dx = x - 119.5f, dy = y - 119.5f;
            if (dx * dx + dy * dy > 117.5f * 117.5f)
              assert(raster.pixel(x, y) == 0);
          }
      }
      repaint(Options(0, 1, 0));
    }
  }
  raster.setClipCircle(0);
  Dot dot(100.25f, 100.5f, 7, speed, -speed);
  raster.draw(&dot, 1, 12.5f, Shape::Circle);
  repaint();
  const Shape invalid[] = {Shape::Count, static_cast<Shape>(255)};
  for (Shape shape : invalid) {
    assert(!sand::render::validShape(shape));
    raster.draw(&dot, 1, 12.5f, shape);
    assert(repaint() == 0); // Invalid shape uses exactly the existing circle.
  }
  raster.draw(nullptr, 0, 12.5f, Shape::Star);
  repaint();
  puts("Stretched shapes clip safely at screen and circular edges; invalid shapes fall back to circles.");
}

// Count shared visible pixels from separately drawn silhouettes, independent
// of the order or colors used to composite the actual pair.
static int sharedInk(const Dot (&dots)[2], float radius, Shape shape) {
  static uint8_t first[240 * 240];
  raster.draw(dots, 1, radius, shape);
  for (int y = 0; y < 240; ++y) memcpy(first + y * 240, raster.row(y), 240);
  raster.draw(dots + 1, 1, radius, shape);
  int shared = 0;
  for (int y = 0; y < 240; ++y)
    for (int x = 0; x < 240; ++x)
      shared += first[y * 240 + x] != 0 && raster.pixel(x, y) != 0;
  return shared;
}

static void visualOverlapAndExpandedBounds() {
  // Exercise retained renderer capacity independently of the randomize bounds.
  const float physicalRadius = sand::config::kDotRadius * sand::config::kMaxPackedScale;
  const float visualRadius = physicalRadius * sand::config::kVisualSizeScale;
  const float separation = 2 * physicalRadius + 2;
  const Dot pair[] = {{90.25f, 120.5f, 2}, {90.25f + separation, 120.5f, 6}};
  for (int index = 0; index < static_cast<int>(Shape::Count); ++index) {
    const Shape shape = static_cast<Shape>(index);
    assert(sharedInk(pair, physicalRadius, shape) == 0);
    assert(sharedInk(pair, visualRadius, shape) > 0);
    raster.draw(pair, 2, visualRadius, shape);
    assert(raster.pixel(static_cast<int>(pair[0].x), 120) &&
           raster.pixel(static_cast<int>(pair[1].x), 120));
    assert(repaint() > 0);
    raster.draw(pair, 2, visualRadius, shape);
    assert(repaint() == 0);
    raster.draw(pair, 2, physicalRadius, shape);
    assert(repaint() > 0); // Retract every enlarged edge without a trail.
    assert(pair[0].x == 90.25f && pair[1].x == 90.25f + separation);
    assert(pair[0].y == 120.5f && pair[1].y == 120.5f);
  }

  // Full-speed 100px polygons can extend beyond the entire 240px screen.
  // Exercise horizontal, vertical and diagonal footprints at every screen
  // edge, then enable circular clipping and remove the enlarged population.
  const float speed = sand::config::kDotStretchFullSpeed;
  const float directions[][2] = {{1, 0}, {0, 1}, {1, 1}, {1, -1}};
  Dot edges[] = {{0, 0, 0}, {239, 0, 1}, {0, 239, 2}, {239, 239, 3},
                 {0, 120, 4}, {239, 120, 5}, {120, 0, 6}, {120, 239, 7},
                 {119.5f, 119.5f, 3}};
  for (int clip = 0; clip < 2; ++clip) {
    raster.setClipCircle(clip ? 117.5f : 0);
    for (int index = 0; index < static_cast<int>(Shape::Count); ++index)
      for (const auto &direction : directions) {
        for (Dot &dot : edges) {
          dot.vx = direction[0] * speed;
          dot.vy = direction[1] * speed;
        }
        raster.draw(edges, 9, visualRadius, static_cast<Shape>(index));
        assert(rowInk(120) > 0);
        if (clip)
          for (int y = 0; y < 240; ++y)
            for (int x = 0; x < 240; ++x) {
              const float dx = x - 119.5f, dy = y - 119.5f;
              if (dx * dx + dy * dy > 117.5f * 117.5f)
                assert(raster.pixel(x, y) == 0);
            }
        assert(repaint(Options(0, 1, 0)) > 0);
        raster.draw(edges, 9, visualRadius, static_cast<Shape>(index));
        assert(repaint() == 0);
      }
  }
  raster.draw(nullptr, 0, visualRadius, Shape::Star);
  assert(repaint() > 0);
  raster.setClipCircle(0);
  puts("100px visuals overlap separated particles; 3x stretched outlines clip to one scratch row and erase exactly.");
}

struct Random {
  uint32_t state;
  uint32_t operator()() {
    state ^= state << 13; state ^= state >> 17; state ^= state << 5;
    return state;
  }
};
struct FixedRandom {
  uint32_t value;
  int calls;
  uint32_t operator()() { ++calls; return value; }
};

static void randomShapeSelection() {
  const int count = static_cast<int>(Shape::Count);
  assert(count == 4); // Circle, triangle, square and star; octagons remain removed.
  assert(!sand::render::validShape(static_cast<Shape>(4)));
  Random random = {0x83c25a19};
  for (int current = 0; current < count; ++current) {
    int samples[static_cast<int>(Shape::Count)] = {};
    for (int i = 0; i < 20000; ++i) {
      const Shape next = sand::variation::sampleShape(random, static_cast<Shape>(current));
      assert(sand::render::validShape(next));
      assert(static_cast<int>(next) != current);
      ++samples[static_cast<int>(next)];
    }
    for (int next = 0; next < count; ++next) {
      if (next == current) assert(samples[next] == 0);
      else {
        const float expected = 20000.0f / (count - 1);
        assert(samples[next] > expected * 0.9f && samples[next] < expected * 1.1f);
      }
    }
    const uint32_t stuckBits[] = {0, UINT32_MAX};
    for (uint32_t bits : stuckBits) {
      FixedRandom fixed = {bits, 0};
      const Shape next = sand::variation::sampleShape(fixed, static_cast<Shape>(current));
      assert(sand::render::validShape(next) && static_cast<int>(next) != current);
      assert(fixed.calls > 0 && fixed.calls <= 64);
    }
  }
  assert(sand::render::validShape(sand::variation::sampleShape(random, static_cast<Shape>(255))));
  puts("Shape randomization reaches every alternative uniformly, avoids repeats, and bounds stuck RNG work.");
}

int main() {
  visibleSilhouettes();
  deformationAndInvalidVelocity();
  populationAndShapeChanges();
  clippingAndFallbacks();
  visualOverlapAndExpandedBounds();
  randomShapeSelection();
}
