#include "../Gravity_Sand/FluidRenderer.h"

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
static uint8_t reference[240 * 240];

static int repaint() {
  int windows = 0;
  raster.planPixels(Options(1, 4, 3), [&](const PixelRect &rect) {
    assert(rect.w && rect.h && rect.x + rect.w <= 240 && rect.y + rect.h <= 240);
    assert(rect.h <= 16);
    ++windows;
    for (int y = rect.y; y < rect.y + rect.h; ++y)
      for (int x = rect.x; x < rect.x + rect.w; ++x)
        panel[y * 240 + x] = raster.pixel(x, y);
  });
  for (int y = 0; y < 240; ++y) assert(memcmp(panel + y * 240, raster.row(y), 240) == 0);
  raster.commit();
  return windows;
}

struct Ink { double area = 0, xx = 0, xy = 0, yy = 0; };
static Ink ink(float cx, float cy) {
  Ink result;
  for (int y = 0; y < 240; ++y) for (int x = 0; x < 240; ++x) {
    const uint8_t pixel = raster.pixel(x, y);
    if (!pixel) continue;
    const double coverage = ((pixel - 1) & 3) + 1;
    const double dx = x - cx, dy = y - cy;
    result.area += coverage;
    result.xx += coverage * dx * dx;
    result.xy += coverage * dx * dy;
    result.yy += coverage * dy * dy;
  }
  return result;
}

static void impactGeometry() {
  for (int shapeIndex = 0; shapeIndex < static_cast<int>(Shape::Count); ++shapeIndex) {
    const Shape shape = static_cast<Shape>(shapeIndex);
    Dot dot(119.5f, 119.5f, 2);
    raster.draw(&dot, 1, 17.5f, shape, 3);
    repaint();
    memcpy(reference, panel, sizeof(panel));
    const Ink base = ink(dot.x, dot.y);
    dot.vx = 400; // Travel would elongate X; wall contact overrides that direction.
    dot.impactX = 2; // Normal need not be unit length.
    dot.impactScale = 0.65f;
    dot.impactWeight = 1;
    raster.draw(&dot, 1, 17.5f, shape, 3);
    assert(repaint() > 0);
    const Ink squash = ink(dot.x, dot.y);
    assert(fabs(squash.area / base.area - 1) < 0.06);
    assert(squash.xx < base.xx * 0.55 && squash.yy > base.yy * 1.9);
    raster.draw(&dot, 1, 17.5f, shape, 3);
    assert(repaint() == 0);

    dot.impactScale = 1.10f;
    raster.draw(&dot, 1, 17.5f, shape, 3);
    assert(repaint() > 0);
    const Ink rebound = ink(dot.x, dot.y);
    assert(fabs(rebound.area / base.area - 1) < 0.06);
    assert(rebound.xx > base.xx * 1.10 && rebound.yy < base.yy * 0.92);

    dot.vx = 0;
    dot.impactWeight = 0;
    raster.draw(&dot, 1, 17.5f, shape, 3);
    repaint();
    assert(memcmp(panel, reference, sizeof(panel)) == 0);

    // Axis and weight changes alone must invalidate the snapshot cache.
    dot.impactWeight = 0.5f;
    dot.impactScale = 0.65f;
    raster.draw(&dot, 1, 17.5f, shape, 3);
    assert(repaint() > 0);
    dot.impactX = 0;
    dot.impactY = 1;
    raster.draw(&dot, 1, 17.5f, shape, 3);
    assert(repaint() > 0);
    const Ink vertical = ink(dot.x, dot.y);
    assert(vertical.xx > base.xx && vertical.yy < base.yy);
  }
  Dot dot(119.5f, 119.5f, 0);
  dot.impactX = dot.impactY = 1;
  dot.impactWeight = 1;
  dot.impactScale = 0.65f;
  raster.draw(&dot, 1, 17.5f);
  repaint();
  const Ink diagonal = ink(dot.x, dot.y);
  assert(diagonal.xy < -diagonal.xx * 0.5);
  puts("Every shape compresses along the impact normal, rebounds, preserves area and repaints without trails.");
}

static void singleOddShape() {
  Dot dots[] = {{40, 45, 0}, {100, 45, 1}, {160, 45, 2}, {65, 130, 3}, {155, 130, 4}};
  raster.draw(dots, 5, 15, Shape::Square, 1);
  repaint();
  memcpy(reference, panel, sizeof(panel));
  dots[2].shape = Shape::Star;
  raster.draw(dots, 5, 15, Shape::Square, 1);
  assert(repaint() > 0);
  int changed = 0;
  for (int y = 0; y < 240; ++y) for (int x = 0; x < 240; ++x) {
    if (panel[y * 240 + x] != reference[y * 240 + x]) {
      ++changed;
      assert(x >= 144 && x <= 176 && y >= 29 && y <= 61);
    }
    if (panel[y * 240 + x] && x >= 144 && x <= 176 && y >= 29 && y <= 61)
      assert((panel[y * 240 + x] - 1) / 4 == 2);
  }
  assert(changed > 10);
  raster.draw(dots, 5, 15, Shape::Square, 1);
  assert(repaint() == 0);
  dots[2].shape = Shape::Count;
  raster.draw(dots, 5, 15, Shape::Square, 1);
  repaint();
  assert(memcmp(panel, reference, sizeof(panel)) == 0);

  // Default-circle populations can contain a polygon, and vice versa.
  const Shape shapes[] = {Shape::Circle, Shape::Triangle, Shape::Square, Shape::Star};
  for (Shape population : shapes) for (Shape odd : shapes) {
    dots[2].shape = odd;
    raster.draw(dots, 5, 15, population, 2);
    repaint();
    Dot separate = dots[2];
    separate.shape = Shape::Count;
    static FluidRaster independent;
    independent.draw(&separate, 1, 15, odd, 2);
    for (int y = 29; y <= 61; ++y) for (int x = 144; x <= 176; ++x)
      assert(raster.pixel(x, y) == independent.pixel(x, y));
    independent.commit();
  }
  dots[2].shape = static_cast<Shape>(255);
  raster.draw(dots, 5, 15, Shape::Square, 1);
  repaint();
  assert(memcmp(panel, reference, sizeof(panel)) == 0);

  Dot overlap[] = {{120, 120, 0}, {120, 120, 1}};
  overlap[1].shape = Shape::Star;
  raster.draw(overlap, 2, 20, Shape::Square, 1);
  repaint();
  assert((raster.pixel(120, 120) - 1) / 4 == 1); // Odd star is the top layer.
  assert((raster.pixel(120, 133) - 1) / 4 == 0); // Its valley reveals the square.
  const Dot swap = overlap[0]; overlap[0] = overlap[1]; overlap[1] = swap;
  raster.draw(overlap, 2, 20, Shape::Square, 1);
  repaint();
  assert((raster.pixel(120, 120) - 1) / 4 == 0); // Reordering preserves painter order.
  assert((raster.pixel(120, 101) - 1) / 4 == 1); // Star's upper tip remains exposed.
  puts("One odd particle preserves every other silhouette and color, supports all shape pairs, and invalidates caches correctly.");
}

static void invalidImpactAndBounds() {
  const float invalid[] = {NAN, INFINITY, -INFINITY, FLT_MAX};
  for (int s = 0; s < static_cast<int>(Shape::Count); ++s) {
    const Shape shape = static_cast<Shape>(s);
    Dot original(120, 120, 1, 300, -100);
    raster.draw(&original, 1, 14, shape, 3);
    repaint();
    memcpy(reference, panel, sizeof(panel));
    for (float value : invalid) {
      Dot dot = original;
      dot.impactWeight = 1;
      dot.impactX = value;
      dot.impactY = value;
      dot.impactScale = 0.65f;
      raster.draw(&dot, 1, 14, shape, 3);
      repaint();
      assert(memcmp(panel, reference, sizeof(panel)) == 0);
    }
    for (int field = 0; field < 2; ++field) {
      Dot dot = original;
      dot.impactX = 1;
      dot.impactWeight = field == 0 ? NAN : 1;
      dot.impactScale = field == 1 ? NAN : 0.65f;
      raster.draw(&dot, 1, 14, shape, 3);
      repaint();
      assert(memcmp(panel, reference, sizeof(panel)) == 0);
    }
  }
  raster.setClipCircle(117.5f);
  for (int frame = 0; frame < 100; ++frame) {
    Dot dot(119.5f + 115 * cosf(frame * 0.37f), 119.5f + 115 * sinf(frame * 0.37f), frame % 5,
            650 * cosf(frame * 0.19f), 650 * sinf(frame * 0.19f));
    dot.impactX = cosf(frame * 0.37f);
    dot.impactY = sinf(frame * 0.37f);
    dot.impactScale = frame & 1 ? -FLT_MAX : FLT_MAX;
    dot.impactWeight = frame % 3 ? 0.5f : FLT_MAX;
    dot.shape = static_cast<Shape>(frame % static_cast<int>(Shape::Count));
    raster.draw(&dot, 1, FluidRaster::kMaxDotRadius, Shape::Circle, frame & 1 ? 1 : 3);
    repaint();
    for (int y = 0; y < 240; ++y) for (int x = 0; x < 240; ++x)
      if ((x - 119.5f) * (x - 119.5f) + (y - 119.5f) * (y - 119.5f) > 117.5f * 117.5f)
        assert(!raster.pixel(x, y));
  }
  puts("Invalid impact inputs fall back safely; maximum-size mixed effects stay clipped and erase exactly.");
}

int main() {
  impactGeometry();
  singleOddShape();
  invalidImpactAndBounds();
}
