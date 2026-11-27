#include "fluid_test_support.h"
#include "../Gravity_Sand/FluidCoverage.h"
#include "../Gravity_Sand/FluidRenderer.h"

#include <float.h>

using sand::render::Dot;
using sand::render::FluidRaster;
using sand::render::Shape;
using sand::render::SpawnCoverage;

static FluidRaster raster;
static SpawnCoverage coverage;
static Dot dots[Simulation::kMaxParticleCount];

static double renderedCoverage(int count, float radius, Shape shape) {
  raster.draw(dots, count, radius, shape);
  double ink = 0;
  for (int y = 0; y < FluidRaster::kSize; ++y) {
    const uint8_t *row = raster.row(y);
    for (int x = 0; x < FluidRaster::kSize; ++x)
      if (row[x]) ink += ((row[x] - 1) % 4 + 1) * 0.25;
  }
  raster.commit();
  return ink / (3.141592653589793 * 120 * 120);
}

static float measuredCoverage(int count, float radius, Shape shape) {
  return coverage.measure(count, radius, shape,
                          [](int i) -> const Dot & { return dots[i]; });
}

static void packedSpawnCoverage() {
  const float scales[] = {1, 1.01f, 1.25f, 1.5f, 2, 2.25f, 3, 3.49f, 3.5f};
  Simulation simulation;
  double minimum = 1, maximum = 0, largestError = 0;
  unsigned samples = 0;
  for (int shapeIndex = 0; shapeIndex < static_cast<int>(Shape::Count); ++shapeIndex) {
    const Shape shape = static_cast<Shape>(shapeIndex);
    for (float scale : scales) {
      const auto range = sand::variation::visualCountRange(scale, shape);
      const int counts[] = {range.low, range.baseline, range.high};
      for (int count : counts) {
        for (uint32_t sample = 0; sample < 12; ++sample) {
          assert(simulation.configurePacked(scale, count, (sample + 1) * 0x9e3779b9u));
          for (int i = 0; i < count; ++i) {
            const auto &p = simulation.particle(i);
            dots[i] = {p.x, p.y, static_cast<uint8_t>(i % 5), 0, 0};
          }
          const float measured = measuredCoverage(count, scale * 5, shape);
          const double rendered = renderedCoverage(count, scale * 5, shape);
          const double error = fabs(measured - rendered);
          // The production guard measures pixel centers; the renderer has AA.
          // Check their independent raster paths against the actual visible ink.
          // Explicit previews may be sparse at the smallest sizes and the
          // particle cap. The population fitter enforces randomize coverage.
          assert(measured > 0 && measured < 1);
          assert(rendered > 0 && rendered < 1);
          assert(error < 0.02);
          if (rendered < minimum) minimum = rendered;
          if (rendered > maximum) maximum = rendered;
          if (error > largestError) largestError = error;
          ++samples;
        }
      }
    }
  }
  printf("%u packed spawns across all shapes: visible AA union %.2f..%.2f%% of the full circular screen; coverage guard differs by <=%.3f percentage points.\n",
         samples, minimum * 100, maximum * 100, largestError * 100);
}

static void overlapAndClipping() {
  const Shape shapes[] = {Shape::Circle, Shape::Triangle, Shape::Square, Shape::Star};
  for (Shape shape : shapes) {
    dots[0] = {100.5f, 120.5f, 0};
    dots[1] = dots[0];
    const float single = measuredCoverage(1, 10, shape);
    assert(single > 0);
    assert(measuredCoverage(2, 10, shape) == single); // Count union, not summed area.
    dots[1].x += 4;
    const float overlapping = measuredCoverage(2, 10, shape);
    assert(overlapping > single && overlapping < 2 * single);
    dots[1].x = 140.5f;
    assert(fabsf(measuredCoverage(2, 10, shape) - 2 * single) < 0.00001f);
    const float interior = measuredCoverage(1, 50, shape);
    dots[0].x = 0;
    const float clipped = measuredCoverage(1, 50, shape);
    assert(clipped > 0 && clipped < interior);
    assert(fabs(clipped - renderedCoverage(1, 50, shape)) < 0.005);
    dots[0] = {-55, 120, 0};
    assert(measuredCoverage(1, 50, shape) == 0);
  }
  dots[0] = {119.5f, 119.5f, 0};
  const float star = measuredCoverage(1, 40, Shape::Star);
  const float circle = measuredCoverage(1, 40, Shape::Circle);
  assert(star < circle * 0.6f); // Star notches must remain empty.
  assert(measuredCoverage(1, 40, static_cast<Shape>(255)) == circle);
  assert(measuredCoverage(0, 40, Shape::Circle) == 0);
  assert(measuredCoverage(-1, 40, Shape::Circle) == 0);
  assert(measuredCoverage(Simulation::kMaxParticleCount + 1, 40, Shape::Circle) == 0);
  assert(measuredCoverage(1, NAN, Shape::Circle) == 0);
  assert(measuredCoverage(1, INFINITY, Shape::Circle) == 0);
  assert(measuredCoverage(1, -1, Shape::Circle) == 0);
  assert(measuredCoverage(1, 51, Shape::Circle) == 0);
  dots[0] = {NAN, 120, 0};
  assert(measuredCoverage(1, 40, Shape::Circle) == 0);
  puts("Spawn coverage counts clipped unions, preserves star notches, and rejects invalid inputs.");
}

static void singleOddOverlapAndClipping() {
  const Shape shapes[] = {Shape::Circle, Shape::Triangle, Shape::Square, Shape::Star};
  const int oddPositions[] = {0, 2, 5};
  double largestError = 0;
  for (Shape majority : shapes) for (Shape alternate : shapes) {
    if (majority == alternate) continue;
    for (int oddIndex : oddPositions) {
      // A clipped dot, overlapping dots, and interior dots share the same union.
      dots[0] = {3.5f, 119.5f, 0};
      dots[1] = {90.25f, 110.5f, 1};
      dots[2] = {102.25f, 112.5f, 2};
      dots[3] = {125.5f, 149.25f, 3};
      dots[4] = {145.5f, 149.25f, 4};
      dots[5] = {120.5f, 195.25f, 0};
      dots[oddIndex].shape = alternate;
      const auto particleAt = [](int i) -> const Dot & { return dots[i]; };
      const float measured = coverage.measure(6, 17.5f, majority, particleAt, oddIndex, alternate);
      const double rendered = renderedCoverage(6, 17.5f, majority);
      const double error = fabs(measured - rendered);
      assert(error < 0.005);
      if (error > largestError) largestError = error;
      const float plain = coverage.measure(6, 17.5f, majority, particleAt);
      assert(coverage.measure(6, 17.5f, majority, particleAt, -1, alternate) == plain);
      assert(coverage.measure(6, 17.5f, majority, particleAt, 6, alternate) == plain);
      assert(coverage.measure(6, 17.5f, majority, particleAt, oddIndex, static_cast<Shape>(255)) == plain);
      assert(coverage.measure(6, 17.5f, majority, particleAt, oddIndex, majority) == plain);
    }
  }
  printf("Single odd silhouettes preserve clipped overlap unions for every shape pair; AA difference <=%.3fpp.\n",
         largestError * 100);
}

int main() {
  raster.setClipCircle(sand::config::kCircleRadius + 0.5f);
  packedSpawnCoverage();
  overlapAndClipping();
  singleOddOverlapAndClipping();
}
