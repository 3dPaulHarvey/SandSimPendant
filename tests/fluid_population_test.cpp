#include "fluid_test_support.h"
#include "../Gravity_Sand/FluidPopulation.h"
#include "../Gravity_Sand/FluidRenderer.h"

using sand::render::Shape;
static sand::render::SpawnCoverage coverage;
static sand::render::FluidRaster raster;
static sand::render::Dot dots[Simulation::kMaxParticleCount];

static double visibleInk(const Simulation &simulation, Shape shape,
                         const sand::variation::OddShape &odd = sand::variation::OddShape()) {
  for (int i = 0; i < simulation.particleCount(); ++i) {
    const auto &particle = simulation.particle(i);
    dots[i] = {particle.x, particle.y, static_cast<uint8_t>(i % 5), 0, 0};
    if (i == odd.index(simulation.particleCount())) dots[i].shape = odd.shape;
  }
  raster.draw(dots, simulation.particleCount(), simulation.dotRadius() * 2, shape);
  double ink = 0;
  for (int y = 0; y < raster.kSize; ++y) {
    const uint8_t *row = raster.row(y);
    for (int x = 0; x < raster.kSize; ++x)
      if (row[x]) ink += ((row[x] - 1) % 4 + 1) * 0.25;
  }
  raster.commit();
  return ink / sand::render::SpawnCoverage::kScreenArea;
}

static void fittedCoverageAcrossShapesSizesAndSeeds() {
  const float sizes[] = {1, 1.01f, 1.25f, 1.5f, 2, 2.25f, 3, 3.49f, 3.5f};
  const float targets[] = {0.16f, 0.18f, 0.20f, 0.22f, 0.25f, 0.28f, 0.31f, 0.34f};
  Simulation simulation;
  double largestFitError = 0, largestAAError = 0;
  double minimum = 1, maximum = 0;
  int maxMeasurements = 0, samples = 0, totalMeasurements = 0, fastFits = 0;
  for (int index = 0; index < static_cast<int>(Shape::Count); ++index) {
    const Shape shape = static_cast<Shape>(index);
    for (float preferred : sizes) {
      for (float target : targets) {
        for (uint32_t sample = 1; sample <= 24; ++sample) {
          const uint32_t seed = sample * 0x9e3779b9u;
          const auto fitted = sand::variation::fitPopulation(simulation, coverage, preferred, target, seed, shape);
          assert(fitted.scale >= 1 && fitted.scale <= 3.5f);
          assert(fitted.count >= 5 && fitted.count <= Simulation::maxPackedCountForScale(fitted.scale));
          assert(fitted.scale == simulation.sizeScale() && fitted.count == simulation.particleCount());
          assert(fitted.measurements >= 1 && fitted.measurements <= sand::variation::kMaxPopulationMeasurements);
          checkFluid(simulation);
          const float checked = coverage.measure(fitted.count, fitted.scale * 5, shape,
            [&](int i) -> const sand::Particle & { return simulation.particle(i); });
          assert(checked == fitted.coverage); // Final simulation matches the reported candidate.
          const double actual = visibleInk(simulation, shape);
          const double fitError = fabs(fitted.coverage - target);
          const double aaError = fabs(actual - fitted.coverage);
          if (fitError > largestFitError) largestFitError = fitError;
          if (aaError > largestAAError) largestAAError = aaError;
          if (fitted.measurements > maxMeasurements) maxMeasurements = fitted.measurements;
          totalMeasurements += fitted.measurements;
          if (fitted.measurements <= 2) ++fastFits;
          if (fitError > 0.01) fprintf(stderr, "FAILED shape%d preferred%.4f target%.4f seed%u fit%.6f size%.4f count%d measurements%d\n",
              index, preferred, target, seed, fitted.coverage, fitted.scale, fitted.count, fitted.measurements);
          assert(fitError < 0.01);
          assert(aaError < 0.01);
          assert(actual >= 0.15 && actual <= 0.35);
          if (actual < minimum) minimum = actual;
          if (actual > maximum) maximum = actual;
          ++samples;
        }
      }
    }
  }
  assert(totalMeasurements / double(samples) < 5);
  printf("%d fitted spawns: AA union %.3f..%.3f%%; target error <=%.3fpp, AA difference <=%.3fpp; checks average %.2f, max %d, %.1f%% finish in 1-2.\n",
    samples, minimum * 100, maximum * 100,
    largestFitError * 100, largestAAError * 100, totalMeasurements / double(samples),
    maxMeasurements, fastFits * 100.0 / samples);
}

static void determinismAndInvalidInputs() {
  Simulation simulation, repeat;
  const auto first = sand::variation::fitPopulation(simulation, coverage, 3.5f, 0.32f, 123, Shape::Circle);
  const auto second = sand::variation::fitPopulation(repeat, coverage, 3.5f, 0.32f, 123, Shape::Circle);
  assert(first.scale == second.scale && first.count == second.count && first.coverage == second.coverage);
  equalFluid(simulation, repeat);
  assert(first.scale <= 3.5f && fabsf(first.coverage - 0.32f) < 0.005f); // Fit stays inside the smaller bounds.
  const float invalid[] = {NAN, INFINITY, -1, 0, 100};
  for (float value : invalid) {
    const auto fitted = sand::variation::fitPopulation(simulation, coverage, value, value, 0, static_cast<Shape>(255));
    assert(fitted.scale >= 1 && fitted.scale <= 3.5f && fitted.measurements <= 20);
    assert(fitted.coverage > 0.15 && fitted.coverage < 0.35);
    checkFluid(simulation);
  }
  puts("Population fitting is deterministic, reserves enough dots for five colors, and bounds invalid inputs.");
}

static void fittedSingleOddShapes() {
  using sand::variation::OddShape;
  const float sizes[] = {1, 1.625f, 2.25f, 3.5f};
  const float targets[] = {0.16f, 0.22f, 0.28f, 0.34f};
  const uint32_t slots[] = {0, 0x80324defu, UINT32_MAX};
  Simulation simulation;
  int samples = 0, totalChecks = 0, maximumChecks = 0;
  double minimum = 1, maximum = 0, maximumError = 0;
  for (int main = 0; main < static_cast<int>(Shape::Count); ++main) {
    const Shape majority = static_cast<Shape>(main);
    for (int other = 0; other < static_cast<int>(Shape::Count); ++other) {
      if (main == other) continue;
      for (float preferred : sizes) for (float target : targets) for (uint32_t slot : slots) {
        const OddShape odd(static_cast<Shape>(other), slot);
        for (uint32_t sample = 1; sample <= 6; ++sample) {
          const auto fitted = sand::variation::fitPopulation(simulation, coverage, preferred, target,
            sample * 0x9e3779b9u, majority, odd);
          assert(fitted.scale >= 1 && fitted.scale <= 3.5f);
          assert(fitted.count >= 5 && fitted.count <= Simulation::maxPackedCountForScale(fitted.scale));
          assert(fitted.measurements <= sand::variation::kMaxPopulationMeasurements);
          const float checked = coverage.measure(fitted.count, fitted.scale * 5, majority,
            [&](int i) -> const sand::Particle & { return simulation.particle(i); },
            odd.index(fitted.count), odd.shape);
          assert(checked == fitted.coverage);
          const double actual = visibleInk(simulation, majority, odd);
          assert(fabs(checked - target) < 0.01 && fabs(checked - actual) < 0.01);
          assert(actual >= 0.15 && actual <= 0.35);
          if (actual < minimum) minimum = actual;
          if (actual > maximum) maximum = actual;
          if (fabs(checked - target) > maximumError) maximumError = fabs(checked - target);
          totalChecks += fitted.measurements;
          if (fitted.measurements > maximumChecks) maximumChecks = fitted.measurements;
          ++samples;
        }
      }
    }
  }
  Simulation reference;
  const auto plain = sand::variation::fitPopulation(reference, coverage, 3.5f, 0.34f, 17, Shape::Triangle);
  const OddShape noops[] = {OddShape(), OddShape(static_cast<Shape>(255), UINT32_MAX),
                           OddShape(Shape::Triangle, UINT32_MAX)};
  for (const auto &odd : noops) {
    const auto noChange = sand::variation::fitPopulation(simulation, coverage, 3.5f, 0.34f, 17, Shape::Triangle, odd);
    assert(noChange.scale == plain.scale && noChange.count == plain.count && noChange.coverage == plain.coverage);
    equalFluid(simulation, reference);
  }
  printf("%d single-odd-shape fits: AA union %.3f..%.3f%%, target error <=%.3fpp; checks average %.2f, max %d.\n",
    samples, minimum * 100, maximum * 100, maximumError * 100, totalChecks / double(samples), maximumChecks);
}

int main() {
  raster.setClipCircle(sand::config::kCircleRadius + 0.5f);
  fittedCoverageAcrossShapesSizesAndSeeds();
  determinismAndInvalidInputs();
  fittedSingleOddShapes();
}
