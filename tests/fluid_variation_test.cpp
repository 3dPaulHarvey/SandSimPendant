#include "fluid_test_support.h"
#include "../Gravity_Sand/FluidVariation.h"
#include <float.h>

struct Random {
  uint32_t state;
  explicit Random(uint32_t seed) : state(seed) {}
  uint32_t operator()() {
    state ^= state << 13; state ^= state >> 17; state ^= state << 5;
    return state;
  }
};

struct FixedRandom {
  uint32_t value;
  int calls = 0;
  explicit FixedRandom(uint32_t bits) : value(bits) {}
  uint32_t operator()() { ++calls; return value; }
};

struct EndpointRandom {
  uint32_t first, next;
  bool used = false;
  EndpointRandom(uint32_t size, uint32_t coverage) : first(size), next(coverage) {}
  uint32_t operator()() { if (!used) { used = true; return first; } return next; }
};

static float nearestPair(const Simulation &simulation) {
  float smallest = 1000;
  for (int i = 0; i < simulation.particleCount(); ++i) {
    for (int j = i + 1; j < simulation.particleCount(); ++j) {
      const float dx = simulation.particle(i).x - simulation.particle(j).x;
      const float dy = simulation.particle(i).y - simulation.particle(j).y;
      const float distance = sqrtf(dx * dx + dy * dy);
      if (distance < smallest) smallest = distance;
    }
  }
  return smallest;
}

static void scaleEndpointsAndDenseConfigurations() {
  const float scales[] = {1, 1.01f, 1.5f, 2, 3, 4.99f, 5};
  assert(Simulation::maxCountForScale(1) == 120);
  assert(Simulation::maxCountForScale(2) == 30);
  assert(Simulation::maxCountForScale(5) == 5);
  assert(Simulation::maxCountForScale(NAN) == 0);
  assert(Simulation::maxCountForScale(0.99f) == 0);
  assert(Simulation::maxCountForScale(5.01f) == 0);
  for (float scale : scales) {
    for (uint32_t seed = 1; seed <= 5; ++seed) {
      Simulation simulation, repeat;
      const int count = Simulation::maxCountForScale(scale);
      assert(simulation.configure(scale, count, seed));
      assert(repeat.configure(scale, count, seed));
      equalFluid(simulation, repeat);
      checkFluid(simulation);
      assert(simulation.particleCount() == count);
      assert(simulation.dotRadius() == sand::config::kDotRadius * scale);
      assert(simulation.boundaryRadius() == Simulation::kCircleRadius - simulation.dotRadius());
      // A redraw starts from a valid, separated patch, even at the densest
      // legal population and maximum visible diameter.
      assert(nearestPair(simulation) > 2 * simulation.dotRadius());
      uint8_t colors[Simulation::kMaxParticleCount];
      for (int i = 0; i < count; ++i) colors[i] = simulation.particle(i).color;
      const float directions[4][2] = {{0, 1}, {-1, 0}, {0, -1}, {1, 0}};
      for (const auto &direction : directions) {
        simulation.explode(Simulation::kCenter, Simulation::kCenter, sand::config::kExplosionMaxImpulse);
        runFluid(simulation, direction[0], direction[1], 0.4f);
      }
      for (int i = 0; i < count; ++i) assert(simulation.particle(i).color == colors[i]);
      simulation.reset(seed);
      equalFluid(simulation, repeat); // Reset preserves the chosen size/count.
    }
  }
  puts("Sizes 1x-5x at maximum legal populations start without overlap and keep their complete radius inside every wall.");
}

static void populationRangeAndDistribution() {
  using namespace sand::variation;
  namespace cfg = sand::config;
  assert(scaleFromUnit(0) == 1 && scaleFromUnit(1) == 3.5f);
  assert(scaleFromUnit(0.25f) == 1.625f);
  assert(scaleFromUnit(0.5f) == 2.25f);
  assert(scaleFromUnit(0.75f) == 2.875f);
  assert(scaleFromUnit(-1) == 1 && scaleFromUnit(2) == 3.5f);
  assert(scaleFromUnit(NAN) == 1);
  assert(validVisualScale(1) && validVisualScale(2.25f) && validVisualScale(3.5f));
  assert(!validVisualScale(0.99f) && !validVisualScale(3.51f));
  assert(!validVisualScale(NAN) && !validVisualScale(INFINITY));
  const CountRange small = visualCountRange(1), large = visualCountRange(3.5f);
  assert(small.low == 120 && small.high == 120);
  assert(large.low == 16 && large.high == 120);
  assert(visualCountRange(NAN).high == 0 && visualCountRange(0).high == 0);
  // One lower-biased band replaces the crowded tail. Its endpoints leave
  // room for approximate fitting and antialiasing inside 15-35% visible fill.
  assert(cfg::kMinSpawnCoverage >= 0.15f && cfg::kMaxSpawnCoverage <= 0.35f);
  assert(coverageFromUnit(0) == cfg::kMinSpawnCoverage);
  assert(coverageFromUnit(-1) == cfg::kMinSpawnCoverage);
  assert(coverageFromUnit(NAN) == cfg::kMinSpawnCoverage);
  assert(fabsf(coverageFromUnit(1) - cfg::kMaxSpawnCoverage) < 0.00001f);
  assert(coverageFromUnit(2) == coverageFromUnit(1));
  assert(fabsf(coverageFromUnit(0.5f) -
      (cfg::kMinSpawnCoverage + 0.25f * (cfg::kMaxSpawnCoverage - cfg::kMinSpawnCoverage))) < 0.00001f);
  assert(fabsf(coverageFromUnit(0.98f) -
      (cfg::kMinSpawnCoverage + 0.98f * 0.98f * (cfg::kMaxSpawnCoverage - cfg::kMinSpawnCoverage))) < 0.00001f);
  EndpointRandom smallOpen(0, 0), largeOpen(UINT32_MAX, 0);
  EndpointRandom smallFull(0, UINT32_MAX), largeFull(UINT32_MAX, UINT32_MAX);
  const Population endpoints[] = {samplePopulation(smallOpen), samplePopulation(largeOpen),
                                  samplePopulation(smallFull), samplePopulation(largeFull)};
  assert(endpoints[0].scale == 1 && endpoints[1].scale == 3.5f);
  assert(endpoints[2].scale == 1 && endpoints[3].scale == 3.5f);
  assert(endpoints[0].targetCoverage == cfg::kMinSpawnCoverage &&
         endpoints[1].targetCoverage == cfg::kMinSpawnCoverage);
  assert(fabsf(endpoints[2].targetCoverage - cfg::kMaxSpawnCoverage) < 0.00001f &&
         fabsf(endpoints[3].targetCoverage - cfg::kMaxSpawnCoverage) < 0.00001f);

  // Equal-width thirds must be equally likely, with half of rolls in the
  // upper half of the range. Coverage fitting may adjust the final size.
  Random random(0x302ab47f);
  const int samples = 1000000;
  int bins[3] = {};
  int fillBins[4] = {};
  int upperHalf = 0;
  double sumScale = 0, sumCoverage = 0;
  for (int i = 0; i < samples; ++i) {
    const Population population = samplePopulation(random);
    assert(validVisualScale(population.scale));
    const float target = population.targetCoverage;
    assert(isfinite(target));
    assert(target >= cfg::kMinSpawnCoverage && target <= cfg::kMaxSpawnCoverage);
    const float diameter = sand::config::kDotRadius * 2 *
                           sand::config::kVisualSizeScale * population.scale;
    assert(diameter >= 10 && diameter <= 35);
    const int bin = population.scale < 11.0f / 6 ? 0 :
                    (population.scale < 8.0f / 3 ? 1 : 2);
    ++bins[bin];
    ++fillBins[target < 0.20f ? 0 : target < 0.25f ? 1 : target < 0.30f ? 2 : 3];
    if (population.scale >= 2.25f) ++upperHalf;
    sumScale += population.scale;
    sumCoverage += target;
  }
  const double expectedSizeBins[] = {1.0 / 3, 1.0 / 3, 1.0 / 3};
  for (int bin = 0; bin < 3; ++bin)
    assert(fabs(bins[bin] / double(samples) - expectedSizeBins[bin]) < 0.002);
  assert(fabs(upperHalf / double(samples) - 0.5) < 0.002);
  assert(fabs(sumScale / samples - 2.25) < 0.01);
  const double below20 = sqrt((0.20 - cfg::kMinSpawnCoverage) /
                             (cfg::kMaxSpawnCoverage - cfg::kMinSpawnCoverage));
  const double below25 = sqrt((0.25 - cfg::kMinSpawnCoverage) /
                             (cfg::kMaxSpawnCoverage - cfg::kMinSpawnCoverage));
  const double below30 = sqrt((0.30 - cfg::kMinSpawnCoverage) /
                             (cfg::kMaxSpawnCoverage - cfg::kMinSpawnCoverage));
  const double expectedFillBins[] = {below20, below25 - below20, below30 - below25, 1 - below30};
  for (int bin = 0; bin < 4; ++bin)
    assert(fabs(fillBins[bin] / double(samples) - expectedFillBins[bin]) < 0.002);
  const double expectedMean = cfg::kMinSpawnCoverage +
      (cfg::kMaxSpawnCoverage - cfg::kMinSpawnCoverage) / 3.0;
  assert(fabs(sumCoverage / samples - expectedMean) < 0.0005);
  printf("1,000,000 preferred size rolls: thirds %.2f%% / %.2f%% / %.2f%%, mean %.3fx; %.2f%% above 225%% before coverage fitting.\n",
         bins[0] * 100.0 / samples, bins[1] * 100.0 / samples,
         bins[2] * 100.0 / samples, sumScale / samples, upperHalf * 100.0 / samples);
  printf("Coverage targets: 15-20%% %.3f%%, 20-25%% %.3f%%, 25-30%% %.3f%%, 30-35%% %.3f%%; mean %.3f%% (equation %.3f%%).\n",
         fillBins[0] * 100.0 / samples, fillBins[1] * 100.0 / samples,
         fillBins[2] * 100.0 / samples, fillBins[3] * 100.0 / samples,
         sumCoverage / samples * 100, expectedMean * 100);
}

static void randomizedStretchCaps() {
  Random random(0x71ceab29u);
  const int samples = 100000;
  int bins[4] = {};
  double sum = 0;
  for (int i = 0; i < samples; ++i) {
    const float cap = sand::variation::sampleStretch(random);
    assert(cap >= 1 && cap <= 3);
    int bin = static_cast<int>((cap - 1) * 2);
    if (bin > 3) bin = 3;
    ++bins[bin];
    sum += cap;
  }
  for (int count : bins) assert(fabs(count / double(samples) - 0.25) < 0.01);
  assert(fabs(sum / samples - 2) < 0.01);
  FixedRandom low(0), high(UINT32_MAX);
  assert(sand::variation::sampleStretch(low) == 1);
  assert(sand::variation::sampleStretch(high) == 3);
  assert(low.calls == 1 && high.calls == 1);
  puts("Stretch caps span 1x-3x uniformly with one random draw per population.");
}

static void explosionStrengthAndBoundedRandomFailure() {
  using namespace sand::variation;
  Random random(0x8cf43e27);
  const int samples = 100000;
  double sumStrength = 0;
  float lowStrength = FLT_MAX, highStrength = 0;
  int bins[4] = {};
  for (int i = 0; i < samples; ++i) {
    const float strength = sampleExplosionStrength(random);
    assert(strength >= 600 && strength <= 1200);
    int bin = static_cast<int>((strength - 600) / 150);
    if (bin > 3) bin = 3;
    ++bins[bin];
    sumStrength += strength;
    if (strength < lowStrength) lowStrength = strength;
    if (strength > highStrength) highStrength = strength;
  }
  assert(fabs(sumStrength / samples - 900) < 3);
  assert(lowStrength < 601 && highStrength > 1199);
  for (int count : bins) assert(fabs(count / double(samples) - 0.25) < 0.01);

  FixedRandom stuckLow(0), stuckHigh(UINT32_MAX);
  const float low = sampleExplosionStrength(stuckLow), high = sampleExplosionStrength(stuckHigh);
  assert(stuckLow.calls == 1 && stuckHigh.calls == 1);
  assert(low == 600 && high == 1200);
  FixedRandom stuckPopulation(0);
  const Population population = samplePopulation(stuckPopulation);
  assert(stuckPopulation.calls == 2 && population.scale == 1 &&
         population.targetCoverage == sand::config::kMinSpawnCoverage);
  FixedRandom stuckLargePopulation(UINT32_MAX);
  const Population largePopulation = samplePopulation(stuckLargePopulation);
  assert(stuckLargePopulation.calls == 2 && largePopulation.scale == 3.5f);
  assert(fabsf(largePopulation.targetCoverage - sand::config::kMaxSpawnCoverage) < 0.00001f);
  FixedRandom stuckBounded(0);
  assert(bounded(stuckBounded, 73) == 0 && stuckBounded.calls == 64);
  puts("100,000 explosion strengths span 600-1200px/s uniformly (100-200%); stuck random sources terminate safely.");
}

static void everyRollHasSingleOddShape() {
  using namespace sand::variation;
  using sand::render::Shape;
  const int count = 37, samples = 100000;
  Random random(0x35d4c67bu);
  int totalOdd = 0;
  for (int main = 0; main < static_cast<int>(Shape::Count); ++main) {
    const Shape majority = static_cast<Shape>(main);
    int selected = 0, alternatives[static_cast<int>(Shape::Count)] = {}, slots[count] = {};
    for (int sample = 0; sample < samples; ++sample) {
      const OddShape odd = sampleOddShape(random, majority);
      const int index = odd.index(count);
      int exceptions = 0;
      for (int i = 0; i < count; ++i) exceptions += odd.shapeAt(i, count, majority) != majority;
      assert(exceptions == 1);
      assert(index >= 0 && index < count && odd.shape != majority);
      assert(odd.index(count) == index); // Stable across repeated snapshots.
      ++selected;
      ++alternatives[static_cast<int>(odd.shape)];
      ++slots[index];
    }
    assert(selected == samples);
    assert(alternatives[main] == 0);
    for (int other = 0; other < static_cast<int>(Shape::Count); ++other)
      if (other != main) assert(fabs(alternatives[other] / double(selected) - 1.0 / 3) < 0.015);
    for (int hits : slots) assert(fabs(hits / double(selected) - 1.0 / count) < 0.006);
    totalOdd += selected;
  }
  assert(OddShape().index(count) == -1);
  assert(OddShape(static_cast<Shape>(255), UINT32_MAX).index(count) == -1);
  const OddShape first(Shape::Star, 0), last(Shape::Triangle, UINT32_MAX);
  for (int size = 2; size <= Simulation::kMaxParticleCount; ++size) {
    assert(first.index(size) == 0 && last.index(size) == size - 1);
  }
  assert(first.index(-1) == -1 && first.index(0) == -1 && first.index(1) == -1);
  assert(first.index(Simulation::kMaxParticleCount + 1) == -1);
  assert(first.shapeAt(-1, count, Shape::Circle) == Shape::Circle);
  assert(OddShape().shapeAt(0, count, static_cast<Shape>(255)) == Shape::Circle);
  const uint32_t fixedBits[] = {0, 1, UINT32_MAX};
  for (int main = 0; main < static_cast<int>(Shape::Count); ++main) {
    const Shape majority = static_cast<Shape>(main);
    for (uint32_t bits : fixedBits) {
      FixedRandom fixed(bits);
      const OddShape odd = sampleOddShape(fixed, majority);
      assert(sand::render::validShape(odd.shape) && odd.shape != majority);
      assert(odd.index(count) >= 0 && odd.index(count) < count);
      assert(fixed.calls >= 2 && fixed.calls <= 65);
    }
  }
  printf("400,000 odd-shape rolls: %.3f%% contain exactly one different shape; alternate shapes and stable slots are uniform.\n",
         totalOdd * 100.0 / (4 * samples));
}

int main() {
  scaleEndpointsAndDenseConfigurations();
  populationRangeAndDistribution();
  randomizedStretchCaps();
  explosionStrengthAndBoundedRandomFailure();
  everyRollHasSingleOddShape();
}
