#include "../shared/ShakeInput.h"
#include "../Gravity_Sand/SandSimulation.h"

#include <assert.h>
#include <stdio.h>

using motion::Vec3;
using sand::Simulation;

static void check(const Simulation &sim) {
  bool seen[Simulation::kParticleCount] = {};
  int count = 0;
  for (int row = 0; row < Simulation::kGridSize; ++row) {
    for (int col = 0; col < Simulation::kGridSize; ++col) {
      const uint16_t id = sim.particleAt(col, row);
      if (!Simulation::inside(col, row)) assert(id == Simulation::kWall);
      else if (id != Simulation::kEmpty) {
        assert(id < Simulation::kParticleCount && !seen[id]);
        seen[id] = true;
        ++count;
        const sand::Particle &p = sim.particle(id);
        assert(p.cell == row * Simulation::kGridSize + col);
        assert(static_cast<int>(floorf(p.x)) == col);
        assert(static_cast<int>(floorf(p.y)) == row);
        assert(isfinite(p.x) && isfinite(p.y) && isfinite(p.vx) && isfinite(p.vy));
        assert(p.vx * p.vx + p.vy * p.vy < 90.01f * 90.01f);
      }
    }
  }
  assert(count == Simulation::kParticleCount);
}

struct Harness {
  motion::ShakeInput input;
  Simulation simulation;
  double physicsAccumulator;
  float maxAgitation, agitationSum;
  float gain;
  bool directionalBoost;
  int activeFrames, physicsFrames;

  explicit Harness(float shakeGain = 1.75f, bool boostXY = true)
      : physicsAccumulator(0), gain(shakeGain), directionalBoost(boostXY) { clearStats(); }

  void clearStats() {
    maxAgitation = agitationSum = 0;
    activeFrames = physicsFrames = 0;
  }

  void sample(Vec3 acceleration, Vec3 gyroDps, float dt) {
    input.update(acceleration, gyroDps, dt);
    physicsAccumulator += dt;
    while (physicsAccumulator + 1e-9 >= 1.0 / 60.0) {
      const motion::ShakePulse pulse = input.consume();
      assert(isfinite(pulse.agitation) && pulse.agitation >= 0 && pulse.agitation <= 18.001f);
      if (pulse.agitation > maxAgitation) maxAgitation = pulse.agitation;
      agitationSum += pulse.agitation;
      if (pulse.agitation > 0.001f) ++activeFrames;
      ++physicsFrames;
      // Production contract: a 4x extra dynamic XY shove, default gain1.75,
      // plus stronger agitation; signed impulses already include sample dt.
      const motion::Vec2 shove = motion::mapGravity(pulse.linearImpulseGSeconds);
      simulation.kick(directionalBoost ? shove.x * 240 * 4 * gain : 0,
                      directionalBoost ? shove.y * 240 * 4 * gain : 0,
                      pulse.agitation * gain);
      const motion::Vec2 gravity = motion::mapGravity(acceleration);
      simulation.step(gravity.x, gravity.y, 1.0f / 60.0f);
      check(simulation);
      physicsAccumulator -= 1.0 / 60.0;
    }
  }

  void rest(Vec3 acceleration, int sampleRate, float seconds) {
    const int count = static_cast<int>(sampleRate * seconds + 0.5f);
    for (int i = 0; i < count; ++i) sample(acceleration, Vec3(), 1.0f / sampleRate);
  }
};

static void flatRestHasNoShake(int sampleRate) {
  Harness harness;
  Simulation original = harness.simulation;
  harness.rest(Vec3(0, 0, 1), sampleRate, 1.0f);
  assert(harness.activeFrames == 0 && harness.maxAgitation == 0);
  for (int i = 0; i < Simulation::kParticleCount; ++i) {
    assert(harness.simulation.particle(i).cell == original.particle(i).cell);
    assert(harness.simulation.particle(i).vx == 0 && harness.simulation.particle(i).vy == 0);
  }
}

static void normalShakeMovesPile(int sampleRate) {
  Harness harness;
  for (int i = 0; i < 600; ++i) harness.simulation.step(0, 1, 1.0f / 60);
  harness.rest(Vec3(0, 1, 0), sampleRate, 0.8f);
  assert(harness.maxAgitation == 0);
  const Simulation before = harness.simulation;
  Simulation baseline = before;
  Harness originalGain(1.0f, false);
  originalGain.simulation = harness.simulation;
  originalGain.input = harness.input;
  originalGain.physicsAccumulator = harness.physicsAccumulator;
  harness.clearStats();
  int mostChanged = 0, oldMostChanged = 0, baselineFrames = 0;
  const float pi = 3.14159265358979323846f;
  for (int i = 1; i <= sampleRate; ++i) {
    const float seconds = static_cast<float>(i) / sampleRate;
    // Upright screen: gravity stays +Y; +/-Z is translation normal to the
    // display, so this must exercise shake rather than ordinary tilt flow.
    const Vec3 acceleration(0, 1, 0.6f * sinf(2 * pi * 7 * seconds));
    harness.sample(acceleration, Vec3(), 1.0f / sampleRate);
    originalGain.sample(acceleration, Vec3(), 1.0f / sampleRate);
    while (baselineFrames < harness.physicsFrames) {
      baseline.step(0, 1, 1.0f / 60);
      ++baselineFrames;
    }
    int changed = 0, oldChanged = 0;
    for (int grain = 0; grain < Simulation::kParticleCount; ++grain) {
      if (harness.simulation.particle(grain).cell != baseline.particle(grain).cell) ++changed;
      if (originalGain.simulation.particle(grain).cell != baseline.particle(grain).cell) ++oldChanged;
    }
    if (changed > mostChanged) mostChanged = changed;
    if (oldChanged > oldMostChanged) oldMostChanged = oldChanged;
  }
  assert(harness.activeFrames >= 20 && harness.maxAgitation > 1);
  assert(mostChanged >= 25);
  assert(mostChanged > oldMostChanged * 1.25f && mostChanged >= oldMostChanged + 20);
  for (int grain = 0; grain < Simulation::kParticleCount; ++grain)
    assert(baseline.particle(grain).cell == before.particle(grain).cell);
  printf("%dHz Z shake: moved grains gain1=%d, gain1.75=%d; input peak=%.2f\n",
         sampleRate, oldMostChanged, mostChanged, harness.maxAgitation);
  harness.rest(Vec3(0, 1, 0), sampleRate, 0.5f);
  harness.clearStats();
  harness.rest(Vec3(0, 1, 0), sampleRate, 0.2f);
  assert(harness.maxAgitation < 0.01f && harness.activeFrames == 0);
}

static float travel(const Simulation &simulation, uint16_t *previous) {
  float distance = 0;
  for (int i = 0; i < Simulation::kParticleCount; ++i) {
    const int before = previous[i], after = simulation.particle(i).cell;
    int dx = after % Simulation::kGridSize - before % Simulation::kGridSize;
    int dy = after / Simulation::kGridSize - before / Simulation::kGridSize;
    if (dx < 0) dx = -dx;
    if (dy < 0) dy = -dy;
    distance += dx + dy;
    previous[i] = static_cast<uint16_t>(after);
  }
  return distance / Simulation::kParticleCount;
}

static float centerX(const Simulation &simulation) {
  float sum = 0;
  for (int i = 0; i < Simulation::kParticleCount; ++i) sum += simulation.particle(i).x;
  return sum / Simulation::kParticleCount;
}

static void lateralShakeIsStronger(int sampleRate) {
  Harness enhanced;
  for (int i = 0; i < 600; ++i) enhanced.simulation.step(0, 1, 1.0f / 60);
  enhanced.rest(Vec3(0, 1, 0), sampleRate, 0.8f);
  enhanced.clearStats();
  Simulation forceOnly = enhanced.simulation;
  uint16_t previousEnhanced[Simulation::kParticleCount], previousOld[Simulation::kParticleCount];
  for (int i = 0; i < Simulation::kParticleCount; ++i)
    previousEnhanced[i] = previousOld[i] = forceOnly.particle(i).cell;
  float enhancedTravel = 0, oldTravel = 0;
  const float initialCenter = centerX(forceOnly);
  float enhancedExcursion = 0, oldExcursion = 0;
  int oldFrames = 0;
  const float pi = 3.14159265358979323846f;
  for (int i = 1; i <= sampleRate; ++i) {
    const float seconds = static_cast<float>(i) / sampleRate;
    const Vec3 acceleration(0.5f * sinf(2 * pi * 5 * seconds), 1, 0);
    enhanced.sample(acceleration, Vec3(), 1.0f / sampleRate);
    while (oldFrames < enhanced.physicsFrames) {
      const motion::Vec2 gravity = motion::mapGravity(acceleration);
      forceOnly.step(gravity.x, gravity.y, 1.0f / 60);
      check(forceOnly);
      enhancedTravel += travel(enhanced.simulation, previousEnhanced);
      oldTravel += travel(forceOnly, previousOld);
      const float currentEnhanced = fabsf(centerX(enhanced.simulation) - initialCenter);
      const float currentOld = fabsf(centerX(forceOnly) - initialCenter);
      if (currentEnhanced > enhancedExcursion) enhancedExcursion = currentEnhanced;
      if (currentOld > oldExcursion) oldExcursion = currentOld;
      ++oldFrames;
    }
  }
  printf("%dHz .5g X shake5Hz: mean path force-only=%.3f, enhanced=%.3f cells; centroid=%.3f/%.3f\n",
         sampleRate, oldTravel, enhancedTravel, oldExcursion, enhancedExcursion);
  assert(enhancedTravel > oldTravel * 1.25f && enhancedTravel > oldTravel + 0.05f);
  enhanced.rest(Vec3(0, 1, 0), sampleRate, 0.5f);
  enhanced.clearStats();
  enhanced.rest(Vec3(0, 1, 0), sampleRate, 0.2f);
  assert(enhanced.activeFrames == 0);
}

static void slowTiltDoesNotShake(int sampleRate) {
  Harness harness;
  harness.rest(Vec3(0, 0, 1), sampleRate, 0.8f);
  harness.clearStats();
  const float pi = 3.14159265358979323846f;
  for (int i = 1; i <= sampleRate * 3; ++i) {
    const float seconds = static_cast<float>(i) / sampleRate;
    const float angle = seconds * pi / 6; // 30deg/sec, ending at90deg.
    // Body rotates about +X: stationary support force in body coordinates is
    // (0,sin(theta),cos(theta)), and calibrated gyro reports +30deg/sec.
    harness.sample(Vec3(0, sinf(angle), cosf(angle)), Vec3(30, 0, 0), 1.0f / sampleRate);
  }
  printf("%dHz slow90deg tilt: peak false agitation=%.6f, active frames=%d\n",
         sampleRate, harness.maxAgitation, harness.activeFrames);
  assert(harness.maxAgitation < 0.01f && harness.activeFrames == 0);
  harness.rest(Vec3(0, 1, 0), sampleRate, 0.5f);
}

int main() {
  flatRestHasNoShake(180);
  flatRestHasNoShake(200);
  normalShakeMovesPile(180);
  normalShakeMovesPile(200);
  lateralShakeIsStronger(180);
  lateralShakeIsStronger(200);
  slowTiltDoesNotShake(180);
  slowTiltDoesNotShake(200);
  puts("Shake pipeline: quiet rest, normal-Z motion, stopped-motion decay, pure tilt rejection,");
  puts("60Hz consumption, conservation, and circular containment passed at180/200Hz input.");
}
