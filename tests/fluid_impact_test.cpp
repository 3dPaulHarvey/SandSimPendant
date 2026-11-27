#include "fluid_test_support.h"
#include <float.h>

using sand::impact::Pulse;

static void pulseTimingAndBounds() {
  Pulse pulse;
  assert(!pulse.active() && pulse.scale() == 1 && pulse.weight() == 0);
  pulse.contact(400, 1, 0);
  assert(pulse.active() && pulse.normalX == 1 && pulse.normalY == 0);
  assert(pulse.scale() == 1 && pulse.weight() == 0);
  pulse.advance(Pulse::kAttack);
  assert(fabsf(pulse.scale() - 0.65f) < 0.00001f && pulse.weight() == 1);
  pulse.advance(Pulse::kRelease - Pulse::kAttack);
  assert(fabsf(pulse.scale() - 1) < 0.00001f);
  pulse.advance(Pulse::kRebound - Pulse::kRelease);
  assert(fabsf(pulse.scale() - 1.10f) < 0.00001f);
  assert(pulse.weight() > 0 && pulse.weight() < 1);
  pulse.advance(Pulse::kDuration);
  assert(!pulse.active() && pulse.scale() == 1 && pulse.weight() == 0);

  const float speeds[] = {81, 120, 240, 400, 650, FLT_MAX};
  float previousPeak = 1;
  for (float speed : speeds) {
    pulse.reset();
    pulse.contact(speed, 0, -1);
    pulse.advance(Pulse::kAttack);
    assert(pulse.scale() <= previousPeak);
    previousPeak = pulse.scale();
    for (int tick = 0; tick < 300; ++tick) {
      assert(isfinite(pulse.scale()) && pulse.scale() >= 0.65f && pulse.scale() <= 1.10f);
      assert(isfinite(pulse.weight()) && pulse.weight() >= 0 && pulse.weight() <= 1);
      pulse.advance(0.001f);
    }
    assert(!pulse.active() && pulse.scale() == 1 && pulse.weight() == 0);
  }
  puts("Wall pulses compress by25ms, rebound, settle within240ms, and remain bounded for all impact speeds.");
}

static void contactsDoNotChatter() {
  Pulse pulse;
  pulse.contact(400, 1, 0);
  for (int tick = 0; tick < 1000; ++tick) {
    pulse.advance(0.001f);
    pulse.contact(650, 1, 0); // A sustained contact must never restart the pulse.
    if (tick >= 240) assert(!pulse.active());
  }
  pulse.leaveWall();
  pulse.contact(400, 0, 1);
  assert(pulse.active() && pulse.normalX == 0 && pulse.normalY == 1);
  pulse.advance(0.04f);
  const float previousScale = pulse.scale(), previousWeight = pulse.weight();
  pulse.leaveWall();
  pulse.contact(650, -1, 0); // A second nearby collision cannot interrupt recovery.
  assert(pulse.normalX == 0 && pulse.normalY == 1);
  assert(pulse.scale() == previousScale && pulse.weight() == previousWeight);
  pulse.advance(Pulse::kDuration);
  pulse.contact(650, -1, 0);
  assert(!pulse.active()); // Re-arm requires leaving after that ignored contact.
  pulse.leaveWall();
  pulse.contact(650, -1, 0);
  assert(pulse.active());

  const float quiet[] = {-400, 0, 40, 80, NAN, INFINITY};
  for (float speed : quiet) {
    pulse.reset();
    pulse.contact(speed, 1, 0);
    assert(!pulse.active());
  }
  const float invalidNormals[][2] = {{0, 0}, {2, 0}, {NAN, 1}, {0, INFINITY}, {FLT_MAX, FLT_MAX}};
  for (const auto &normal : invalidNormals) {
    pulse.reset();
    pulse.contact(400, normal[0], normal[1]);
    assert(!pulse.active());
  }
  pulse.reset();
  pulse.contact(400, 1, 0);
  pulse.advance(0.02f);
  const float scale = pulse.scale(), weight = pulse.weight();
  const float invalidSeconds[] = {NAN, INFINITY, -1.0f, 0.0f};
  for (float invalid : invalidSeconds) pulse.advance(invalid);
  assert(pulse.scale() == scale && pulse.weight() == weight);
  pulse.reset();
  assert(!pulse.active() && pulse.normalX == 0 && pulse.normalY == 0);
  puts("Quiet/tangential-speed contacts, sustained contact, rapid re-hits, reset, and invalid inputs cannot chatter.");
}

static void setParticle(Simulation &simulation, int index, float x, float y, float vx, float vy) {
  // Place independent bodies to test real wall collision handling without
  // relying on the random spawn rotation or pressure from a neighbouring body.
  sand::Particle &particle = const_cast<sand::Particle &>(simulation.particle(index));
  particle = {x, y, vx, vy, 0};
}

static void collisionIntegration() {
  Simulation simulation;
  assert(simulation.configure(1, 3));
  const float center = Simulation::kCenter, boundary = simulation.boundaryRadius();
  setParticle(simulation, 0, center + boundary - 0.5f, center, 400, 0);
  setParticle(simulation, 1, center - 40, center - 40, 0, 0);
  setParticle(simulation, 2, center - 40, center + 40, 0, 0);
  simulation.step(0, 0, fluidDt());
  assert(simulation.impact(0).active());
  assert(simulation.impact(0).normalX > 0.999f && fabsf(simulation.impact(0).normalY) < 0.001f);
  assert(simulation.particle(0).vx < 0); // The original round-body rebound still occurs.
  assert(!simulation.impact(1).active() && !simulation.impact(2).active());
  bool compressed = false, rebounded = false;
  for (int tick = 0; tick < 70; ++tick) {
    simulation.step(0, 0, fluidDt());
    compressed |= simulation.impact(0).scale() < 0.7f;
    rebounded |= simulation.impact(0).scale() > 1.05f;
    checkFluid(simulation);
  }
  assert(compressed && rebounded && !simulation.impact(0).active());

  for (int mode = 0; mode < 2; ++mode) {
    simulation.reset();
    setParticle(simulation, 0, center + boundary, center, mode == 0 ? 40 : 0, mode == 1 ? 400 : 0);
    setParticle(simulation, 1, center - 40, center - 40, 0, 0);
    setParticle(simulation, 2, center - 40, center + 40, 0, 0);
    simulation.step(0, 0, fluidDt());
    assert(!simulation.impact(0).active());
  }
  simulation.reset();
  setParticle(simulation, 0, center, center - boundary + 0.5f, 0, -400);
  setParticle(simulation, 1, center - 40, center + 40, 0, 0);
  setParticle(simulation, 2, center + 40, center + 40, 0, 0);
  simulation.step(0, 0, fluidDt());
  assert(simulation.impact(0).active() && simulation.impact(0).normalY < -0.999f);
  simulation.reset(7);
  for (int index = 0; index < simulation.particleCount(); ++index)
    assert(!simulation.impact(index).active());
  assert(simulation.configurePacked(3.5f, 8, 9));
  for (int index = 0; index < simulation.particleCount(); ++index)
    assert(!simulation.impact(index).active());
  puts("Actual round-wall impacts start aligned pulses; tangential/quiet contacts do not, and resets clear them.");
}

int main() {
  pulseTimingAndBounds();
  contactsDoNotChatter();
  collisionIntegration();
}
