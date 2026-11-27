#include "../shared/MotionHealth.h"
#include <assert.h>
#include <stdio.h>

static void exercise(uint32_t start) {
  motion::MotionHealth health;
  uint32_t now = start;
  assert(health.update(now, 0, false, false, false));
  assert(!health.trusted(now));
  for (int i = 1; i <= 76; ++i) {
    now = start + i * 10000U;
    assert(!health.update(now, 0, true, true, true));
    assert(health.trusted(now) == (i == 76));
  }
  // One transport/packet failure immediately blocks amplification and clears
  // the old transient. Fresh valid samples must rebuild a full clean window.
  now += 10000;
  assert(health.update(now, 1, true, true, false));
  const uint32_t failedAt = now;
  assert(!health.trusted(now));
  for (int i = 1; i <= 76; ++i) {
    now = failedAt + i * 10000U;
    assert(!health.update(now, 1, true, true, true));
    assert(health.trusted(now) == (i == 76));
  }
  // Even without a bus error, an input gap must not resume stale shake forces.
  now += 50001;
  assert(!health.trusted(now));
  assert(health.update(now, 1, true, true, true));
  assert(!health.trusted(now));
  now += 10000;
  assert(health.update(now, 1, true, false, true));
  assert(!health.trusted(now));
  now += 1000000;
  health.update(now, 1, true, true, true);
  assert(!health.trusted(now)); // A silent outage cannot count as recovery.
  for (int i = 0; i < 200; ++i) {
    now += 10000;
    health.update(now, 2 + i / 10, true, true, true);
    assert(!health.trusted(now)); // A faulting stream cannot build impulse energy.
  }
}

int main() {
  motion::MotionHealth lateStart;
  assert(lateStart.update(10000000, 0, true, true, true));
  assert(!lateStart.trusted(10000000)); // Even an already-online sensor needs observation.
  exercise(100000);
  exercise(UINT32_MAX - 400000U); // All deadlines must survive micros() wrap.
  puts("Motion health: warmup, fault recovery, stale input, repeated errors and clock wrap passed.");
}
