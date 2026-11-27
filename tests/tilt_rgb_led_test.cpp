#include <stdint.h>

void rgbLedWrite(uint8_t pin, uint8_t red, uint8_t green, uint8_t blue);
#include "../shared/TiltRgbLed.h"

#include <assert.h>
#include <stdio.h>

struct LedWrite {
  uint8_t pin;
  motion::TiltRgb color;
};

static uint32_t clockUs = 0, writeCostUs = 10;
static LedWrite packets[32];
static unsigned packetCount = 0;

uint32_t micros() { return clockUs; }
uint32_t millis() { return clockUs / 1000; }
void delay(uint32_t milliseconds) { clockUs += milliseconds * 1000U; }
void rgbLedWrite(uint8_t pin, uint8_t red, uint8_t green, uint8_t blue) {
  assert(packetCount < sizeof(packets) / sizeof(packets[0]));
  packets[packetCount++] = {pin, motion::TiltRgb(red, green, blue)};
  clockUs += writeCostUs;
}

static void update(motion::TiltRgbLed &led, uint32_t now, float x, float y,
                   bool ready, uint32_t sample) {
  clockUs = now;
  led.update(now, x, y, ready, sample);
  if (packetCount) {
    assert(packets[packetCount - 1].pin == hardware::kRgbLed);
    assert(packets[packetCount - 1].color == led.color());
  }
}

static void readinessAndFreshness(uint32_t start) {
  packetCount = 0;
  clockUs = start;
  writeCostUs = 10;
  motion::TiltRgbLed led;
  led.begin();
  assert(packetCount == 1);
  assert(packets[0].pin == hardware::kRgbLed);
  assert(packets[0].color == motion::TiltRgb());
  assert(led.writes == 0); // Startup allocation/write is outside runtime stats.

  update(led, start + 100, 1, 0, false, start);
  assert(packetCount == 1); // No usable motion yet: keep the LED dark.
  update(led, start + 200, 0, -1, true, start + 200);
  assert(packetCount == 2);
  assert(led.color() == motion::TiltRgb(0, 0, 32));
  update(led, start + 40000, 1, 0, true, start + 40000);
  assert(packetCount == 2); // Still within 40ms of the first usable sample.
  update(led, start + 40200, 1, 0, true, start + 40200);
  assert(packetCount == 3);
  assert(led.color() == motion::TiltRgb(0, 32, 0));
  update(led, start + 80200, 1, 0, true, start + 80200);
  assert(packetCount == 3); // Unchanged color suppresses an RMT packet.

  update(led, start + 80201, 1, 0, false, start + 80201);
  assert(packetCount == 4);
  assert(led.color() == motion::TiltRgb());
  update(led, start + 80202, 1, 0, false, start + 80202);
  assert(packetCount == 4);

  // A sample exactly at the freshness limit remains usable. One microsecond
  // later it must darken immediately, even though a normal LED slot is not due.
  const uint32_t boundary = start + motion::TiltRgbLed::kFreshnessUs;
  update(led, boundary, -1, 0, true, start);
  assert(packetCount == 5);
  assert(led.color() == motion::TiltRgb(32, 0, 0));
  update(led, boundary + 1, -1, 0, true, start);
  assert(packetCount == 6);
  assert(led.color() == motion::TiltRgb());
  update(led, boundary + 2, -1, 0, true, start);
  assert(packetCount == 6);
  update(led, boundary + 40000, 0, 1, true, boundary + 40000);
  assert(packetCount == 7);
  assert(led.color() == motion::TiltRgb(32, 32, 0));
  assert(led.writes == packetCount - 1);
  assert(led.totalWriteUs == led.writes * writeCostUs);
  assert(led.maxWriteUs == writeCostUs);
}

static void timingStatistics() {
  packetCount = 0;
  clockUs = 0;
  motion::TiltRgbLed led;
  led.begin();
  writeCostUs = 7;
  update(led, 100, 0, -1, true, 100);
  writeCostUs = 13;
  update(led, 40100, 1, 0, true, 40100);
  assert(led.writes == 2 && led.totalWriteUs == 20 && led.maxWriteUs == 13);
  led.resetStats();
  assert(led.writes == 0 && led.totalWriteUs == 0 && led.maxWriteUs == 0);
  assert(led.color() == motion::TiltRgb(0, 32, 0));
  update(led, 80100, 1, 0, true, 80100);
  assert(led.writes == 0 && packetCount == 3);
  // Duration arithmetic also survives micros() wrapping during an LED write.
  writeCostUs = 10;
  update(led, UINT32_MAX - 4, -1, 0, true, UINT32_MAX - 4);
  assert(led.writes == 1 && led.totalWriteUs == 10 && led.maxWriteUs == 10);
}

int main() {
  readinessAndFreshness(1000000);
  readinessAndFreshness(UINT32_MAX - 50000U);
  timingStatistics();
  puts("Tilt RGB hardware adapter: startup off, readiness/freshness gates, immediate stale-off, deduplicated writes and wrap-safe write timings passed.");
}
