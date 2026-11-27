#include "../shared/TiltColorWheel.h"

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>

using motion::TiltColorWheel;
using motion::TiltRgb;

static int peak(TiltRgb color) {
  int value = color.r > color.g ? color.r : color.g;
  return value > color.b ? value : color.b;
}

static int difference(TiltRgb a, TiltRgb b) {
  const int red = static_cast<int>(a.r) - b.r;
  const int green = static_cast<int>(a.g) - b.g;
  const int blue = static_cast<int>(a.b) - b.b;
  int maximum = red < 0 ? -red : red;
  const int g = green < 0 ? -green : green;
  const int bValue = blue < 0 ? -blue : blue;
  if (g > maximum) maximum = g;
  return bValue > maximum ? bValue : maximum;
}

static void directionalAnchors() {
  struct Sample { float x, y; TiltRgb expected; };
  const Sample samples[] = {
    {0, -1, TiltRgb(0, 0, 32)},
    {1, -1, TiltRgb(0, 32, 32)},
    {1, 0, TiltRgb(0, 32, 0)},
    {1, 1, TiltRgb(16, 32, 0)},
    {0, 1, TiltRgb(32, 32, 0)},
    {-1, 1, TiltRgb(32, 16, 0)},
    {-1, 0, TiltRgb(32, 0, 0)},
    {-1, -1, TiltRgb(32, 0, 32)}
  };
  for (const Sample &sample : samples)
    assert(TiltColorWheel::colorFor(sample.x, sample.y) == sample.expected);

  // A full turn must not jump at quadrant or diagonal boundaries. This tests
  // visible continuity, without prescribing the angle approximation itself.
  const float radians = 0.017453292519943295f;
  TiltRgb previous = TiltColorWheel::colorFor(1, 0);
  for (int degree = 1; degree <= 360; ++degree) {
    const TiltRgb current = TiltColorWheel::colorFor(
      cosf(degree * radians), sinf(degree * radians));
    assert(difference(previous, current) <= 2);
    assert(peak(current) == TiltColorWheel::kMaxBrightness);
    previous = current;
  }
  assert(difference(previous, TiltColorWheel::colorFor(1, 0)) <= 1);
}

static void tiltControlsBrightness() {
  const TiltRgb off;
  assert(TiltColorWheel::colorFor(0, 0) == off);
  assert(TiltColorWheel::colorFor(-0.0f, 0.0f) == off);
  const float quiet[] = {-0.02f, 0, 0.02f};
  for (float x : quiet) for (float y : quiet)
    assert(TiltColorWheel::colorFor(x, y) == off);

  // Increasing tilt must brighten monotonically in each direction; it must
  // saturate instead of overflowing under strong tilt or acceleration.
  const float directions[][2] = {
    {1, 0}, {-1, 0}, {0, 1}, {0, -1},
    {1, 1}, {-1, 1}, {1, -1}, {-1, -1}, {1, 0.4f}
  };
  for (const auto &direction : directions) {
    int previous = 0;
    bool sawIntermediate = false;
    for (int step = 0; step <= 2000; ++step) {
      const float magnitude = step * 0.001f;
      const TiltRgb color = TiltColorWheel::colorFor(
        direction[0] * magnitude, direction[1] * magnitude);
      const int brightness = peak(color);
      assert(brightness >= previous);
      assert(brightness <= TiltColorWheel::kMaxBrightness);
      if (brightness > 0 && brightness < TiltColorWheel::kMaxBrightness)
        sawIntermediate = true;
      previous = brightness;
    }
    assert(sawIntermediate);
    assert(previous == TiltColorWheel::kMaxBrightness);
  }
  for (unsigned maximum = 0; maximum <= 255; ++maximum) {
    const uint8_t cap = static_cast<uint8_t>(maximum);
    assert(TiltColorWheel::colorFor(0, -1, cap) == TiltRgb(0, 0, cap));
    const TiltRgb color = TiltColorWheel::colorFor(-0.2f, 0.3f, cap);
    assert(peak(color) <= cap);
  }
}

static void invalidInputIsDark() {
  const float invalid[] = {NAN, INFINITY, -INFINITY};
  for (float value : invalid) {
    assert(TiltColorWheel::colorFor(value, 0.5f) == TiltRgb());
    assert(TiltColorWheel::colorFor(0.5f, value) == TiltRgb());
    assert(TiltColorWheel::colorFor(value, value) == TiltRgb());
  }
  const float extreme[] = {-FLT_MAX, -1000, -1, -FLT_MIN, 0,
                           FLT_MIN, 1, 1000, FLT_MAX};
  for (float x : extreme) for (float y : extreme)
    assert(peak(TiltColorWheel::colorFor(x, y)) <= TiltColorWheel::kMaxBrightness);
}

static void scheduling(uint32_t start) {
  TiltColorWheel wheel;
  const uint32_t period = TiltColorWheel::kUpdateUs;
  assert(period == 40000); // One possible hardware write per 40ms / 25Hz.
  assert(wheel.update(start, 0, -1, true));
  assert(wheel.color() == TiltRgb(0, 0, 32));
  assert(!wheel.update(start + period - 1, 1, 0, true));
  assert(wheel.color() == TiltRgb(0, 0, 32));
  assert(wheel.update(start + period, 1, 0, true));
  assert(wheel.color() == TiltRgb(0, 32, 0));

  // Evaluate a stable pose without repeatedly sending the same RGB packet.
  assert(!wheel.update(start + period * 2, 1, 0, true));
  assert(!wheel.update(start + period * 2 + 1, -1, 0, true));
  assert(wheel.update(start + period * 3, -1, 0, true));
  assert(wheel.color() == TiltRgb(32, 0, 0));

  // Loss of usable input turns the LED off without waiting for its next slot.
  const uint32_t fault = start + period * 3 + 1;
  assert(wheel.update(fault, -1, 0, false));
  assert(wheel.color() == TiltRgb());
  assert(!wheel.update(fault + 1, -1, 0, false));
  assert(!wheel.update(fault + 2, -1, 0, true));
  assert(wheel.update(fault + period, 0, 1, true));
  assert(wheel.color() == TiltRgb(32, 32, 0));

  // A delayed call sends only the latest result; no catch-up burst of writes.
  const uint32_t delayed = fault + period * 15;
  assert(wheel.update(delayed, -1, -1, true));
  assert(wheel.color() == TiltRgb(32, 0, 32));
  assert(!wheel.update(delayed + 1, 1, 0, true));
}

static void initializationAndInvalidRecovery() {
  TiltColorWheel wheel;
  assert(wheel.update(0, 0, 0, false)); // Explicit first black packet.
  assert(wheel.color() == TiltRgb());
  assert(!wheel.update(1, 0, 0, false));
  assert(wheel.update(TiltColorWheel::kUpdateUs, 0, -1, true));
  assert(wheel.color() == TiltRgb(0, 0, 32));
  assert(wheel.update(TiltColorWheel::kUpdateUs * 2, NAN, 0, true));
  assert(wheel.color() == TiltRgb());
  assert(!wheel.update(TiltColorWheel::kUpdateUs * 3, NAN, 0, true));
  assert(wheel.update(TiltColorWheel::kUpdateUs * 4, 1, 0, true));
  assert(wheel.color() == TiltRgb(0, 32, 0));
}

int main() {
  directionalAnchors();
  tiltControlsBrightness();
  invalidInputIsDark();
  scheduling(100000);
  scheduling(UINT32_MAX - 20000U);
  initializationAndInvalidRecovery();
  puts("Tilt RGB: wheel continuity, tilt brightness, bounded invalid input, 25Hz scheduling, deduplication and timer wrap passed.");
}
