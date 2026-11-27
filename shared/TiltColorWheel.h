#ifndef SIMCIRCLE_TILT_COLOR_WHEEL_H
#define SIMCIRCLE_TILT_COLOR_WHEEL_H

#include <math.h>
#include <stdint.h>

namespace motion {

struct TiltRgb {
  uint8_t r, g, b;
  constexpr TiltRgb(uint8_t red = 0, uint8_t green = 0, uint8_t blue = 0)
      : r(red), g(green), b(blue) {}
  bool operator==(const TiltRgb &other) const {
    return r == other.r && g == other.g && b == other.b;
  }
  bool operator!=(const TiltRgb &other) const { return !(*this == other); }
};

// Reuse filtered, screen-mapped gravity: +X right, +Y down, measured in g.
// Eight color anchors approximate the wheel; no angles, square roots, HSV
// conversion, allocations, or additional sensor reads are needed.
class TiltColorWheel {
 public:
  static constexpr uint32_t kUpdateUs = 40000; // At most 25 ordinary updates/s.
  static constexpr uint8_t kMaxBrightness = 32; // Per-channel ceiling, out of 255.
  static constexpr int kUnitsPerG = 1024;
  static constexpr int kDeadzoneQ = 41;  // About 0.04g: dark near level.
  static constexpr int kFullTiltQ = 666; // About 0.65g: full configured brightness.

  static TiltRgb colorFor(float x, float y, uint8_t maximum = kMaxBrightness) {
    if (!isfinite(x) || !isfinite(y) || !maximum) return TiltRgb();
    // Clamp before conversion so corrupt, but finite, floats cannot overflow.
    // Normal accelerometer gravity lies within +/-1g on each axis.
    const int qx = quantize(x), qy = quantize(y);
    const int ax = qx < 0 ? -qx : qx;
    const int ay = qy < 0 ? -qy : qy;
    const int major = ax > ay ? ax : ay;
    const int minor = ax > ay ? ay : ax;
    // Cheap length approximation; close enough for an LED brightness ramp.
    const int magnitude = major + ((minor * 3) >> 3);
    if (magnitude <= kDeadzoneQ) return TiltRgb();
    const int span = kFullTiltQ - kDeadzoneQ;
    const unsigned brightness = magnitude >= kFullTiltQ ? maximum :
      static_cast<unsigned>(((magnitude - kDeadzoneQ) * maximum + span / 2) / span);
    if (!brightness) return TiltRgb();

    // Only one variable integer divide: ratio within an octant, 0..256.
    const unsigned ratio = static_cast<unsigned>((minor << 8) / major);
    unsigned octant, fraction;
    if (qx >= 0 && qy < 0) { // Up -> cyan -> right.
      octant = ax <= ay ? 0 : 1;
      fraction = ax <= ay ? ratio : 256 - ratio;
    } else if (qx >= 0) { // Right -> yellow-green -> down.
      octant = ax >= ay ? 2 : 3;
      fraction = ax >= ay ? ratio : 256 - ratio;
    } else if (qy >= 0) { // Down -> orange -> left.
      octant = ax <= ay ? 4 : 5;
      fraction = ax <= ay ? ratio : 256 - ratio;
    } else { // Left -> magenta -> up.
      octant = ax >= ay ? 6 : 7;
      fraction = ax >= ay ? ratio : 256 - ratio;
    }
    static const TiltRgb anchors[8] = {
      TiltRgb(0, 0, 255), TiltRgb(0, 255, 255),
      TiltRgb(0, 255, 0), TiltRgb(128, 255, 0),
      TiltRgb(255, 255, 0), TiltRgb(255, 128, 0),
      TiltRgb(255, 0, 0), TiltRgb(255, 0, 255)
    };
    const TiltRgb &a = anchors[octant];
    const TiltRgb &b = anchors[(octant + 1) & 7];
    return TiltRgb(channel(a.r, b.r, fraction, brightness),
                   channel(a.g, b.g, fraction, brightness),
                   channel(a.b, b.b, fraction, brightness));
  }

  // True means write color() to the LED. First use initializes the output,
  // then identical RGB values are suppressed even when a sample is due.
  // Invalid/stale input may turn the LED off immediately; repeated faults do
  // not write again or postpone recovery by restarting the update timer.
  bool update(uint32_t nowUs, float x, float y, bool valid) {
    if (!valid || !isfinite(x) || !isfinite(y)) return setColor(TiltRgb());
    if (sampled_ && static_cast<uint32_t>(nowUs - lastUpdateUs_) < kUpdateUs)
      return false;
    sampled_ = true;
    lastUpdateUs_ = nowUs;
    return setColor(colorFor(x, y));
  }

  const TiltRgb &color() const { return color_; }

 private:
  uint32_t lastUpdateUs_ = 0;
  TiltRgb color_;
  bool sampled_ = false, initialized_ = false;

  static int quantize(float value) {
    if (value > 1.0f) value = 1.0f;
    else if (value < -1.0f) value = -1.0f;
    return static_cast<int>(value * kUnitsPerG);
  }

  static uint8_t channel(unsigned a, unsigned b, unsigned fraction,
                         unsigned brightness) {
    const unsigned mixed = (a * (256 - fraction) + b * fraction + 128) >> 8;
    return static_cast<uint8_t>((mixed * brightness + 127) / 255);
  }

  bool setColor(TiltRgb next) {
    const bool changed = !initialized_ || next != color_;
    initialized_ = true;
    color_ = next;
    return changed;
  }
};

} // namespace motion

#endif
