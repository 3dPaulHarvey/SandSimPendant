#ifndef GRAVITY_SAND_FLUID_IMPACT_H
#define GRAVITY_SAND_FLUID_IMPACT_H

#include <math.h>

namespace sand {
namespace impact {

// Visual state only. A real wall collision starts one short, bounded pulse;
// leaving the wall re-arms it. Settled contacts never keep the shape breathing.
class Pulse {
 public:
  static constexpr float kMinSpeed = 80.0f;
  static constexpr float kFullSpeed = 400.0f;
  static constexpr float kAttack = 0.025f;
  static constexpr float kRelease = 0.115f;
  static constexpr float kRebound = 0.165f;
  static constexpr float kDuration = 0.240f;
  static constexpr float kMinScale = 0.65f;
  static constexpr float kMaxScale = 1.10f;

  float normalX = 0, normalY = 0;

  void reset() {
    normalX = normalY = strength_ = 0;
    age_ = kDuration;
    armed_ = true;
  }

  void advance(float seconds) {
    if (!active() || !isfinite(seconds) || seconds <= 0) return;
    age_ += seconds;
    if (age_ >= kDuration) {
      age_ = kDuration;
      strength_ = 0;
      normalX = normalY = 0;
    }
  }

  void leaveWall() { armed_ = true; }

  void contact(float incomingSpeed, float nx, float ny) {
    const bool wasArmed = armed_;
    armed_ = false;
    if (!wasArmed || active() || !isfinite(incomingSpeed) ||
        incomingSpeed <= kMinSpeed || !isfinite(nx) || !isfinite(ny)) return;
    const float length2 = nx * nx + ny * ny;
    // The collision solver supplies a unit normal. Reject corrupt input rather
    // than adding a second normalization/square-root to every collision.
    if (!isfinite(length2) || length2 < 0.99f || length2 > 1.01f) return;
    strength_ = (incomingSpeed - kMinSpeed) / (kFullSpeed - kMinSpeed);
    if (strength_ > 1) strength_ = 1;
    normalX = nx;
    normalY = ny;
    age_ = 0;
  }

  bool active() const { return strength_ > 0 && age_ < kDuration; }

  float scale() const {
    if (!active()) return 1;
    if (age_ < kAttack)
      return 1 - (1 - kMinScale) * strength_ * smooth(age_ / kAttack);
    if (age_ < kRelease)
      return 1 - (1 - kMinScale) * strength_ *
          (1 - smooth((age_ - kAttack) / (kRelease - kAttack)));
    if (age_ < kRebound)
      return 1 + (kMaxScale - 1) * strength_ *
          smooth((age_ - kRelease) / (kRebound - kRelease));
    return 1 + (kMaxScale - 1) * strength_ *
        (1 - smooth((age_ - kRebound) / (kDuration - kRebound)));
  }

  // Smoothly take over from velocity stretch and then return control to it.
  float weight() const {
    if (!active()) return 0;
    if (age_ < kAttack) return strength_ * smooth(age_ / kAttack);
    if (age_ < kRelease) return strength_;
    return strength_ * (1 - smooth((age_ - kRelease) / (kDuration - kRelease)));
  }

 private:
  float age_ = kDuration, strength_ = 0;
  bool armed_ = true;

  static float smooth(float value) { return value * value * (3 - 2 * value); }
};

} // namespace impact
} // namespace sand
#endif
