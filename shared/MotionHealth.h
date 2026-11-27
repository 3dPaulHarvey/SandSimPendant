#ifndef SIMCIRCLE_MOTION_HEALTH_H
#define SIMCIRCLE_MOTION_HEALTH_H

#include <stdint.h>

namespace motion {
// Extra shake effects amplify input, so require a clean, fresh sensor stream.
// Ordinary accelerometer tilt remains independent of this optional-effect gate.
class MotionHealth {
 public:
  static constexpr uint32_t kRecoveryUs = 750000;
  static constexpr uint32_t kFreshnessUs = 50000;

  // Returns true when the transient detector should forget its input history.
  bool update(uint32_t nowUs, uint32_t errors, bool online, bool ready, bool fresh) {
    const bool stale = haveFresh_ && static_cast<uint32_t>(nowUs - lastFreshUs_) > kFreshnessUs;
    const bool fault = !observed_ || errors != errors_ || !online || !ready || stale;
    observed_ = true;
    errors_ = errors;
    if (fault) {
      lastFaultUs_ = nowUs;
      haveFresh_ = false;
    }
    if (fresh && online && ready) {
      if (!haveFresh_) lastFaultUs_ = nowUs; // Silence before this sample is not clean data.
      lastFreshUs_ = nowUs;
      haveFresh_ = true;
    }
    return fault;
  }

  bool trusted(uint32_t nowUs) const {
    return haveFresh_ && static_cast<uint32_t>(nowUs - lastFreshUs_) <= kFreshnessUs &&
           static_cast<uint32_t>(nowUs - lastFaultUs_) >= kRecoveryUs;
  }

 private:
  uint32_t errors_ = 0, lastFaultUs_ = 0, lastFreshUs_ = 0;
  bool haveFresh_ = false, observed_ = false;
};
} // namespace motion
#endif
