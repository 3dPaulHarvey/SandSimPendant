#ifndef SIMCIRCLE_TILT_RGB_LED_H
#define SIMCIRCLE_TILT_RGB_LED_H

#include <Arduino.h>
#include "BoardConfig.h"
#include "TiltColorWheel.h"

namespace motion {

// Loop-core only. The portable wheel limits work to 25 Hz and skips identical
// colors. Arduino's existing RGB driver uses hardware RMT, independent of SPI.
class TiltRgbLed {
 public:
  static constexpr uint32_t kFreshnessUs = 150000;

  void begin() {
    // Initialize the RMT channel before normal input/physics scheduling.
    rgbLedWrite(hardware::kRgbLed, 0, 0, 0);
    wheel_.update(micros(), 0, 0, false);
  }

  void update(uint32_t nowUs, float x, float y, bool ready, uint32_t sampleUs) {
    const bool fresh = ready && static_cast<uint32_t>(nowUs - sampleUs) <= kFreshnessUs;
    if (!wheel_.update(nowUs, x, y, fresh)) return;
    const TiltRgb &rgb = wheel_.color();
    const uint32_t began = micros();
    rgbLedWrite(hardware::kRgbLed, rgb.r, rgb.g, rgb.b);
    const uint32_t duration = micros() - began;
    ++writes;
    totalWriteUs += duration;
    if (duration > maxWriteUs) maxWriteUs = duration;
  }

  const TiltRgb &color() const { return wheel_.color(); }
  void resetStats() { writes = totalWriteUs = maxWriteUs = 0; }
  uint32_t writes = 0, totalWriteUs = 0, maxWriteUs = 0;

 private:
  TiltColorWheel wheel_;
};

} // namespace motion
#endif
