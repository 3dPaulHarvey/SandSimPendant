#ifndef GRAVITY_SAND_BUTTON_INPUT_H
#define GRAVITY_SAND_BUTTON_INPUT_H

#include <stdint.h>

namespace sand {

// Feed the logical pressed state (digitalRead(pin) == LOW for INPUT_PULLUP).
// A stable press emits one event; holding the switch never repeats it.
class ButtonInput {
 public:
  explicit ButtonInput(uint32_t debounceMs = 25) : debounceMs_(debounceMs) {}

  bool update(bool pressed, uint32_t nowMs) {
    if (!initialized_) {
      initialized_ = true;
      candidate_ = stable_ = pressed;
      changedAtMs_ = nowMs;
      // Ignore a switch held during boot until it has been released stably.
      armed_ = !pressed;
      return false;
    }

    if (pressed != candidate_) {
      candidate_ = pressed;
      changedAtMs_ = nowMs;
    }
    if (candidate_ == stable_ ||
        static_cast<uint32_t>(nowMs - changedAtMs_) < debounceMs_) return false;

    stable_ = candidate_;
    if (!stable_) {
      armed_ = true;
      return false;
    }
    const bool event = armed_;
    armed_ = false;
    return event;
  }

  void reset() { initialized_ = false; }

 private:
  uint32_t debounceMs_;
  uint32_t changedAtMs_ = 0;
  bool initialized_ = false;
  bool candidate_ = false;
  bool stable_ = false;
  bool armed_ = false;
};

}  // namespace sand

#endif
