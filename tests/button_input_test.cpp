#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "../Gravity_Sand/ButtonInput.h"

int main() {
  // Contact bounce must settle for the full interval before one event fires.
  sand::ButtonInput button;
  assert(!button.update(false, 0));
  assert(!button.update(true, 10));
  assert(!button.update(false, 12));
  assert(!button.update(true, 15));
  assert(!button.update(false, 18));
  assert(!button.update(true, 20));
  assert(!button.update(true, 44));
  assert(button.update(true, 45));
  assert(!button.update(true, 500));
  // A short release bounce must not rearm a held switch.
  assert(!button.update(false, 501));
  assert(!button.update(true, 510));
  assert(!button.update(true, 600));
  assert(!button.update(false, 610));
  assert(!button.update(false, 635));
  assert(!button.update(true, 640));
  assert(button.update(true, 665));

  // A held-at-boot button is silent until a debounced release and new press.
  sand::ButtonInput bootHeld;
  assert(!bootHeld.update(true, 0));
  assert(!bootHeld.update(true, 1000));
  assert(!bootHeld.update(false, 1001));
  assert(!bootHeld.update(true, 1010));
  assert(!bootHeld.update(true, 1100));
  assert(!bootHeld.update(false, 1101));
  assert(!bootHeld.update(false, 1126));
  assert(!bootHeld.update(true, 1130));
  assert(bootHeld.update(true, 1155));

  // millis() wrapping must not shorten or extend the debounce interval.
  sand::ButtonInput wrapping;
  assert(!wrapping.update(false, UINT32_MAX - 20));
  assert(!wrapping.update(true, UINT32_MAX - 10));
  assert(!wrapping.update(true, 13));  // 24 elapsed milliseconds.
  assert(wrapping.update(true, 14));
  assert(!wrapping.update(false, UINT32_MAX - 5));
  assert(!wrapping.update(false, 19));
  assert(!wrapping.update(true, 20));
  assert(wrapping.update(true, 45));

  // Both buttons debounce independently and accept a configurable interval.
  sand::ButtonInput fast(5), slow(40);
  assert(!fast.update(false, 0));
  assert(!slow.update(false, 0));
  assert(!fast.update(true, 1));
  assert(!slow.update(true, 1));
  assert(fast.update(true, 6));
  assert(!slow.update(true, 6));
  assert(slow.update(true, 41));
  assert(!fast.update(true, 41));

  // Reset discards an old pending press and reapplies the boot-held policy.
  fast.reset();
  assert(!fast.update(true, 100));
  assert(!fast.update(true, 200));
  assert(!fast.update(false, 201));
  assert(!fast.update(false, 206));
  assert(!fast.update(true, 207));
  assert(fast.update(true, 212));

  // Zero debounce remains useful for electronically debounced inputs.
  sand::ButtonInput immediate(0);
  assert(!immediate.update(false, 0));
  assert(immediate.update(true, 1));
  assert(!immediate.update(true, 2));
  assert(!immediate.update(false, 3));
  assert(immediate.update(true, 4));

  puts("Button debounce: bounce, hold, boot-held, reset, independence and wrap pass.");
  return 0;
}
