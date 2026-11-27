#ifndef SIMCIRCLE_BOARD_CONFIG_H
#define SIMCIRCLE_BOARD_CONFIG_H

#include <stdint.h>

// Aideepen ESP32-S3 Super Mini. Numbers are raw GPIOs printed on the board.
// Standalone GC9A01 SPI display + GY-521 I2C module + two optional buttons.
namespace hardware {

constexpr int kLcdCS = 1;
constexpr int kLcdDC = 2;
constexpr int kLcdReset = 4;
constexpr int kLcdClock = 7;
constexpr int kLcdMosi = 9;
constexpr int kI2cSda = 12;
constexpr int kI2cScl = 13;
constexpr int kButtonPrimary = 10;
constexpr int kButtonSecondary = 11; // Randomize button; moved off GPIO8 for diagnosis.
constexpr int kRgbLed = 48;
constexpr int kDisplaySize = 240;
constexpr uint32_t kLcdSpiHz = 20000000; // Conservative starting point for jumper wires.
constexpr uint32_t kI2cHz = 400000;
// This particular replacement GY-521 reports an undocumented ID of 0x75.
// Permit that exact ID after configuration readback; 0 disables compatibility.
// Standard MPU6050 identity 0x68 is always supported.
constexpr uint8_t kImuCompatibilityId = 0x75;

// COM35 face-flipped mount: screen gravity = (+accelY, -accelX).
// Connector end remains at the top: infer a left/right reversal from COM33.
// Confirm physical tilt directions once the display is wired.
// Saved mounting adjustments override these defaults in both sketches.
constexpr bool kImuSwapAxes = true;
constexpr bool kImuFlipX = true;
constexpr bool kImuFlipY = true;

// Restrict external connections to the accessible numbered header pins.
// GPIO3 is a strapping pin; native USB and the onboard RGB LED stay reserved.
constexpr bool usableHeaderPin(int pin) { return pin >= 1 && pin <= 13 && pin != 3; }
constexpr int kPeripheralPins[] = {kLcdCS, kLcdDC, kLcdReset, kLcdClock,
                                 kLcdMosi, kI2cSda, kI2cScl,
                                 kButtonPrimary, kButtonSecondary};
constexpr int kPeripheralPinCount = sizeof(kPeripheralPins) / sizeof(kPeripheralPins[0]);
constexpr bool validPins(int index = 0) {
  return index == kPeripheralPinCount ||
         (usableHeaderPin(kPeripheralPins[index]) && validPins(index + 1));
}
constexpr bool differentFromRest(int first, int second) {
  return second == kPeripheralPinCount ||
         (kPeripheralPins[first] != kPeripheralPins[second] &&
          differentFromRest(first, second + 1));
}
constexpr bool uniquePins(int index = 0) {
  return index == kPeripheralPinCount ||
         (differentFromRest(index, index + 1) && uniquePins(index + 1));
}
static_assert(validPins(), "Use non-strapping Super Mini header GPIOs 1..13");
static_assert(uniquePins(), "LCD, IMU and buttons must have distinct GPIOs");

}  // namespace hardware

#endif
