#include <Arduino.h>
#include <esp_system.h>
#include "../shared/BoardConfig.h"

// Aideepen ESP32-S3 Super Mini: addressable onboard RGB LED on GPIO48.
// Raw GPIO numbers, independent of board-specific LED_BUILTIN aliases.
constexpr uint8_t RGB_PIN = hardware::kRgbLed;
constexpr uint8_t BRIGHTNESS = 24;
constexpr uint32_t STEP_MS = 1000;

void setup() {
  rgbLedWrite(RGB_PIN, 0, 0, 0);
  Serial.begin(115200);
  // Never wait for a serial monitor: LEDs must work with USB power alone.
}

void loop() {
  static uint8_t phase = 0;
  static const char *names[] = {"RED", "OFF", "GREEN", "OFF", "BLUE", "OFF"};
  rgbLedWrite(RGB_PIN,
              phase == 0 ? BRIGHTNESS : 0,
              phase == 2 ? BRIGHTNESS : 0,
              phase == 4 ? BRIGHTNESS : 0);

  if (Serial) {
    Serial.printf("S3_DIAG uptime=%lu ms RGB48=%s reset=%d flash=%lu bytes heap=%lu\n",
                  static_cast<unsigned long>(millis()), names[phase],
                  static_cast<int>(esp_reset_reason()),
                  static_cast<unsigned long>(ESP.getFlashChipSize()),
                  static_cast<unsigned long>(ESP.getFreeHeap()));
  }
  phase = (phase + 1) % 6;
  delay(STEP_MS);
}
