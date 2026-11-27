#ifndef SIMCIRCLE_TEST_ARDUINO_H
#define SIMCIRCLE_TEST_ARDUINO_H

#include <stdint.h>

// Deterministic clock, supplied by the driver test translation unit.
uint32_t millis();
uint32_t micros();
void delay(uint32_t milliseconds);

#endif
