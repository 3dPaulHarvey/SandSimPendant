/*
  Gravity Sand: XIAO ESP32-S3 / GC9A01 240x240 / CST816D / MPU6050.
  Original working display and touch wiring is preserved.

  MPU6050 shares the touch I2C bus:
    VCC -> 3V3, GND -> GND, SDA -> D4/GPIO5, SCL -> D5/GPIO6.
    INT is unused. AD0 low/floating = 0x68, high = 0x69.

  Board: XIAO_ESP32S3, USB Mode: Hardware CDC and JTAG,
  USB CDC On Boot: Enabled. No PSRAM or new libraries needed.
  Tap or press D2 to scatter; D9 changes palette. Buttons connect to GND.
  Tune SandConfig.h. Serial monitor: 115200; '?' for help.
*/

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_GC9A01A.h>
#include <Preferences.h>
#include <esp_system.h>
#include <stdarg.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include "SandConfig.h" // Edit the documented performance/feel controls here.
#include "SandSimulation.h"
#include "ButtonInput.h"
#include "DirtyRenderer.h"
#include "../shared/MPU6050Motion.h"
#include "../shared/ShakeInput.h"

constexpr int LCD_CS = 1, LCD_DC = 2, LCD_RST = 4;
constexpr int LCD_SCK = 7, LCD_MOSI = 9;
constexpr int I2C_SDA = 5, I2C_SCL = 6;
constexpr int TOUCH_RST = 43, TOUCH_INT = 44;
constexpr uint8_t TOUCH_ADDR = 0x15;
constexpr uint32_t LCD_SPI_HZ = sand::config::kLcdSPIHz;
constexpr uint32_t STEP_US = (1000000UL + sand::config::kPhysicsHz / 2) / sand::config::kPhysicsHz;
constexpr uint32_t FRAME_US = (1000000UL + sand::config::kDisplayHz / 2) / sand::config::kDisplayHz;
constexpr uint16_t BLACK = 0;
constexpr int GRID_SIZE = sand::Simulation::kGridSize;
constexpr int CELL_SIZE = sand::Simulation::kCellSize;
constexpr int DISPLAY_SIZE = sand::Simulation::kDisplaySize;

// GPIO numbers, not the numbers printed after D on the board. GPIO8 is free
// because this write-only LCD uses SPI with MISO explicitly disabled below.
constexpr int BUTTON_SCATTER = sand::config::kButtonScatterPin; // D2 = GPIO3.
constexpr int BUTTON_PALETTE = sand::config::kButtonPalettePin; // D9 = GPIO8.
constexpr bool usedHardwarePin(int pin) {
  return pin == LCD_CS || pin == LCD_DC || pin == LCD_RST || pin == LCD_SCK ||
         pin == LCD_MOSI || pin == I2C_SDA || pin == I2C_SCL ||
         pin == TOUCH_RST || pin == TOUCH_INT || pin == 19 || pin == 20;
}
static_assert(BUTTON_SCATTER != BUTTON_PALETTE && !usedHardwarePin(BUTTON_SCATTER) &&
              !usedHardwarePin(BUTTON_PALETTE), "Button pin conflicts with LCD, touch, I2C or USB");
static_assert(sand::config::kPhysicsHz > 0 && sand::config::kDisplayHz > 0,
              "Update rates must be positive");
static_assert(sand::config::kRenderBandRows >= 1 && sand::config::kRenderBandRows <= 8,
              "Render bands must be 1..8 cell rows (bounded pixel buffer)");

Adafruit_GC9A01A tft(&SPI, LCD_DC, LCD_CS, LCD_RST);
sand::Simulation simulation;
Preferences preferences;
bool preferencesReady = false;

// Native RGB565. Warm sand and two optional palettes.
const uint16_t palettes[3][8] = {
  {0xF800, 0xF900, 0xFA00, 0xFB00, 0xFC00, 0xFD00, 0xFE00, 0xFFE8},
  {0x027F, 0x039F, 0x04BF, 0x05DF, 0x06FF, 0x07FF, 0x5FFF, 0xAFFF},
  {0xF810, 0xF81A, 0xD81F, 0xA81F, 0x781F, 0x501F, 0xC35F, 0xF69F}
};
uint8_t palette = 0;
uint32_t lcdSPIHz = LCD_SPI_HZ;

// The loop owns simulation, Wire, input, settings and Serial. The LCD task
// exclusively owns tft/SPI after setup. Frames contain copies, never pointers
// into live particles. A one-frame mailbox replaces stale pending frames.
struct RenderFrame {
  uint8_t cells[GRID_SIZE * GRID_SIZE];
  uint8_t palette;
  uint8_t indicator; // 0: waiting, 1: IMU, 2: manual/demo.
  bool showSensorStatus;
  uint32_t spiHz;
  uint32_t capturedUs;
};
void renderSand(const RenderFrame &frame); // Keep Arduino's prototype after this type.
struct RenderStats {
  uint32_t frames, changedFrames, totalUs, maxUs, pixels, windows;
  uint32_t ageTotalUs, maxAgeUs;
};
QueueHandle_t renderQueue = nullptr;
TaskHandle_t renderTaskHandle = nullptr;
portMUX_TYPE renderStatsMux = portMUX_INITIALIZER_UNLOCKED;
RenderStats renderStats = {};
RenderFrame outgoingFrame, drawingFrame;
uint8_t previousCells[GRID_SIZE * GRID_SIZE]; // LCD task only.
uint16_t pixelStrip[DISPLAY_SIZE * CELL_SIZE * sand::config::kRenderBandRows];
// Preserve the decorative rim even when merged rectangles include blank
// exterior cells. A 1-bit background uses 7.2KB, rather than a full RGB frame.
GFXcanvas1 background(DISPLAY_SIZE, DISPLAY_SIZE);
sand::ButtonInput scatterButton(sand::config::kButtonDebounceMs);
sand::ButtonInput paletteButton(sand::config::kButtonDebounceMs);

// Small bounded USB log queue: an absent monitor never blocks the sim.
char logQueue[16][200];
uint8_t logHead = 0, logTail = 0;
size_t logOffset = 0;
void logMessage(const char *format, ...) {
  uint8_t next = (logHead + 1) % 16;
  if (next == logTail) return;
  va_list args;
  va_start(args, format);
  vsnprintf(logQueue[logHead], sizeof(logQueue[0]), format, args);
  va_end(args);
  logHead = next;
}

void pumpLogs() {
  if (!Serial) {
    logHead = logTail = 0;
    logOffset = 0;
    return;
  }
  if (logHead == logTail) return;
  int room = Serial.availableForWrite();
  if (room <= 0) return;
  size_t remaining = strlen(logQueue[logTail]) - logOffset;
  size_t count = min(remaining, (size_t)min(room, 64));
  logOffset += Serial.write((uint8_t *)logQueue[logTail] + logOffset, count);
  if (logOffset == strlen(logQueue[logTail])) {
    logTail = (logTail + 1) % 16;
    logOffset = 0;
  }
}

bool readRegisters(uint8_t address, uint8_t reg, uint8_t *out,
                   uint8_t count, bool stopBetween = false) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(stopBetween) != 0) return false;
  if (Wire.requestFrom(address, count) != count) {
    while (Wire.available()) Wire.read();
    return false;
  }
  for (uint8_t i = 0; i < count; ++i) out[i] = Wire.read();
  return true;
}

bool writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission(true) == 0;
}

motion::MPU6050Motion motionSensor;
motion::ShakeInput shakeInput;
bool shakeEnabled = true;
float shakeGain = sand::config::kShakeGain, peakShakeImpulse = 0, peakShoveImpulse = 0;
uint32_t lastShakeSampleUs = 0, shakePulses = 0;
uint8_t imuAddress = 0;
bool imuOnline = false;
bool swapAxes = false, invertX = false, invertY = false;
float accelX = 0, accelY = 0, accelZ = 0;
float filteredX = 0, filteredY = 0;
float gravityX = 0, gravityY = 1;
uint32_t imuErrors = 0;
int gravityMode = 0; // 0: automatic, 1..4: down/left/up/right, 5: demo.

// LCD-task-only overlay helpers. UI state arrives in each immutable frame.
void invalidateSensorStatus() {
  for (int row = 8; row < 16; ++row)
    for (int col = 24; col < 56; ++col) previousCells[row * GRID_SIZE + col] = 0xFF;
}

void drawSensorStatus() {
  tft.fillRect(72, 24, 96, 24, BLACK);
  tft.setTextWrap(false);
  tft.setTextSize(1);
  tft.setTextColor(0xFD20, BLACK);
  tft.setCursor(90, 26);
  tft.print("NO MPU6050");
  tft.setCursor(75, 37);
  tft.print("SDA D4  SCL D5");
}

void drawIndicator(uint8_t indicator) {
  uint16_t color = indicator == 2 ? 0x07FF : (indicator == 1 ? 0x07E0 : 0xFD20);
  // Above the sand boundary; green = IMU, amber = waiting, cyan = manual/demo.
  tft.fillCircle(119, 2, 2, color);
}

void updateIMUState() {
  bool previous = imuOnline;
  imuOnline = motionSensor.online;
  imuAddress = motionSensor.address;
  accelX = motionSensor.accel.x;
  accelY = motionSensor.accel.y;
  accelZ = motionSensor.accel.z;
  filteredX = motionSensor.gravity.x;
  filteredY = motionSensor.gravity.y;
  imuErrors = motionSensor.errors;
  if (previous != imuOnline) {
    if (imuOnline) logMessage("MPU6050 connected 0x%02X; 200Hz/94Hz filter, tau8ms, calibration=%s.\n",
      imuAddress, motionSensor.calibration.valid ? "SAVED" : "NONE");
    else logMessage("MPU6050 disconnected; using screen-bottom gravity.\n");
  }
}

bool beginIMU() {
  motionSensor.setMapping(swapAxes, invertX, invertY);
  bool ok = motionSensor.begin();
  updateIMUState();
  return ok;
}

int16_t signedWord(const uint8_t *bytes) {
  return (int16_t)(((uint16_t)bytes[0] << 8) | bytes[1]);
}

void sampleIMU() {
  motionSensor.setMapping(swapAxes, invertX, invertY);
  bool fresh = motionSensor.update();
  if (!motionSensor.online) {
    shakeInput.reset();
    lastShakeSampleUs = 0;
  } else if (fresh) {
    uint32_t sampleUs = motionSensor.lastSampleUs;
    float dt = lastShakeSampleUs ? (uint32_t)(sampleUs - lastShakeSampleUs) * 0.000001f : 0.005f;
    shakeInput.update(motionSensor.accel, motionSensor.gyro, dt);
    lastShakeSampleUs = sampleUs;
  }
  updateIMUState();
}

void updateGravity() {
  gravityX = 0; gravityY = 1;
  if (gravityMode == 0 && imuOnline) {
    gravityX = filteredX; gravityY = filteredY;
  } else if (gravityMode == 2) { gravityX = -1; gravityY = 0;
  } else if (gravityMode == 3) { gravityX = 0; gravityY = -1;
  } else if (gravityMode == 4) { gravityX = 1; gravityY = 0;
  } else if (gravityMode == 5) {
    float angle = millis() * 0.00045f;
    gravityX = sinf(angle); gravityY = cosf(angle);
  }
}

void resetSand() {
  simulation.reset(esp_random());
  shakeInput.consume();
  logMessage("Scattered %d grains.\n", sand::Simulation::kParticleCount);
}

bool touchAvailable = false, touching = false, touchCancelled = false;
int touchStartX = 0, touchStartY = 0, touchExcursion2 = 0;
uint32_t touchBeganMs = 0, lastTouchGoodMs = 0, lastTouchPollUs = 0;

void beginTouch() {
  pinMode(TOUCH_RST, OUTPUT);
  pinMode(TOUCH_INT, INPUT_PULLUP);
  digitalWrite(TOUCH_RST, LOW); delay(10);
  digitalWrite(TOUCH_RST, HIGH); delay(150);
  bool ok = writeRegister(TOUCH_ADDR, 0xFA, 0x60);
  touchAvailable = writeRegister(TOUCH_ADDR, 0xFE, 0x01) && ok;
  logMessage("Touch %s; tap to scatter sand.\n", touchAvailable ? "ready" : "unavailable");
}

void sampleTouch() {
  if (!touchAvailable) return;
  uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastTouchPollUs) < 20000) return;
  lastTouchPollUs = nowUs;
  uint32_t now = millis();
  uint8_t bytes[7];
  // Preserve the STOP-between-reads behavior from the working touch sketch.
  if (!readRegisters(TOUCH_ADDR, 0x00, bytes, sizeof(bytes), true)) {
    if (touching && (uint32_t)(now - lastTouchGoodMs) > 250) touchCancelled = true;
    return;
  }
  int fingers = bytes[2] & 3;
  int event = bytes[3] >> 6;
  if (!fingers || event == 1) {
    if (touching && !touchCancelled && touchExcursion2 <= 15 * 15 &&
        (uint32_t)(now - lastTouchGoodMs) <= 250 &&
        (uint32_t)(now - touchBeganMs) <= 400) resetSand();
    touching = false;
    touchCancelled = false;
    return;
  }
  int x = ((bytes[3] & 15) << 8) | bytes[4];
  int y = ((bytes[5] & 15) << 8) | bytes[6];
  if (x >= 240 || y >= 240) return;
  if (!touching) {
    touching = true; touchBeganMs = now;
    touchStartX = x; touchStartY = y; touchExcursion2 = 0;
  } else if ((uint32_t)(now - lastTouchGoodMs) > 250) touchCancelled = true;
  int dx = x - touchStartX, dy = y - touchStartY;
  touchExcursion2 = max(touchExcursion2, dx * dx + dy * dy);
  lastTouchGoodMs = now;
}

uint8_t cellColor(int column, int row) {
  uint16_t id = simulation.particleAt(column, row);
  return id < sand::Simulation::kParticleCount ? simulation.particle(id).color + 1 : 0;
}

uint32_t physicsSteps = 0, physicsTotalUs = 0, maxPhysicsUs = 0;

void renderSand(const RenderFrame &frame) {
  const uint32_t began = micros();
  static uint8_t previousPalette = 0, previousIndicator = 0xFF;
  static bool previousStatus = false;
  static uint32_t previousSPIHz = LCD_SPI_HZ;
  if (frame.spiHz != previousSPIHz) {
    tft.setSPISpeed(frame.spiHz);
    previousSPIHz = frame.spiHz;
  }
  if (frame.palette != previousPalette) {
    for (int row = 0; row < GRID_SIZE; ++row)
      for (int col = 0; col < GRID_SIZE; ++col)
        if (sand::Simulation::inside(col, row)) previousCells[row * GRID_SIZE + col] = 0xFF;
  }
  if (frame.showSensorStatus != previousStatus) invalidateSensorStatus();
  bool statusDirty = frame.showSensorStatus != previousStatus;
  bool indicatorDirty = frame.indicator != previousIndicator;
  bool transaction = false;
  uint32_t pixels = 0, windows = 0;
  const uint8_t *rim = background.getBuffer();
  const sand::render::Options options(sand::config::kRenderMergeGapCells,
    sand::config::kRenderBandRows, sand::config::kRenderMergeOverdrawCells);
  sand::render::plan(frame.cells, previousCells, GRID_SIZE, GRID_SIZE, options,
    [&](const sand::render::Rect &rect) {
      const int width = rect.w * CELL_SIZE, height = rect.h * CELL_SIZE;
      for (int cellY = 0; cellY < rect.h; ++cellY) {
        for (int yy = 0; yy < CELL_SIZE; ++yy) {
          const int pixelY = (rect.y + cellY) * CELL_SIZE + yy;
          for (int cellX = 0; cellX < rect.w; ++cellX) {
            const uint8_t color = frame.cells[(rect.y + cellY) * GRID_SIZE + rect.x + cellX];
            const int offset = (cellY * CELL_SIZE + yy) * width + cellX * CELL_SIZE;
            if (color) {
              const uint16_t rgb = palettes[frame.palette][color - 1];
              for (int xx = 0; xx < CELL_SIZE; ++xx) pixelStrip[offset + xx] = rgb;
            } else {
              for (int xx = 0; xx < CELL_SIZE; ++xx) {
                const int pixelX = (rect.x + cellX) * CELL_SIZE + xx;
                const bool onRim = rim && (rim[pixelY * ((DISPLAY_SIZE + 7) / 8) + pixelX / 8] &
                                          (0x80 >> (pixelX & 7)));
                pixelStrip[offset + xx] = onRim ? 0x18E3 : BLACK;
              }
            }
          }
        }
      }
      if (!transaction) { tft.startWrite(); transaction = true; }
      tft.setAddrWindow(rect.x * CELL_SIZE, rect.y * CELL_SIZE, width, height);
      tft.writePixels(pixelStrip, width * height, true, false);
      pixels += width * height;
      ++windows;
      if (rect.y < 16 && rect.y + rect.h > 8 && rect.x < 56 && rect.x + rect.w > 24)
        statusDirty = true;
      if (rect.y * CELL_SIZE <= 4) indicatorDirty = true;
    });
  if (transaction) tft.endWrite();
  memcpy(previousCells, frame.cells, sizeof(previousCells));
  if (statusDirty && frame.showSensorStatus) drawSensorStatus();
  if (indicatorDirty) drawIndicator(frame.indicator);
  previousPalette = frame.palette;
  previousIndicator = frame.indicator;
  previousStatus = frame.showSensorStatus;
  const uint32_t duration = micros() - began;
  const uint32_t age = micros() - frame.capturedUs;
  portENTER_CRITICAL(&renderStatsMux);
  ++renderStats.frames;
  if (transaction || indicatorDirty || (statusDirty && frame.showSensorStatus)) ++renderStats.changedFrames;
  renderStats.totalUs += duration;
  if (duration > renderStats.maxUs) renderStats.maxUs = duration;
  renderStats.pixels += pixels;
  renderStats.windows += windows;
  renderStats.ageTotalUs += age;
  if (age > renderStats.maxAgeUs) renderStats.maxAgeUs = age;
  portEXIT_CRITICAL(&renderStatsMux);
}

void lcdTask(void *) {
  for (;;) {
    if (xQueueReceive(renderQueue, &drawingFrame, portMAX_DELAY) == pdTRUE) renderSand(drawingFrame);
    // Give core 0's idle/system tasks time even during continuous full motion.
    vTaskDelay(1);
  }
}

void beginRenderer() {
  renderQueue = xQueueCreate(1, sizeof(RenderFrame));
#if CONFIG_FREERTOS_UNICORE
  const BaseType_t displayCore = 0;
#else
  const BaseType_t displayCore = 1 - xPortGetCoreID();
#endif
  if (renderQueue && xTaskCreatePinnedToCore(lcdTask, "sandLCD",
      sand::config::kRenderTaskStackBytes, nullptr, sand::config::kRenderTaskPriority,
      &renderTaskHandle, displayCore) == pdPASS) {
    logMessage("Cores: input/physics=%d LCD=%d; latest-frame mailbox, no WiFi/Bluetooth.\n",
               xPortGetCoreID(), displayCore);
  } else {
    if (renderQueue) vQueueDelete(renderQueue);
    renderQueue = nullptr;
    logMessage("LCD task allocation failed; using single-task rendering.\n");
  }
}

void publishFrame() {
  for (int row = 0; row < GRID_SIZE; ++row)
    for (int col = 0; col < GRID_SIZE; ++col) outgoingFrame.cells[row * GRID_SIZE + col] = cellColor(col, row);
  outgoingFrame.palette = palette;
  outgoingFrame.indicator = gravityMode ? 2 : (imuOnline ? 1 : 0);
  outgoingFrame.showSensorStatus = !imuOnline && gravityMode == 0;
  outgoingFrame.spiHz = lcdSPIHz;
  outgoingFrame.capturedUs = micros();
  if (renderQueue) xQueueOverwrite(renderQueue, &outgoingFrame);
  else renderSand(outgoingFrame);
}

void cyclePalette() {
  palette = (palette + 1) % 3;
  logMessage("Palette %u.\n", palette + 1);
}

void sampleButtons() {
  if (!sand::config::kButtonsEnabled) return;
  const uint32_t now = millis();
  if (scatterButton.update(digitalRead(BUTTON_SCATTER) == LOW, now)) resetSand();
  if (paletteButton.update(digitalRead(BUTTON_PALETTE) == LOW, now)) cyclePalette();
}

void showHelp() {
  logMessage("Gravity Sand: r=scatter, p=palette, d=rotating demo, 0=live IMU.\n");
  logMessage("1=down, 2=left, 3=up, 4=right (manual gravity). Tap=scatter.\n");
  logMessage("Mount mapping: s=swap axes, x=flip X, y=flip Y (saved). v=40/8MHz LCD.\n");
  logMessage("Top dot: green=IMU, amber=waiting, cyan=manual/demo. ?=help, i=scan, m=MPU diagnostic.\n");
  logMessage("Shake: h=on/off, +/-=strength, t=test jolt. XYZ shocks loosen grains; tilt still drives gravity.\n");
  logMessage("Buttons: D2/GPIO3=scatter, D9/GPIO8=palette; normally open to GND. Tune SandConfig.h.\n");
}

void diagnoseIMU() {
  logMessage("Shake %s gain=%.2f linear=(%.3f,%.3f,%.3f)g activity=%.3f.\n",
    shakeEnabled ? "ON" : "OFF", shakeGain,
    shakeInput.latestLinear.x, shakeInput.latestLinear.y, shakeInput.latestLinear.z, shakeInput.activity);
  logMessage("Motion mode=FAST, 200Hz sensor, 94Hz filter, tau8ms; calibration=%s.\n",
    motionSensor.calibration.valid ? "SAVED" : "NONE");
  logMessage("Saved accel bias=(%.5f,%.5f,%.5f)g. Calibrate using MPU_Visualizer.\n",
    motionSensor.calibration.accelBias.x, motionSensor.calibration.accelBias.y, motionSensor.calibration.accelBias.z);
  logMessage("MPU wiring: SDA=D4/GPIO5, SCL=D5/GPIO6, VCC=3V3, GND=GND.\n");
  logMessage("LCD D8/D10 are SPI. MPU shares touch I2C, not LCD SDA/SCL. Idle SDA=%d SCL=%d\n",
             digitalRead(I2C_SDA), digitalRead(I2C_SCL));
  const uint8_t addresses[] = {TOUCH_ADDR, 0x68, 0x69};
  for (uint8_t address : addresses) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission(true);
    logMessage("Probe 0x%02X: %s (I2C error=%u)\n", address,
               error == 0 ? "ACK" : "NO ACK", error);
    if (error || address == TOUCH_ADDR) continue;
    for (int style = 0; style < 2; ++style) {
      uint8_t who = 0;
      bool ok = readRegisters(address, 0x75, &who, 1, style != 0);
      logMessage("0x%02X WHO_AM_I %s=%s 0x%02X (MPU6050 expects 0x68)\n",
                 address, style ? "STOP" : "RESTART", ok ? "OK" : "FAIL", who);
    }
    uint8_t bytes[6];
    if (readRegisters(address, 0x3B, bytes, sizeof(bytes)))
      logMessage("0x%02X raw accel: X=%d Y=%d Z=%d\n", address,
                 signedWord(bytes), signedWord(bytes + 2), signedWord(bytes + 4));
  }
}

void scanI2C() {
  logMessage("I2C scan: SDA=GPIO5/D4, SCL=GPIO6/D5.\n");
  for (uint8_t address = 1; address < 127; ++address) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) logMessage("I2C device at 0x%02X\n", address);
    pumpLogs();
  }
}

void handleSerial() {
  // Bound work per loop even if a monitor sends a large burst.
  for (int count = 0; count < 8 && Serial.available(); ++count) {
    char command = Serial.read();
    if (command == '?') showHelp();
    else if (command == 'r') resetSand();
    else if (command == 'h') {
      shakeEnabled = !shakeEnabled;
      shakeInput.consume();
      logMessage("Shake %s, gain=%.2f.\n", shakeEnabled ? "ON" : "OFF", shakeGain);
    } else if (command == '+' || command == '-') {
      shakeGain = constrain(shakeGain + (command == '+' ? 0.25f : -0.25f), 0.25f, 3.0f);
      logMessage("Shake strength=%.2f.\n", shakeGain);
    } else if (command == 't') {
      float impulse = min(sand::config::kMaxKick, 18.0f * shakeGain);
      simulation.kick(0, 0, impulse);
      peakShakeImpulse = max(peakShakeImpulse, impulse);
      ++shakePulses;
      logMessage("Test shake impulse.\n");
    }
    else if (command == 'p') cyclePalette();
    else if (command >= '0' && command <= '4') {
      gravityMode = command - '0';
    } else if (command == 'd') {
      gravityMode = gravityMode == 5 ? 0 : 5;
    } else if (command == 's' || command == 'x' || command == 'y') {
      if (command == 's') swapAxes = !swapAxes;
      if (command == 'x') invertX = !invertX;
      if (command == 'y') invertY = !invertY;
      motionSensor.setMapping(swapAxes, invertX, invertY);
      if (preferencesReady) {
        preferences.putBool("swap", swapAxes);
        preferences.putBool("flipX", invertX);
        preferences.putBool("flipY", invertY);
      }
      logMessage("Mount: swap=%d flipX=%d flipY=%d (saved when NVS available).\n", swapAxes, invertX, invertY);
    } else if (command == 'v') {
      lcdSPIHz = lcdSPIHz == 40000000 ? 8000000 : 40000000;
      logMessage("LCD SPI = %lu MHz.\n", (unsigned long)(lcdSPIHz / 1000000));
    } else if (command == 'i') scanI2C();
    else if (command == 'm') diagnoseIMU();
  }
}

uint32_t lastStepUs = 0, lastFrameUs = 0, lastStatsMs = 0;

void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(1);
  preferencesReady = preferences.begin("gravity-sand", false);
  if (preferencesReady) {
    swapAxes = preferences.getBool("swap", false);
    invertX = preferences.getBool("flipX", false);
    invertY = preferences.getBool("flipY", false);
  }
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  Wire.setTimeOut(10);
  SPI.begin(LCD_SCK, -1, LCD_MOSI, LCD_CS);
  tft.begin(LCD_SPI_HZ);
  tft.setRotation(0);
  tft.fillScreen(BLACK);
  tft.drawCircle(119, 119, 118, 0x18E3);
  if (background.getBuffer()) background.drawCircle(119, 119, 118, 1);
  if (sand::config::kButtonsEnabled) {
    pinMode(BUTTON_SCATTER, INPUT_PULLUP);
    pinMode(BUTTON_PALETTE, INPUT_PULLUP);
    sampleButtons(); // Seed held-at-boot states before any startup delays.
  }
  beginTouch();
  if (!beginIMU()) logMessage("Waiting for MPU6050 at 0x68/0x69; using screen-bottom gravity.\n");
  resetSand();
  showHelp();
  beginRenderer(); // All subsequent LCD access belongs to the renderer.
  publishFrame();
  lastStepUs = lastFrameUs = micros();
  lastStatsMs = millis();
}

void loop() {
  handleSerial();
  sampleIMU();
  sampleTouch();
  sampleButtons();
  updateGravity();
  uint32_t nowUs = micros();
  int steps = 0;
  while ((uint32_t)(nowUs - lastStepUs) >= STEP_US && steps < sand::config::kMaxCatchUpSteps) {
    uint32_t began = micros();
    motion::ShakePulse pulse = shakeInput.consume();
    if (shakeEnabled && imuOnline && gravityMode == 0) {
      float impulse = min(sand::config::kMaxKick, pulse.agitation * shakeGain);
      motion::Vec2 shove = motion::mapGravity(pulse.linearImpulseGSeconds, swapAxes, invertX, invertY);
      // Deliberately amplify only transient movement for a clearer tabletop
      // shove. The ordinary tilt path keeps its original force/speed.
      shove.x *= sand::config::kAcceleration * sand::config::kShoveGain * shakeGain;
      shove.y *= sand::config::kAcceleration * sand::config::kShoveGain * shakeGain;
      float shoveLength = sqrtf(shove.x * shove.x + shove.y * shove.y);
      if (shoveLength > sand::config::kMaxKick) {
        shove.x *= sand::config::kMaxKick / shoveLength;
        shove.y *= sand::config::kMaxKick / shoveLength;
        shoveLength = sand::config::kMaxKick;
      }
      if (impulse > 0 || shoveLength > 0) {
        simulation.kick(shove.x, shove.y, impulse);
        peakShakeImpulse = max(peakShakeImpulse, impulse);
        peakShoveImpulse = max(peakShoveImpulse, shoveLength);
        ++shakePulses;
      }
    }
    simulation.step(gravityX, gravityY, 1.0f / sand::config::kPhysicsHz);
    uint32_t duration = micros() - began;
    physicsTotalUs += duration;
    maxPhysicsUs = max(maxPhysicsUs, duration);
    lastStepUs += STEP_US;
    ++steps; ++physicsSteps;
  }
  if ((uint32_t)(nowUs - lastStepUs) >= STEP_US) lastStepUs = nowUs;
  nowUs = micros();
  if ((uint32_t)(nowUs - lastFrameUs) >= FRAME_US) {
    // Keep the cadence tied to scheduled deadlines, so polling and scheduler
    // jitter do not accumulate into a lower frame rate. Skip stale deadlines
    // after a slow frame rather than drawing a burst of duplicate frames.
    lastFrameUs += FRAME_US;
    if ((uint32_t)(nowUs - lastFrameUs) >= FRAME_US) lastFrameUs = nowUs;
    publishFrame();
  }
  uint32_t now = millis(), elapsed = now - lastStatsMs;
  if (elapsed >= sand::config::kStatsIntervalMs) {
    RenderStats drawing;
    portENTER_CRITICAL(&renderStatsMux);
    drawing = renderStats;
    renderStats = {};
    portEXIT_CRITICAL(&renderStatsMux);
    logMessage("%s mode=%d g=(%.2f,%.2f) accel=(%.2f,%.2f,%.2f) steps=%.1f/s fps=%.1f renderMax=%luus errors=%lu heap=%lu\n",
      imuOnline ? "IMU" : "WAIT", gravityMode, gravityX, gravityY,
      accelX, accelY, accelZ, physicsSteps * 1000.0f / elapsed,
      drawing.frames * 1000.0f / elapsed, (unsigned long)drawing.maxUs,
      (unsigned long)imuErrors, (unsigned long)ESP.getFreeHeap());
    logMessage("PERF grains=%d SPI=%luMHz fps=%.1f changed=%.1f drawAvg=%luus drawMax=%luus stepAvg=%luus stepMax=%luus px/frame=%lu win/frame=%lu\n",
      sand::Simulation::kParticleCount, (unsigned long)(lcdSPIHz / 1000000),
      drawing.frames * 1000.0f / elapsed, drawing.changedFrames * 1000.0f / elapsed,
      (unsigned long)(drawing.frames ? drawing.totalUs / drawing.frames : 0),
      (unsigned long)drawing.maxUs,
      (unsigned long)(physicsSteps ? physicsTotalUs / physicsSteps : 0),
      (unsigned long)maxPhysicsUs,
      (unsigned long)(drawing.frames ? drawing.pixels / drawing.frames : 0),
      (unsigned long)(drawing.frames ? drawing.windows / drawing.frames : 0));
    logMessage("DISPLAY snapshot-to-draw avg=%luus max=%luus; input/physics core=%d.\n",
      (unsigned long)(drawing.frames ? drawing.ageTotalUs / drawing.frames : 0),
      (unsigned long)drawing.maxAgeUs, xPortGetCoreID());
    logMessage("SHAKE on=%d gain=%.2f linear=(%.3f,%.3f,%.3f)g activity=%.3f pulses=%lu peak=%.2f shove=%.2f\n",
      shakeEnabled, shakeGain, shakeInput.latestLinear.x, shakeInput.latestLinear.y,
      shakeInput.latestLinear.z, shakeInput.activity, (unsigned long)shakePulses, peakShakeImpulse, peakShoveImpulse);
    lastStatsMs = now;
    physicsSteps = physicsTotalUs = maxPhysicsUs = 0;
    peakShakeImpulse = peakShoveImpulse = 0; shakePulses = 0;
  }
  pumpLogs();
  delay(sand::config::kLoopDelayMs);
}
