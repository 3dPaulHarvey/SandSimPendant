/*
  Reactive fluid test: ESP32-S3 Super Mini / GC9A01 / GY-521 MPU6050.
  Pin assignments are shared with MPU_Visualizer in shared/BoardConfig.h.

  GY-521 I2C wiring:
    VCC -> 3V3, GND -> GND, SDA -> GPIO12, SCL -> GPIO13.
    INT is unused. AD0 low/floating = 0x68, high = 0x69.

  Board: ESP32S3 Dev Module, USB Mode: Hardware CDC and JTAG,
  USB CDC On Boot: Enabled. No PSRAM or new libraries needed.
  Press GPIO10 to explode; GPIO11 randomizes size/count/shape/swatch. Buttons connect to GND.
  Tune SandConfig.h. Serial monitor: 115200; '?' for help.
*/

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_GC9A01A.h>
#include <Preferences.h>
#include <esp_system.h>
#include <esp_random.h>
#include <bootloader_random.h>
#include <driver/gpio.h>
#include <stdarg.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include "SandConfig.h" // Edit the documented performance/feel controls here.
#include "SandSimulation.h"
#include "FluidVariation.h"
#include "FluidCluster.h"
#include "../shared/BoardConfig.h"
#include "../shared/ButtonInput.h"
#include "FluidRenderer.h"
#include "FluidBackground.h"
#include "FluidCoverage.h"
#include "FluidPopulation.h"
#include "../shared/MPU6050Motion.h"
#include "../shared/ShakeInput.h"
#include "../shared/MotionHealth.h"

constexpr int LCD_CS = hardware::kLcdCS, LCD_DC = hardware::kLcdDC;
constexpr int LCD_RST = hardware::kLcdReset;
constexpr int LCD_SCK = hardware::kLcdClock, LCD_MOSI = hardware::kLcdMosi;
constexpr int I2C_SDA = hardware::kI2cSda, I2C_SCL = hardware::kI2cScl;
constexpr uint32_t LCD_SPI_HZ = sand::config::kLcdSPIHz;
constexpr uint32_t STEP_US = (1000000UL + sand::config::kPhysicsHz / 2) / sand::config::kPhysicsHz;
constexpr uint32_t FRAME_US = (1000000UL + sand::config::kDisplayHz / 2) / sand::config::kDisplayHz;
constexpr int DISPLAY_SIZE = sand::Simulation::kDisplaySize;
constexpr int TILE_SIZE = sand::render::FluidRaster::kTileSize;

// Raw GPIO numbers. This write-only display does not need an SPI MISO pin.
constexpr int BUTTON_EXPLOSION = sand::config::kButtonExplosionPin;
constexpr int BUTTON_POPULATION = sand::config::kButtonPopulationPin;
constexpr bool usedHardwarePin(int pin) {
  return pin == LCD_CS || pin == LCD_DC || pin == LCD_RST || pin == LCD_SCK ||
         pin == LCD_MOSI || pin == I2C_SDA || pin == I2C_SCL ||
         pin == hardware::kRgbLed || pin == 19 || pin == 20;
}
static_assert(BUTTON_EXPLOSION != BUTTON_POPULATION && !usedHardwarePin(BUTTON_EXPLOSION) &&
              !usedHardwarePin(BUTTON_POPULATION), "Button pin conflicts with LCD, I2C, RGB LED or USB");
static_assert(DISPLAY_SIZE == hardware::kDisplaySize, "Simulation and panel dimensions must match");
static_assert(sand::config::kPhysicsHz > 0 && sand::config::kDisplayHz > 0,
              "Update rates must be positive");
static_assert(sand::config::kRenderBandRows >= 1 && sand::config::kRenderBandRows <= 8,
              "Render bands must be 1..8 tile rows (bounded pixel buffer)");
static_assert(DISPLAY_SIZE == sand::render::FluidRaster::kSize, "Raster and panel dimensions must match");
static_assert(sand::config::kDotRadius * sand::config::kMaxVisualVariation * sand::config::kVisualSizeScale <=
              sand::render::FluidRaster::kMaxDotRadius, "Dot size exceeds renderer capacity");

Adafruit_GC9A01A tft(&SPI, LCD_DC, LCD_CS, LCD_RST);
sand::Simulation simulation;
Preferences preferences;
bool preferencesReady = false;

// Great Wave on boot. A swatch remains locked until randomize or serial p.
uint8_t palette = 1;
uint32_t backgroundSeed = 0x6D726F34u;
uint8_t visualColors[sand::Simulation::kMaxParticleCount] = {};
sand::render::Shape particleShape = sand::render::Shape::Circle;
sand::variation::OddShape visualOddShape;
float visualSizeVariation = 3.0f;
float visualStretchMax = 2.0f;
float spawnFill = 0;
sand::render::SpawnCoverage spawnCoverage; // Physics core only; no per-frame coverage scan.
uint32_t lcdSPIHz = LCD_SPI_HZ;
uint32_t impactSnapshots = 0;
int peakImpactDots = 0;

// The loop owns simulation, Wire, input, settings and Serial. The LCD task
// exclusively owns tft/SPI after setup. Frames contain copies, never pointers
// into live particles. A one-frame mailbox replaces stale pending frames.
struct RenderFrame {
  sand::render::Dot dots[sand::Simulation::kMaxParticleCount];
  int dotCount;
  float dotRadius; // Twice the scaled round physics radius, retaining visual overlap.
  float stretchMax;
  sand::render::Shape shape;
  uint8_t palette;
  uint32_t backgroundSeed;
  uint32_t spiHz;
  uint32_t capturedUs;
};
void renderSand(const RenderFrame &frame); // Keep Arduino's prototype after this type.
struct RenderStats {
  uint32_t frames, changedFrames, totalUs, maxUs, pixels, windows;
  uint32_t ageTotalUs, maxAgeUs;
  uint32_t backgroundFields, backgroundTiles;
};
QueueHandle_t renderQueue = nullptr;
TaskHandle_t renderTaskHandle = nullptr;
portMUX_TYPE renderStatsMux = portMUX_INITIALIZER_UNLOCKED;
RenderStats renderStats = {};
RenderFrame outgoingFrame, drawingFrame;
sand::render::FluidRaster fluidRaster; // LCD task only; allocated in internal SRAM.
sand::render::FluidBackground fluidBackground; // Two small color grids; LCD core only.
uint16_t pixelStrip[DISPLAY_SIZE * TILE_SIZE * sand::config::kRenderBandRows];
// Preserve the decorative rim even when merged rectangles include blank
// exterior cells. A 1-bit background uses 7.2KB, rather than a full RGB frame.
GFXcanvas1 rimMask(DISPLAY_SIZE, DISPLAY_SIZE);
input::ButtonInput explosionButton(sand::config::kButtonDebounceMs);
input::ButtonInput populationButton(sand::config::kButtonDebounceMs);
uint32_t explosionButtonEdges = 0, populationButtonEdges = 0;
uint32_t explosionButtonPresses = 0, populationButtonPresses = 0;

// This sketch uses neither ADC nor radio. Enable the internal ADC noise source
// only while drawing button choices, then release it. RF/ADC features added
// later must coordinate this source (see Espressif's RNG documentation).
class RandomEntropyScope {
public:
  RandomEntropyScope() { bootloader_random_enable(); }
  ~RandomEntropyScope() { bootloader_random_disable(); }
  RandomEntropyScope(const RandomEntropyScope &) = delete;
  RandomEntropyScope &operator=(const RandomEntropyScope &) = delete;
};

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

motion::MPU6050Motion motionSensor(Wire, hardware::kImuCompatibilityId);
motion::ShakeInput shakeInput;
motion::MotionHealth motionHealth;
bool shakeEnabled = true;
float shakeGain = sand::config::kShakeGain, peakShakeImpulse = 0, peakShoveImpulse = 0;
uint32_t lastShakeSampleUs = 0, shakePulses = 0;
uint8_t imuAddress = 0;
bool imuOnline = false;
bool swapAxes = hardware::kImuSwapAxes;
bool invertX = hardware::kImuFlipX, invertY = hardware::kImuFlipY;
float accelX = 0, accelY = 0, accelZ = 0;
float filteredX = 0, filteredY = 0;
float gravityX = 0, gravityY = 1;
uint32_t imuErrors = 0;
int gravityMode = 0; // 0: automatic, 1..4: down/left/up/right, 5: demo.

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
    if (imuOnline) logMessage("IMU connected 0x%02X ID=0x%02X compat=%d; requested 200Hz/94Hz filter, tau8ms, calibration=%s.\n",
      imuAddress, motionSensor.identity, motionSensor.compatibilityMode(), motionSensor.calibration.valid ? "SAVED" : "NONE");
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
  const bool fault = motionHealth.update(micros(), motionSensor.errors, motionSensor.online,
                                        motionSensor.motionReady, fresh);
  if (fault) {
    shakeInput.reset();
    lastShakeSampleUs = 0;
  }
  if (fresh && motionSensor.online && motionSensor.motionReady) {
    uint32_t sampleUs = motionSensor.lastSampleUs;
    float dt = lastShakeSampleUs ? (uint32_t)(sampleUs - lastShakeSampleUs) * 0.000001f : 0.005f;
    shakeInput.update(motionSensor.motionAccel, motionSensor.motionGyro, dt);
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
    float angle = millis() * 0.0018f;
    gravityX = sinf(angle); gravityY = cosf(angle);
  }
}

void assignVisualColors(uint32_t seed) {
  uint32_t state = seed ? seed : 1;
  auto random = [&state]() -> uint32_t {
    state ^= state << 13;
    state ^= state >> 17;
    state ^= state << 5;
    return state;
  };
  sand::variation::assignColorSlots(visualColors, simulation.particleCount(), random);
}

void logSwatch() {
  const sand::palette::Swatch &swatch = sand::palette::get(palette);
  logMessage("Swatch=%u/%d id=%s name=\"%s\" foreground=5 background=3 animation=gradient.\n",
             palette + 1, sand::palette::kCount, swatch.id, swatch.name);
  logMessage("Background seed=%lu layers=3 motion=8..18px/s; wandering shapes, saturation and intensity pulses.\n",
             (unsigned long)backgroundSeed);
}

void randomizeBackground() {
  {
    RandomEntropyScope entropy;
    backgroundSeed = esp_random();
  }
  logSwatch();
}

void logDeformation() {
  logMessage("Deformation stretchMax=%.3fx squashMin=%.3fx (velocity-aligned).\n",
             visualStretchMax, 1.0f / visualStretchMax);
}

void logOddShape() {
  const int index = visualOddShape.index(simulation.particleCount());
  if (index >= 0 && visualOddShape.shape != particleShape)
    logMessage("Odd count=1 index=%d shape=%s; exactly one on every Randomize.\n",
               index, sand::render::shapeName(visualOddShape.shape));
  else logMessage("Odd count=0; no alternate shape configured.\n");
}

void randomizeStretch() {
  {
    RandomEntropyScope entropy;
    auto random = []() -> uint32_t { return esp_random(); };
    visualStretchMax = sand::variation::sampleStretch(random);
  }
  logDeformation();
}

void resetSand() {
  const uint32_t seed = esp_random();
  simulation.reset(seed);
  assignVisualColors(seed);
  spawnFill = spawnCoverage.measure(simulation.particleCount(),
    simulation.dotRadius() * sand::config::kVisualSizeScale, particleShape,
    [](int index) -> const sand::Particle & { return simulation.particle(index); },
    visualOddShape.index(simulation.particleCount()), visualOddShape.shape);
  shakeInput.consume();
  logMessage("Reset %d fluid dots at %.1f%% visual size.\n", simulation.particleCount(), visualSizeVariation * 100);
}

void explodeSand() {
  const sand::cluster::Point origin = sand::cluster::explosionOrigin(
    simulation.particleCount(), simulation.dotRadius(),
    [](int index) -> const sand::Particle & { return simulation.particle(index); });
  float strength;
  {
    RandomEntropyScope entropy;
    auto random = []() -> uint32_t { return esp_random(); };
    strength = sand::variation::sampleExplosionStrength(random);
  }
  simulation.explode(origin.x, origin.y, strength);
  float peakSpeedSquared = 0;
  for (int i = 0; i < simulation.particleCount(); ++i) {
    const sand::Particle &particle = simulation.particle(i);
    const float speedSquared = particle.vx * particle.vx + particle.vy * particle.vy;
    if (speedSquared > peakSpeedSquared) peakSpeedSquared = speedSquared;
  }
  logMessage("Explosion cluster=(%.1f,%.1f) impulse=%.1fpx/s strength=%.1f%% peak=%.1fpx/s (100%%=previous maximum).\n",
             origin.x, origin.y, strength, strength * 100 / sand::config::kExplosionMinImpulse,
             sqrtf(peakSpeedSquared));
}

void finishPopulation(uint32_t seed, sand::render::Shape shape) {
  particleShape = sand::render::validShape(shape) ? shape : sand::render::Shape::Circle;
  visualSizeVariation = simulation.sizeScale();
  assignVisualColors(seed);
  shakeInput.consume();
  logMessage("Population size=%.1f%% diameter=%.2fpx dots=%d shape=%s physicsDiameter=%.2fpx spawnFill=%.2f%%.\n",
    visualSizeVariation * 100,
    simulation.dotRadius() * 2 * sand::config::kVisualSizeScale,
    simulation.particleCount(), sand::render::shapeName(particleShape),
    simulation.dotRadius() * 2, spawnFill * 100);
  logOddShape();
}

// Explicit serial previews/stress tests retain the requested size and count.
void setPopulation(float scale, int count, uint32_t seed, sand::render::Shape shape) {
  if (!sand::variation::validVisualScale(scale) || count < sand::palette::kForegroundCount ||
      !simulation.configurePacked(scale, count, seed)) {
    logMessage("Population rejected: scale=%.3f count=%d.\n", scale, count);
    return;
  }
  spawnFill = spawnCoverage.measure(count,
    simulation.dotRadius() * sand::config::kVisualSizeScale, shape,
    [](int index) -> const sand::Particle & { return simulation.particle(index); },
    visualOddShape.index(count), visualOddShape.shape);
  finishPopulation(seed, shape);
}

void fitPopulation(float preferredScale, float targetCoverage, uint32_t seed,
                   sand::render::Shape shape) {
  const uint32_t started = micros();
  const sand::variation::FittedPopulation fitted = sand::variation::fitPopulation(
    simulation, spawnCoverage, preferredScale, targetCoverage, seed, shape, visualOddShape);
  const uint32_t elapsed = micros() - started;
  spawnFill = fitted.coverage;
  finishPopulation(seed, shape);
  logMessage("Fill target=%.2f%% checks=%d fit=%.2fms preferredSize=%.1f%%.\n",
    targetCoverage * 100, fitted.measurements,
    elapsed / 1000.0f, preferredScale * 100);
}

void randomizePopulation() {
  sand::variation::Population population;
  sand::render::Shape shape;
  uint8_t nextPalette;
  uint32_t seed;
  {
    RandomEntropyScope entropy;
    auto random = []() -> uint32_t { return esp_random(); };
    shape = sand::variation::sampleShape(random, particleShape);
    visualOddShape = sand::variation::sampleOddShape(random, shape);
    population = sand::variation::samplePopulation(random);
    visualStretchMax = sand::variation::sampleStretch(random);
    nextPalette = sand::variation::samplePalette(random, palette);
    seed = random();
    backgroundSeed = random();
  }
  fitPopulation(population.scale, population.targetCoverage, seed, shape);
  palette = nextPalette;
  logDeformation();
  logSwatch();
}

uint32_t physicsSteps = 0, physicsTotalUs = 0, maxPhysicsUs = 0;
uint32_t skippedPhysicsSteps = 0, lastImuSamples = 0;

bool backgroundTileVisible(int x, int y) {
  // Skip tiles wholly outside the physical round panel, with a corner margin.
  const int dx = 2 * x + 3 - (DISPLAY_SIZE - 1);
  const int dy = 2 * y + 3 - (DISPLAY_SIZE - 1);
  if (dx * dx + dy * dy > 245 * 245) return false;
  for (int row = y; row < y + TILE_SIZE; ++row) {
    const uint8_t *pixels = fluidRaster.row(row) + x;
    for (int column = 0; column < TILE_SIZE; ++column) {
      const uint8_t color = pixels[column];
      if (!color || (color & 3)) return true; // Empty or antialiased: backdrop is visible.
    }
  }
  return false;
}

void renderSand(const RenderFrame &frame) {
  const uint32_t began = micros();
  static uint32_t previousSPIHz = LCD_SPI_HZ;
  static uint32_t previousDrawUs = 0;
  if (frame.spiHz != previousSPIHz) {
    tft.setSPISpeed(frame.spiHz);
    previousSPIHz = frame.spiHz;
  }
  fluidRaster.draw(frame.dots, frame.dotCount, frame.dotRadius, frame.shape, frame.stretchMax);
  // Bound LCD work rather than freezing motion under a busy foreground.
  const int backgroundBudget = previousDrawUs < 8000 ? 160 :
      (previousDrawUs < 12000 ? 96 : (previousDrawUs < 17000 ? 64 : 32));
  uint32_t backgroundTiles = 0;
  const uint32_t backgroundRevision = fluidBackground.revision();
  fluidBackground.prepare(frame.palette, frame.backgroundSeed, millis(), backgroundBudget,
    backgroundTileVisible, [&](int x, int y, int width, int height) {
      fluidRaster.invalidate(x, y, width, height);
      backgroundTiles += width * height / (TILE_SIZE * TILE_SIZE);
    });
  bool transaction = false;
  uint32_t pixels = 0, windows = 0;
  const uint8_t *rim = rimMask.getBuffer();
  const uint16_t rimColor = fluidBackground.rimColor();
  const sand::render::Options options(sand::config::kRenderMergeGapCells,
    sand::config::kRenderBandRows, sand::config::kRenderMergeOverdrawCells);
  fluidRaster.planPixels(options,
    [&](const sand::render::PixelRect &rect) {
      const int left = rect.x, top = rect.y;
      const int width = rect.w, height = rect.h;
      for (int y = 0; y < height; ++y) {
        const int pixelY = top + y;
        const uint8_t *row = fluidRaster.row(pixelY) + left;
        const uint8_t *rimRow = rim ? rim + pixelY * ((DISPLAY_SIZE + 7) / 8) : nullptr;
        for (int x = 0; x < width; ++x) {
          const int pixelX = left + x;
          const uint8_t color = row[x];
          const bool onRim = !color && rimRow &&
            (rimRow[pixelX / 8] & (0x80 >> (pixelX & 7)));
          pixelStrip[y * width + x] = onRim ? rimColor : fluidBackground.composite(color, pixelX, pixelY);
        }
      }
      if (!transaction) { tft.startWrite(); transaction = true; }
      tft.setAddrWindow(left, top, width, height);
      tft.writePixels(pixelStrip, width * height, true, false);
      pixels += width * height;
      ++windows;
    });
  if (transaction) tft.endWrite();
  fluidRaster.commit();
  const uint32_t duration = micros() - began;
  previousDrawUs = duration;
  const uint32_t age = micros() - frame.capturedUs;
  portENTER_CRITICAL(&renderStatsMux);
  ++renderStats.frames;
  if (transaction) ++renderStats.changedFrames;
  renderStats.totalUs += duration;
  if (duration > renderStats.maxUs) renderStats.maxUs = duration;
  renderStats.pixels += pixels;
  renderStats.windows += windows;
  renderStats.ageTotalUs += age;
  renderStats.backgroundFields += fluidBackground.revision() != backgroundRevision;
  renderStats.backgroundTiles += backgroundTiles;
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
  fluidRaster.setClipCircle(sand::config::kCircleRadius + 0.5f);
  renderQueue = xQueueCreate(1, sizeof(RenderFrame));
#if CONFIG_FREERTOS_UNICORE
  const BaseType_t displayCore = 0;
#else
  const BaseType_t displayCore = 1 - xPortGetCoreID();
#endif
  if (renderQueue && xTaskCreatePinnedToCore(lcdTask, "fluidLCD",
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
  outgoingFrame.dotCount = simulation.particleCount();
  outgoingFrame.dotRadius = simulation.dotRadius() * sand::config::kVisualSizeScale;
  outgoingFrame.stretchMax = visualStretchMax;
  outgoingFrame.shape = particleShape;
  const int oddIndex = visualOddShape.index(outgoingFrame.dotCount);
  int impactDots = 0;
  for (int i = 0; i < outgoingFrame.dotCount; ++i) {
    const sand::Particle &particle = simulation.particle(i);
    outgoingFrame.dots[i] = {particle.x, particle.y, visualColors[i], particle.vx, particle.vy};
    sand::render::Dot &dot = outgoingFrame.dots[i];
    if (i == oddIndex) dot.shape = visualOddShape.shape;
    const sand::impact::Pulse &impact = simulation.impact(i);
    dot.impactWeight = impact.weight();
    if (dot.impactWeight > 0) {
      dot.impactX = impact.normalX;
      dot.impactY = impact.normalY;
      dot.impactScale = impact.scale();
      ++impactDots;
    }
  }
  if (impactDots) ++impactSnapshots;
  if (impactDots > peakImpactDots) peakImpactDots = impactDots;
  outgoingFrame.palette = palette;
  outgoingFrame.backgroundSeed = backgroundSeed;
  outgoingFrame.spiHz = lcdSPIHz;
  outgoingFrame.capturedUs = micros();
  if (renderQueue) xQueueOverwrite(renderQueue, &outgoingFrame);
  else renderSand(outgoingFrame);
}

void cyclePalette() {
  palette = (palette + 1) % sand::palette::kCount;
  randomizeBackground();
}

void sampleButtons() {
  if (!sand::config::kButtonsEnabled) return;
  const uint32_t now = millis();
  const bool explosionDown = digitalRead(BUTTON_EXPLOSION) == LOW;
  const bool populationDown = digitalRead(BUTTON_POPULATION) == LOW;
  static bool initialized = false, previousExplosion = false, previousPopulation = false;
  if (initialized) {
    if (explosionDown != previousExplosion) ++explosionButtonEdges;
    if (populationDown != previousPopulation) ++populationButtonEdges;
  }
  previousExplosion = explosionDown; previousPopulation = populationDown;
  initialized = true;
  // Sample/debounce both inputs before either action changes the fluid.
  const bool explode = explosionButton.update(explosionDown, now);
  const bool resize = populationButton.update(populationDown, now);
  if (explode) {
    ++explosionButtonPresses;
    logMessage("BUTTON GPIO%d explosion press #%lu.\n", BUTTON_EXPLOSION, (unsigned long)explosionButtonPresses);
    explodeSand();
  }
  if (resize) {
    ++populationButtonPresses;
    logMessage("BUTTON GPIO%d randomize press #%lu.\n", BUTTON_POPULATION, (unsigned long)populationButtonPresses);
    randomizePopulation();
  }
}

void diagnoseButtons() {
  const int pins[] = {BUTTON_EXPLOSION, BUTTON_POPULATION};
  for (int pin : pins) {
    gpio_io_config_t config = {};
    const esp_err_t result = gpio_get_io_config(static_cast<gpio_num_t>(pin), &config);
    logMessage("BUTTON CONFIG GPIO%d level=%s input=%d output=%d pullup=%d pulldown=%d mux=%lu read=%d\n",
      pin, digitalRead(pin) == LOW ? "LOW" : "HIGH", config.ie, config.oe,
      config.pu, config.pd, (unsigned long)config.fun_sel, result);
  }
}

void showHelp() {
  logMessage("Fluid: e=100%%-200%% explosion, n=randomize (exactly one odd shape), r=reset current dots.\n");
  logMessage("p=cycle 16 locked swatches; g=randomize gradient only. Three wandering shapes pulse in saturation/intensity.\n");
  logMessage("c=cycle circle/triangle/square/star; physics stays round. Stretch randomizes 1x-3x; wall impacts squash then rebound.\n");
  logMessage("5..9=100/162.5/225/287.5/350%% size; !=dense fill at current size.\n");
  const float baselineDiameter = 2 * sand::config::kDotRadius * sand::config::kVisualSizeScale;
  logMessage("Original visual baseline=%.1fpx; random sizes=%.1f..%.1fpx. Physics spacing grows with size; 2x overlap.\n",
    baselineDiameter, baselineDiameter * sand::config::kMinVisualVariation,
    baselineDiameter * sand::config::kMaxVisualVariation);
  logMessage("Randomize: uniform 100-350%% sizes, roughly 15-35%% fill favoring the lower end. No crowded rolls.\n");
  logMessage("d=swirling demo, 0=live IMU.\n");
  logMessage("1=down, 2=left, 3=up, 4=right (manual gravity).\n");
  logMessage("Mount mapping: s=swap axes, x=flip X, y=flip Y (saved). v=%lu/8MHz LCD.\n",
             (unsigned long)(LCD_SPI_HZ / 1000000));
  logMessage("No on-screen IMU indicators. Diagnostics are serial only. ?=help, i=scan, m=MPU diagnostic.\n");
  logMessage("Shake: h=on/off, +/-=strength, t=test splash. Tilt drives fluid pressure and flow.\n");
  logMessage("b=toggle 100/400kHz IMU bus; m=motion diagnostics.\n");
  logMessage("k=button pin diagnostics; BUTTONS telemetry counts physical edges and accepted presses.\n");
  logMessage("Targets: physics=%luHz, snapshots=%luHz; PERF reports actual rates.\n",
    (unsigned long)sand::config::kPhysicsHz, (unsigned long)sand::config::kDisplayHz);
  logMessage("Buttons: GPIO%d=explosion, GPIO%d=randomize; normally open to GND. Tune SandConfig.h.\n",
             BUTTON_EXPLOSION, BUTTON_POPULATION);
}

void diagnoseIMU() {
  const bool savedMapping = preferencesReady && preferences.isKey("swap") &&
    preferences.isKey("flipX") && preferences.isKey("flipY") &&
    preferences.getBool("swap") == swapAxes &&
    preferences.getBool("flipX") == invertX && preferences.getBool("flipY") == invertY;
  logMessage("MOUNT swap=%d flipX=%d flipY=%d saved=%d\n",
             swapAxes, invertX, invertY, savedMapping);
  logMessage("Shake %s gain=%.2f transient=(%.3f,%.3f,%.3f)g activity=%.3f.\n",
    shakeEnabled ? "ON" : "OFF", shakeGain,
    shakeInput.latestLinear.x, shakeInput.latestLinear.y, shakeInput.latestLinear.z, shakeInput.activity);
  logMessage("Motion mode=FAST, 200Hz sensor, 94Hz filter, tau8ms; calibration=%s.\n",
    motionSensor.calibration.valid ? "SAVED" : "NONE");
  logMessage("Saved accel bias=(%.5f,%.5f,%.5f)g. Calibrate using MPU_Visualizer.\n",
    motionSensor.calibration.accelBias.x, motionSensor.calibration.accelBias.y, motionSensor.calibration.accelBias.z);
  logMessage("Gyro raw=(%.2f,%.2f,%.2f)dps qualified=(%.2f,%.2f,%.2f)dps I2C=%luHz rejected=%lu ready=%d\n",
    motionSensor.rawGyro.x, motionSensor.rawGyro.y, motionSensor.rawGyro.z,
    motionSensor.motionGyro.x, motionSensor.motionGyro.y, motionSensor.motionGyro.z,
    (unsigned long)Wire.getClock(), (unsigned long)motionSensor.rejectedSamples, motionSensor.motionReady);
  if (motionSensor.rejectedSamples) {
    const uint8_t *packet = motionSensor.lastRejectedPacket;
    logMessage("Last rejected reason=0x%02X packet=%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
      motionSensor.lastRejectReason, packet[0], packet[1], packet[2], packet[3], packet[4],
      packet[5], packet[6], packet[7], packet[8], packet[9], packet[10], packet[11],
      packet[12], packet[13], packet[14]);
  }
  logMessage("GY-521 wiring: SDA=GPIO%d, SCL=GPIO%d, VCC=3V3, GND=GND.\n", I2C_SDA, I2C_SCL);
  logMessage("LCD SDA/SCL are separate SPI signals. IMU I2C idle SDA=%d SCL=%d\n",
             digitalRead(I2C_SDA), digitalRead(I2C_SCL));
  const uint8_t addresses[] = {0x68, 0x69};
  for (uint8_t address : addresses) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission(true);
    logMessage("Probe 0x%02X: %s (I2C error=%u)\n", address,
               error == 0 ? "ACK" : "NO ACK", error);
    if (error) continue;
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
  logMessage("I2C scan: SDA=GPIO%d, SCL=GPIO%d.\n", I2C_SDA, I2C_SCL);
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
    else if (command == 'k') diagnoseButtons();
    else if (command == 'r') resetSand();
    else if (command == 'e') explodeSand();
    else if (command == 'n') randomizePopulation();
    else if (command == 'c') {
      particleShape = static_cast<sand::render::Shape>(
          (static_cast<uint8_t>(particleShape) + 1) % static_cast<uint8_t>(sand::render::Shape::Count));
      if (sand::render::validShape(visualOddShape.shape))
        visualOddShape.shape = static_cast<sand::render::Shape>(
            (static_cast<uint8_t>(visualOddShape.shape) + 1) % static_cast<uint8_t>(sand::render::Shape::Count));
      logMessage("Shape=%s.\n", sand::render::shapeName(particleShape));
      logOddShape();
    }
    else if (command >= '5' && command <= '9') {
      const float scale = sand::config::kMinVisualVariation +
          (sand::config::kMaxVisualVariation - sand::config::kMinVisualVariation) *
          ((command - '5') * 0.25f);
      setPopulation(scale, sand::variation::visualCountRange(scale, particleShape).baseline, esp_random(), particleShape);
    } else if (command == '!') {
      setPopulation(visualSizeVariation, sand::Simulation::maxPackedCountForScale(visualSizeVariation),
                    esp_random(), particleShape);
    }
    else if (command == 'h') {
      shakeEnabled = !shakeEnabled;
      shakeInput.consume();
      logMessage("Shake %s, gain=%.2f.\n", shakeEnabled ? "ON" : "OFF", shakeGain);
    } else if (command == '+' || command == '-') {
      shakeGain = constrain(shakeGain + (command == '+' ? 0.25f : -0.25f), 0.25f, 3.0f);
      logMessage("Shake strength=%.2f.\n", shakeGain);
    } else if (command == 't') {
      float impulse = min(sand::config::kMaxKick, 150.0f * shakeGain);
      simulation.kick(0, -impulse, impulse * 0.3f);
      peakShakeImpulse = max(peakShakeImpulse, impulse);
      ++shakePulses;
      logMessage("Test upward splash.\n");
    }
    else if (command == 'p') cyclePalette();
    else if (command == 'g') randomizeBackground();
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
      lcdSPIHz = lcdSPIHz == LCD_SPI_HZ ? 8000000 : LCD_SPI_HZ;
      logMessage("LCD SPI = %lu MHz.\n", (unsigned long)(lcdSPIHz / 1000000));
    } else if (command == 'b') {
      const uint32_t rate = Wire.getClock() == 100000 ? 400000 : 100000;
      const bool changed = Wire.setClock(rate);
      shakeInput.reset();
      lastShakeSampleUs = 0;
      logMessage("IMU I2C=%luHz change=%s.\n", (unsigned long)Wire.getClock(), changed ? "OK" : "FAILED");
    } else if (command == 'i') scanI2C();
    else if (command == 'm') diagnoseIMU();
  }
}

uint32_t lastStepUs = 0, lastFrameUs = 0, lastStatsMs = 0;

void publishDueFrame(uint32_t nowUs) {
  if ((uint32_t)(nowUs - lastFrameUs) < FRAME_US) return;
  // Deadlines retain a 60Hz cadence through scheduler jitter. Publish between
  // catch-up steps too, so a busy physics core cannot hold the LCD's next frame
  // behind an entire batch of overdue simulation work.
  lastFrameUs += FRAME_US;
  if ((uint32_t)(nowUs - lastFrameUs) >= FRAME_US) lastFrameUs = nowUs;
  publishFrame();
}

void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(1);
  preferencesReady = preferences.begin("gravity-sand", false);
  if (preferencesReady) {
    swapAxes = preferences.getBool("swap", hardware::kImuSwapAxes);
    invertX = preferences.getBool("flipX", hardware::kImuFlipX);
    invertY = preferences.getBool("flipY", hardware::kImuFlipY);
  }
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(sand::config::kImuI2cHz);
  Wire.setTimeOut(10);
  SPI.begin(LCD_SCK, -1, LCD_MOSI, LCD_CS);
  tft.begin(LCD_SPI_HZ);
  tft.setRotation(0);
  tft.fillScreen(sand::palette::get(palette).background[0]);
  tft.drawCircle(119, 119, 118, sand::palette::get(palette).background[2]);
  if (rimMask.getBuffer()) rimMask.drawCircle(119, 119, 118, 1);
  if (sand::config::kButtonsEnabled) {
    pinMode(BUTTON_EXPLOSION, INPUT_PULLUP);
    pinMode(BUTTON_POPULATION, INPUT_PULLUP);
    sampleButtons(); // Seed held-at-boot states before any startup delays.
  }
  if (!beginIMU()) logMessage("Waiting for MPU6050 at 0x68/0x69; using screen-bottom gravity.\n");
  {
    RandomEntropyScope entropy;
    auto random = []() -> uint32_t { return esp_random(); };
    visualOddShape = sand::variation::sampleOddShape(random, particleShape);
  }
  fitPopulation(visualSizeVariation, 0.22f, esp_random(), particleShape);
  randomizeStretch();
  randomizeBackground();
  showHelp();
  beginRenderer(); // All subsequent LCD access belongs to the renderer.
  publishFrame();
  lastStepUs = lastFrameUs = micros();
  lastStatsMs = millis();
}

void loop() {
  handleSerial();
  sampleIMU();
  sampleButtons();
  updateGravity();
  uint32_t nowUs = micros();
  int steps = 0;
  while ((uint32_t)(nowUs - lastStepUs) >= STEP_US && steps < sand::config::kMaxCatchUpSteps) {
    uint32_t began = micros();
    motion::ShakePulse pulse = shakeInput.consume();
    if (shakeEnabled && imuOnline && gravityMode == 0 && motionHealth.trusted(micros())) {
      float impulse = min(sand::config::kMaxKick,
        pulse.agitation * shakeGain * sand::config::kShakePixelScale);
      motion::Vec2 shove = motion::mapGravity(pulse.linearImpulseGSeconds, swapAxes, invertX, invertY);
      // The shared detector's agitation uses legacy 3px cells; convert above.
      // Directional acceleration already uses the fluid's pixel/s^2 scale.
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
    nowUs = micros();
    publishDueFrame(nowUs);
    nowUs = micros();
  }
  nowUs = micros();
  if ((uint32_t)(nowUs - lastStepUs) >= STEP_US) {
    skippedPhysicsSteps += (uint32_t)(nowUs - lastStepUs) / STEP_US;
    lastStepUs = nowUs;
  }
  nowUs = micros();
  publishDueFrame(nowUs);
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
    logMessage("PERF dots=%d size=%.1f%% SPI=%luMHz fps=%.1f changed=%.1f drawAvg=%luus drawMax=%luus stepAvg=%luus stepMax=%luus px/frame=%lu win/frame=%lu shape=%s\n",
      simulation.particleCount(), visualSizeVariation * 100, (unsigned long)(lcdSPIHz / 1000000),
      drawing.frames * 1000.0f / elapsed, drawing.changedFrames * 1000.0f / elapsed,
      (unsigned long)(drawing.frames ? drawing.totalUs / drawing.frames : 0),
      (unsigned long)drawing.maxUs,
      (unsigned long)(physicsSteps ? physicsTotalUs / physicsSteps : 0),
      (unsigned long)maxPhysicsUs,
      (unsigned long)(drawing.frames ? drawing.pixels / drawing.frames : 0),
      (unsigned long)(drawing.frames ? drawing.windows / drawing.frames : 0), sand::render::shapeName(particleShape));
    logMessage("DISPLAY snapshot-to-draw avg=%luus max=%luus; input/physics core=%d stretchMax=%.3fx impactSnapshots=%lu peakImpactDots=%d oddDots=%d.\n",
      (unsigned long)(drawing.frames ? drawing.ageTotalUs / drawing.frames : 0),
      (unsigned long)drawing.maxAgeUs, xPortGetCoreID(), visualStretchMax,
      (unsigned long)impactSnapshots, peakImpactDots,
      visualOddShape.index(simulation.particleCount()) >= 0 && visualOddShape.shape != particleShape ? 1 : 0);
    impactSnapshots = 0;
    peakImpactDots = 0;
    logMessage("BACKGROUND fields=%.1fHz visibleTiles/frame=%lu seed=%lu.\n",
      drawing.backgroundFields * 1000.0f / elapsed,
      (unsigned long)(drawing.frames ? drawing.backgroundTiles / drawing.frames : 0),
      (unsigned long)backgroundSeed);
    logMessage("RATES physics=%.1fHz target=%luHz IMU=%.1fHz snapshotTarget=%luHz skipped=%lu\n",
      physicsSteps * 1000.0f / elapsed, (unsigned long)sand::config::kPhysicsHz,
      (uint32_t)(motionSensor.samples - lastImuSamples) * 1000.0f / elapsed,
      (unsigned long)sand::config::kDisplayHz, (unsigned long)skippedPhysicsSteps);
    logMessage("SHAKE on=%d gain=%.2f transient=(%.3f,%.3f,%.3f)g activity=%.3f pulses=%lu peak=%.2f shove=%.2f\n",
      shakeEnabled, shakeGain, shakeInput.latestLinear.x, shakeInput.latestLinear.y,
      shakeInput.latestLinear.z, shakeInput.activity, (unsigned long)shakePulses, peakShakeImpulse, peakShoveImpulse);
    float speedSquared = 0, maximumSpeedSquared = 0, centerX = 0, centerY = 0;
    for (int i = 0; i < simulation.particleCount(); ++i) {
      const sand::Particle &p = simulation.particle(i);
      const float squared = p.vx * p.vx + p.vy * p.vy;
      speedSquared += squared;
      if (squared > maximumSpeedSquared) maximumSpeedSquared = squared;
      centerX += p.x; centerY += p.y;
    }
    logMessage("FLUID rms=%.2fpx/s max=%.2fpx/s center=(%.2f,%.2f)\n",
      sqrtf(speedSquared / simulation.particleCount()), sqrtf(maximumSpeedSquared),
      centerX / simulation.particleCount(), centerY / simulation.particleCount());
    logMessage("SENSOR gyro=(%.2f,%.2f,%.2f)dps I2C=%luHz rejected=%lu ready=%d shakeTrusted=%d\n",
      motionSensor.motionGyro.x, motionSensor.motionGyro.y, motionSensor.motionGyro.z,
      (unsigned long)Wire.getClock(), (unsigned long)motionSensor.rejectedSamples, motionSensor.motionReady,
      motionHealth.trusted(micros()));
    logMessage("BUTTONS GPIO%d=%s edges=%lu presses=%lu GPIO%d=%s edges=%lu presses=%lu\n",
      BUTTON_EXPLOSION, digitalRead(BUTTON_EXPLOSION) == LOW ? "LOW" : "HIGH",
      (unsigned long)explosionButtonEdges, (unsigned long)explosionButtonPresses,
      BUTTON_POPULATION, digitalRead(BUTTON_POPULATION) == LOW ? "LOW" : "HIGH",
      (unsigned long)populationButtonEdges, (unsigned long)populationButtonPresses);
    lastStatsMs = now;
    physicsSteps = physicsTotalUs = maxPhysicsUs = 0;
    skippedPhysicsSteps = 0;
    lastImuSamples = motionSensor.samples;
    peakShakeImpulse = peakShoveImpulse = 0; shakePulses = 0;
  }
  pumpLogs();
  delay(sand::config::kLoopDelayMs);
}
