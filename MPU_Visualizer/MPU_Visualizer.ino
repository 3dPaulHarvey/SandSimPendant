/*
  Super Mini + GC9A01 + GY-521 visual diagnostic.
  Pins live in shared/BoardConfig.h. Optional B1 calibrates; B2 changes mode.
  USB controls work without buttons. Calibration is shared with Gravity_Sand.
*/
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_GC9A01A.h>
#include <Preferences.h>
#include <stdarg.h>
#include "../shared/BoardConfig.h"
#include "../shared/ButtonInput.h"
#include "../shared/MPU6050Motion.h"

constexpr uint16_t BLACK = 0, WHITE = 0xFFFF, DIM = 0x2945;
constexpr uint16_t CYAN = 0x07FF, GOLD = 0xFEA0, GREEN = 0x07E0, RED = 0xF800;
constexpr int SCOPE_X = 28, SCOPE_Y = 44, SCOPE_W = 184, SCOPE_H = 114;
constexpr uint32_t FRAME_US = 33333;

Adafruit_GC9A01A tft(&SPI, hardware::kLcdDC, hardware::kLcdCS, hardware::kLcdReset);
GFXcanvas16 scope(SCOPE_W, SCOPE_H);
GFXcanvas16 numbers(184, 42);
input::ButtonInput calibrateButton, modeButton;
motion::MPU6050Motion imu(Wire, hardware::kImuCompatibilityId);
motion::StationaryCalibrator calibrator(400);
Preferences mountSettings;
bool mountReady = false;
bool swapAxes = hardware::kImuSwapAxes;
bool invertX = hardware::kImuFlipX, invertY = hardware::kImuFlipY;
bool i2cReady = false;

// Bound USB output so an absent or slow monitor cannot block the display.
char logQueue[16][320];
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
  if (!Serial) { logHead = logTail = 0; logOffset = 0; return; }
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

enum class CalStage { Idle, Countdown, Collecting, Success, Failed };
CalStage calStage = CalStage::Idle;
motion::ResponseMode previousMode = motion::Fast;
uint32_t calStageMs = 0, calLastSampleUs = 0;
char calError[40] = "";
bool previousOnline = false;
uint32_t lastFreshUs = 0;
uint32_t lastFrameUs = 0, lastStatsMs = 0;
uint32_t frameCount = 0, lastSampleCount = 0;
float actualHz = 0, actualFPS = 0;

bool calibrationActive() {
  return calStage == CalStage::Countdown || calStage == CalStage::Collecting;
}
bool readingFresh() {
  return imu.online && lastFreshUs != 0 && (uint32_t)(micros() - lastFreshUs) < 150000;
}
const char *sensorState() {
  return !imu.online ? "OFFLINE" : readingFresh() ? "LIVE" : "WAITING";
}
const char *modeName() {
  if (imu.mode == motion::Raw) return "RAW";
  return "FAST";
}
const char *calibrationError(motion::CalibrationError error) {
  using motion::CalibrationError;
  switch (error) {
    case CalibrationError::TooFewSamples: return "Not enough readings";
    case CalibrationError::InvalidSample: return "Invalid sensor reading";
    case CalibrationError::AccelMoving: return "MPU moved: try again";
    case CalibrationError::GyroMoving: return "MPU rotated: try again";
    case CalibrationError::GyroTooFast: return "Rotation too large";
    case CalibrationError::GravityMagnitude: return "Need stationary 1g";
    case CalibrationError::NotFlat: return "Place flat, CHIP UP";
    default: return "Calibration failed";
  }
}
void failCalibration(const char *reason) {
  snprintf(calError, sizeof(calError), "%s", reason);
  calStage = CalStage::Failed;
  calStageMs = millis();
  imu.setMode(previousMode);
  logMessage("CAL FAIL: %s. Previous calibration preserved.\n", calError);
}
void startCalibration() {
  if (calibrationActive()) {
    failCalibration("Cancelled");
    return;
  }
  if (!readingFresh()) { logMessage("CAL: waiting for live GY-521 readings.\n"); return; }
  previousMode = imu.mode;
  if (!imu.setMode(motion::Fast)) { logMessage("CAL: MPU configuration failed.\n"); return; }
  calibrator.reset();
  calStage = CalStage::Countdown;
  calStageMs = millis();
  logMessage("CAL starts in 3s. MPU FLAT, COMPONENT SIDE UP. Do not move it.\n");
}
void serviceMotion() {
  bool fresh = imu.update();
  if (imu.online != previousOnline) {
    previousOnline = imu.online;
    lastFreshUs = 0;
    logMessage("MPU %s address=0x%02X\n", imu.online ? "ONLINE" : "OFFLINE", imu.address);
  }
  if (fresh) lastFreshUs = imu.lastSampleUs;
  if (calStage == CalStage::Collecting && fresh) {
    if ((uint32_t)(imu.lastSampleUs - calLastSampleUs) > 100000) {
      failCalibration("Sensor reading gap");
      return;
    }
    calLastSampleUs = imu.lastSampleUs;
    if (!calibrator.add(imu.rawAccel, imu.rawGyro)) {
      failCalibration("Invalid sensor reading");
    } else if (calibrator.ready()) {
      motion::CalibrationResult result = calibrator.finish();
      if (!result.ok()) {
        failCalibration(calibrationError(result.error));
      } else if (!imu.saveCalibration(result.calibration)) {
        failCalibration("Could not save offsets");
      } else {
        calStage = CalStage::Success;
        calStageMs = millis();
        imu.setMode(previousMode);
        logMessage("CAL SAVED n=%lu accelBias=(%.5f,%.5f,%.5f)g gyroBias=(%.3f,%.3f,%.3f)dps\n",
          (unsigned long)result.sampleCount,
          result.calibration.accelBias.x, result.calibration.accelBias.y, result.calibration.accelBias.z,
          result.calibration.gyroBias.x, result.calibration.gyroBias.y, result.calibration.gyroBias.z);
        logMessage("CAL noise accelStd=(%.5f,%.5f,%.5f)g gyroStd=(%.3f,%.3f,%.3f)dps; expected flat accel=(0,0,+1)g.\n",
          result.accelStd.x, result.accelStd.y, result.accelStd.z,
          result.gyroStd.x, result.gyroStd.y, result.gyroStd.z);
      }
    }
  }
}
void updateCalibration() {
  uint32_t elapsed = millis() - calStageMs;
  if (calibrationActive() && !imu.online) { failCalibration("MPU disconnected"); return; }
  if (calStage == CalStage::Countdown && elapsed >= 3000) {
    calibrator.reset();
    calStage = CalStage::Collecting;
    calStageMs = millis();
    calLastSampleUs = micros();
  } else if (calStage == CalStage::Collecting) {
    if ((uint32_t)(micros() - calLastSampleUs) > 100000) failCalibration("Sensor reading gap");
    else if (elapsed > 8000) failCalibration("Sensor capture timed out");
  } else if ((calStage == CalStage::Success || calStage == CalStage::Failed) && elapsed > 5000) {
    calStage = CalStage::Idle;
  }
}

void centerText(Adafruit_GFX &target, int cx, int y, const char *text, uint16_t color, int size = 1) {
  target.setTextSize(size);
  target.setTextWrap(false);
  target.setTextColor(color, BLACK);
  target.setCursor(cx - (int)strlen(text) * 3 * size, y);
  target.print(text);
}
void sendCanvas(GFXcanvas16 &canvas, int x, int y) {
  // Small SPI transfers keep fresh IMU samples flowing during repaints.
  tft.startWrite();
  for (int row = 0; row < canvas.height(); row += 6) {
    const int height = min(6, canvas.height() - row);
    tft.setAddrWindow(x, y + row, canvas.width(), height);
    tft.writePixels(canvas.getBuffer() + row * canvas.width(), canvas.width() * height, true, false);
    serviceMotion();
    pumpLogs();
  }
  tft.endWrite();
}
void drawFrame() {
  char line[64];
  tft.fillRect(36, 32, 168, 9, BLACK);
  if (imu.compatibilityMode()) {
    snprintf(line, sizeof(line), "%s ID%02X COMPAT %s", sensorState(), imu.identity, modeName());
  } else {
    snprintf(line, sizeof(line), "%s %s 0x%02X CAL:%s", sensorState(), modeName(), imu.address,
             imu.calibration.valid ? "OK" : "--");
  }
  centerText(tft, 120, 32, line, readingFresh() ? GREEN : RED);

  scope.fillScreen(BLACK);
  constexpr int cx = SCOPE_W / 2, cy = 51, radius = 46;
  if (calibrationActive()) {
    centerText(scope, cx, 18, "GY-521 FLAT / CHIP UP", WHITE);
    if (calStage == CalStage::Countdown) {
      snprintf(line, sizeof(line), "STARTING IN %lu", (unsigned long)(3 - min((uint32_t)3, (millis() - calStageMs) / 1000)));
      centerText(scope, cx, 43, line, GOLD);
    } else {
      snprintf(line, sizeof(line), "%lu / 400 readings", (unsigned long)calibrator.count());
      centerText(scope, cx, 43, line, GOLD);
      scope.drawRect(16, 63, 152, 8, DIM);
      scope.fillRect(17, 64, calibrator.count() * 150 / 400, 6, GOLD);
    }
    centerText(scope, cx, 83, "Keep still; B1 cancels", WHITE);
  } else if (calStage == CalStage::Success || calStage == CalStage::Failed) {
    centerText(scope, cx, 27, calStage == CalStage::Success ? "CAL SAVED" : "TRY AGAIN",
               calStage == CalStage::Success ? GREEN : RED, 2);
    centerText(scope, cx, 54, calStage == CalStage::Success ? "Now tilt the GY-521" : calError, WHITE);
    centerText(scope, cx, 76, calStage == CalStage::Success ? "Shared with sand" : "Old offsets kept", DIM);
  } else if (!readingFresh()) {
    centerText(scope, cx, 20, imu.online ? "WAITING" : imu.identity ? "IMU REJECTED" : "NO GY-521", RED, 2);
    snprintf(line, sizeof(line), "SDA GPIO%d / SCL GPIO%d", hardware::kI2cSda, hardware::kI2cScl);
    centerText(scope, cx, 49, line, WHITE);
    centerText(scope, cx, 67, "VCC 3V3 / GND to GND", WHITE);
    if (imu.identity) {
      snprintf(line, sizeof(line), "ID 0x%02X; errors %lu", imu.identity, (unsigned long)imu.errors);
      centerText(scope, cx, 88, line, GOLD);
    } else centerText(scope, cx, 88, "Retrying automatically", DIM);
  } else {
    scope.drawCircle(cx, cy, radius, DIM);
    scope.drawCircle(cx, cy, radius / 2, 0x2104);
    scope.drawFastHLine(cx - radius, cy, radius * 2 + 1, DIM);
    scope.drawFastVLine(cx, cy - radius, radius * 2 + 1, DIM);
    const motion::Vec2 input = imu.inputGravity, filtered = imu.gravity;
    const float inputScale = radius / max(1.0f, sqrtf(input.x * input.x + input.y * input.y));
    const float outputScale = radius / max(1.0f, sqrtf(filtered.x * filtered.x + filtered.y * filtered.y));
    const int ix = cx + lroundf(input.x * inputScale), iy = cy + lroundf(input.y * inputScale);
    const int ox = cx + lroundf(filtered.x * outputScale), oy = cy + lroundf(filtered.y * outputScale);
    scope.drawLine(cx, cy, ix, iy, CYAN);
    scope.drawCircle(ix, iy, 6, CYAN);
    scope.fillCircle(ox, oy, 4, GOLD);
    scope.drawPixel(cx, cy, WHITE);
    snprintf(line, sizeof(line), "|A| %.2fg   RING = 1g", motion::norm(imu.accel));
    centerText(scope, cx, 105, line, WHITE);
  }
  sendCanvas(scope, SCOPE_X, SCOPE_Y);

  numbers.fillScreen(BLACK);
  if (readingFresh()) {
    snprintf(line, sizeof(line), "A X%+.2f Y%+.2f Z%+.2f g", imu.accel.x, imu.accel.y, imu.accel.z);
    centerText(numbers, 92, 0, line, WHITE);
    snprintf(line, sizeof(line), "G X%+.0f Y%+.0f Z%+.0f d/s", imu.gyro.x, imu.gyro.y, imu.gyro.z);
    centerText(numbers, 92, 11, line, CYAN);
  } else {
    centerText(numbers, 92, 0, "A X-- Y-- Z-- g", DIM);
    centerText(numbers, 92, 11, "G X-- Y-- Z-- d/s", DIM);
  }
  snprintf(line, sizeof(line), "%.0fHz %.0ffps ERR:%lu", actualHz, actualFPS, (unsigned long)imu.errors);
  centerText(numbers, 92, 24, line, imu.errors ? GOLD : GREEN);
  sendCanvas(numbers, 28, 163);
  ++frameCount;
}

void cycleMode() {
  if (calibrationActive()) return;
  const bool ok = imu.setMode(imu.mode == motion::Fast ? motion::Raw : motion::Fast);
  logMessage("MODE %s %s\n", modeName(), ok ? "OK" : "FAILED");
}
void applyMapping() {
  imu.setMapping(swapAxes, invertX, invertY);
  bool saved = false;
  if (mountReady) {
    saved = mountSettings.putBool("swap", swapAxes) == 1;
    saved = (mountSettings.putBool("flipX", invertX) == 1) && saved;
    saved = (mountSettings.putBool("flipY", invertY) == 1) && saved;
  }
  logMessage("MOUNT swap=%d flipX=%d flipY=%d save=%s\n", swapAxes, invertX, invertY, saved ? "OK" : "FAILED");
}
void showInfo() {
  logMessage("MOUNT swap=%d flipX=%d flipY=%d storage=%s screen=(%.3f,%.3f)g\n",
             swapAxes, invertX, invertY, mountReady ? "READY" : "UNAVAILABLE",
             imu.gravity.x, imu.gravity.y);
  if (readingFresh()) {
    logMessage("IMU_DIAG ms=%lu state=LIVE addr=0x%02X id=0x%02X compat=%d mode=%s cal=%d Hz=%.1f fps=%.1f samples=%lu age_ms=%lu A=(%.3f,%.3f,%.3f)g G=(%.2f,%.2f,%.2f)dps errors=%lu\n",
      (unsigned long)millis(), imu.address, imu.identity, imu.compatibilityMode(), modeName(), imu.calibration.valid, actualHz, actualFPS,
      (unsigned long)imu.samples, (unsigned long)((micros() - lastFreshUs) / 1000),
      imu.accel.x, imu.accel.y, imu.accel.z, imu.gyro.x, imu.gyro.y, imu.gyro.z, (unsigned long)imu.errors);
  } else {
    logMessage("IMU_DIAG ms=%lu state=%s addr=0x%02X id=0x%02X mode=%s cal=%d Hz=%.1f fps=%.1f samples=%lu A=unavailable G=unavailable errors=%lu SDA=%d SCL=%d\n",
      (unsigned long)millis(), sensorState(), imu.address, imu.identity, modeName(), imu.calibration.valid, actualHz,
      actualFPS, (unsigned long)imu.samples, (unsigned long)imu.errors, hardware::kI2cSda, hardware::kI2cScl);
  }
}
void showHelp() {
  logMessage("IMU TEST: c/B1=flat chip-up calibration (3s + 400 fresh samples); q=cancel; m/B2=FAST/RAW.\n");
  logMessage("s=swap XY; x/y=reverse screen axis (saved/shared with sand); z=clear calibration; i=status; ?=help.\n");
  logMessage("Cyan ring/vector=acceleration; gold bubble=filtered output. Flat chip-up: A~(0,0,+1)g.\n");
  logMessage("b=I2C scan/identity at 100/400kHz; 1=use 100kHz; 4=use 400kHz. Scans briefly pause drawing.\n");
  logMessage("d=read configuration and raw sensor registers without accepting an unknown chip ID.\n");
}
void dumpRegisterBlock(uint8_t address, uint8_t reg, uint8_t count) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  const uint8_t status = Wire.endTransmission(false);
  const size_t received = status == 0 ? Wire.requestFrom(address, count) : 0;
  char hex[96] = "";
  size_t used = 0;
  while (Wire.available()) {
    const int value = Wire.read();
    if (used + 4 < sizeof(hex)) used += snprintf(hex + used, sizeof(hex) - used, "%02X ", value);
  }
  logMessage("I2C_DUMP addr=0x%02X reg=0x%02X bytes=%u/%u data=[%s]\n",
             address, reg, (unsigned)received, count, hex);
}
void dumpSensorRegisters() {
  if (calibrationActive()) return;
  const uint8_t addresses[] = {0x68, 0x69};
  for (uint8_t address : addresses) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission(true) != 0) continue;
    dumpRegisterBlock(address, 0x75, 1);
    dumpRegisterBlock(address, 0x19, 4);
    dumpRegisterBlock(address, 0x37, 4);
    dumpRegisterBlock(address, 0x6A, 3);
    for (int sample = 0; sample < 10; ++sample) {
      dumpRegisterBlock(address, 0x3B, 14);
      for (int tick = 0; tick < 50; ++tick) { pumpLogs(); delay(1); }
    }
  }
  lastFreshUs = 0;
  logMessage("I2C_DUMP_DONE (raw counts; configuration and identity unchanged)\n");
}
void probeRegister(uint8_t address, uint8_t reg, bool stopBetween) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  const uint8_t status = Wire.endTransmission(stopBetween);
  const size_t count = status == 0 ? Wire.requestFrom(address, (uint8_t)1) : 0;
  const int value = count == 1 ? Wire.read() : -1;
  while (Wire.available()) Wire.read();
  // In this core a repeated-start pointer write is deferred until requestFrom.
  // Its zero status alone is not evidence that a device acknowledged.
  logMessage("I2C_REG addr=0x%02X reg=0x%02X %s pointer_status=%u bytes=%u value=%d (0x%02X)\n",
             address, reg, stopBetween ? "STOP" : "RESTART", status,
             (unsigned)count, value, value < 0 ? 0 : value);
  pumpLogs();
}
void diagnoseI2c() {
  if (calibrationActive()) { logMessage("I2C: cancel calibration before scanning.\n"); return; }
  const uint32_t originalClock = Wire.getClock();
  logMessage("I2C_BEGIN ready=%d SDA=GPIO%d level=%d SCL=GPIO%d level=%d clock=%lu\n",
             i2cReady, hardware::kI2cSda, digitalRead(hardware::kI2cSda),
             hardware::kI2cScl, digitalRead(hardware::kI2cScl), (unsigned long)originalClock);
  const uint32_t clocks[] = {100000, 400000};
  for (uint32_t clock : clocks) {
    const bool changed = Wire.setClock(clock);
    uint16_t found = 0, nacks = 0, timeouts = 0, other = 0;
    char addresses[128] = "";
    size_t used = 0;
    for (uint8_t address = 8; address < 120; ++address) {
      Wire.beginTransmission(address);
      const uint8_t status = Wire.endTransmission(true);
      if (status == 0) {
        ++found;
        if (used + 6 < sizeof(addresses)) {
          used += snprintf(addresses + used, sizeof(addresses) - used, "0x%02X ", address);
        }
      } else if (status == 2) ++nacks;
      else if (status == 5) ++timeouts;
      else ++other;
      pumpLogs();
      delay(1);
    }
    logMessage("I2C_SCAN clock=%lu set=%d ACK=%u [%s] NACK=%u timeout=%u other=%u idle_SDA=%d idle_SCL=%d\n",
               (unsigned long)Wire.getClock(), changed, found, addresses, nacks, timeouts, other,
               digitalRead(hardware::kI2cSda), digitalRead(hardware::kI2cScl));
    for (uint8_t address : {uint8_t(0x68), uint8_t(0x69)}) {
      Wire.beginTransmission(address);
      const uint8_t ackStatus = Wire.endTransmission(true);
      logMessage("I2C_TARGET addr=0x%02X ack_status=%u (0=ACK)\n", address, ackStatus);
      if (ackStatus == 0) {
        probeRegister(address, 0x75, false);
        probeRegister(address, 0x75, true);
        probeRegister(address, 0x6B, false);
      }
      pumpLogs();
    }
  }
  Wire.setClock(originalClock);
  lastFreshUs = 0;
  imu.begin();
  logMessage("I2C_DONE restored=%lu IMU=%s\n", (unsigned long)Wire.getClock(), imu.online ? "ONLINE" : "OFFLINE");
}
void setI2cClock(uint32_t clock) {
  if (calibrationActive()) return;
  const bool changed = Wire.setClock(clock);
  lastFreshUs = 0;
  imu.begin();
  logMessage("I2C_SPEED requested=%lu actual=%lu set=%d IMU=%s\n",
             (unsigned long)clock, (unsigned long)Wire.getClock(), changed, imu.online ? "ONLINE" : "OFFLINE");
}
void handleControls() {
  const uint32_t now = millis();
  if (calibrateButton.update(digitalRead(hardware::kButtonPrimary) == LOW, now)) startCalibration();
  if (modeButton.update(digitalRead(hardware::kButtonSecondary) == LOW, now)) cycleMode();
  for (int n = 0; n < 8 && Serial.available(); ++n) {
    const char c = Serial.read();
    if (c == '?') showHelp();
    else if (c == 'i') showInfo();
    else if (c == 'b') diagnoseI2c();
    else if (c == 'd') dumpSensorRegisters();
    else if (c == '1') setI2cClock(100000);
    else if (c == '4') setI2cClock(400000);
    else if (c == 'c') startCalibration();
    else if (c == 'q' && calibrationActive()) failCalibration("Cancelled");
    else if (!calibrationActive()) {
      if (c == 'm') cycleMode();
      else if (c == 'f' || c == 'u') imu.setMode(c == 'f' ? motion::Fast : motion::Raw);
      else if (c == 's' || c == 'x' || c == 'y') {
        if (c == 's') swapAxes = !swapAxes;
        if (c == 'x') invertX = !invertX;
        if (c == 'y') invertY = !invertY;
        applyMapping();
      } else if (c == 'z') logMessage("CAL CLEAR %s\n", imu.clearCalibration() ? "OK" : "FAILED");
    }
  }
}
void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(1);
  pinMode(hardware::kButtonPrimary, INPUT_PULLUP);
  pinMode(hardware::kButtonSecondary, INPUT_PULLUP);
  // Capture boot-held switches before the color test; require release first.
  calibrateButton.update(digitalRead(hardware::kButtonPrimary) == LOW, millis());
  modeButton.update(digitalRead(hardware::kButtonSecondary) == LOW, millis());
  i2cReady = Wire.begin(hardware::kI2cSda, hardware::kI2cScl);
  Wire.setClock(hardware::kI2cHz);
  Wire.setTimeOut(10);
  SPI.begin(hardware::kLcdClock, -1, hardware::kLcdMosi, hardware::kLcdCS);
  tft.begin(hardware::kLcdSpiHz);
  tft.setRotation(0);
  tft.setTextWrap(false);
  const uint16_t colors[] = {RED, GREEN, 0x001F};
  for (uint16_t color : colors) { tft.fillScreen(color); delay(300); }
  tft.fillScreen(BLACK);
  tft.drawCircle(119, 119, 118, DIM);
  centerText(tft, 120, 14, "IMU TEST", WHITE, 2);
  centerText(tft, 120, 212, "B1:CAL  B2:MODE", WHITE);
  if (!scope.getBuffer() || !numbers.getBuffer()) {
    centerText(tft, 120, 108, "NO DRAW BUFFER", RED);
    while (true) { if (Serial) Serial.println("IMU_DIAG FATAL: no draw buffer"); delay(1000); }
  }
  mountReady = mountSettings.begin("gravity-sand", false);
  if (mountReady) {
    swapAxes = mountSettings.getBool("swap", hardware::kImuSwapAxes);
    invertX = mountSettings.getBool("flipX", hardware::kImuFlipX);
    invertY = mountSettings.getBool("flipY", hardware::kImuFlipY);
  }
  imu.setMapping(swapAxes, invertX, invertY);
  imu.begin();
  previousOnline = imu.online;
  logMessage("DISPLAY: GC9A01 color sequence sent; SPI=%luHz. Visual confirmation required.\n", (unsigned long)hardware::kLcdSpiHz);
  showHelp();
  lastFrameUs = micros();
  lastStatsMs = millis();
  lastSampleCount = imu.samples;
}
void loop() {
  serviceMotion();
  updateCalibration();
  handleControls();
  const uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastFrameUs) >= FRAME_US) {
    lastFrameUs += FRAME_US;
    if ((uint32_t)(nowUs - lastFrameUs) >= FRAME_US) lastFrameUs = nowUs;
    drawFrame();
  }
  const uint32_t now = millis(), elapsed = now - lastStatsMs;
  if (elapsed >= 1000) {
    actualHz = (imu.samples - lastSampleCount) * 1000.0f / elapsed;
    actualFPS = frameCount * 1000.0f / elapsed;
    showInfo();
    lastStatsMs = now;
    lastSampleCount = imu.samples;
    frameCount = 0;
  }
  pumpLogs();
  delay(1);
}
