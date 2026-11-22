/*
  MPU motion check / flat bias calibration. Separate from the sand sketch.
  XIAO ESP32-S3 + existing GC9A01/CST816D + MPU6050 on D4/D5.
  Touch CAL: place MPU itself FLAT, COMPONENT SIDE UP, leave still.
  Touch right button: FAST -> RAW -> OLD response for comparison.
  Calibration is saved in NVS and also used by Gravity_Sand.
*/
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_GC9A01A.h>
#include <Preferences.h>
#include <stdarg.h>
#include "../shared/MPU6050Motion.h"

constexpr int LCD_CS = 1, LCD_DC = 2, LCD_RST = 4;
constexpr int LCD_SCK = 7, LCD_MOSI = 9;
constexpr int TOUCH_RST = 43, TOUCH_INT = 44;
constexpr uint8_t TOUCH_ADDR = 0x15;
constexpr uint16_t BLACK = 0, WHITE = 0xFFFF, DIM = 0x2945;
constexpr uint16_t CYAN = 0x07FF, GOLD = 0xFEA0, GREEN = 0x07E0, RED = 0xF800;
constexpr int SCOPE_X = 44, SCOPE_Y = 42, SCOPE_W = 152, SCOPE_H = 112;
constexpr uint32_t FRAME_US = 16667;

Adafruit_GC9A01A tft(&SPI, LCD_DC, LCD_CS, LCD_RST);
GFXcanvas16 scope(SCOPE_W, SCOPE_H);
GFXcanvas16 numbers(172, 43);
motion::MPU6050Motion imu;
motion::StationaryCalibrator calibrator(400);
Preferences mountSettings;
bool mountReady = false, swapAxes = false, invertX = false, invertY = false;

char logQueue[16][256];
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
bool chromeDirty = true, previousOnline = false;
bool plotCSV = false;
uint32_t lastFrameUs = 0, lastNumbersMs = 0, lastStatsMs = 0, lastPlotMs = 0;
uint32_t frameCount = 0, lastSampleCount = 0;
float actualHz = 0, actualFPS = 0;

bool calibrationActive() {
  return calStage == CalStage::Countdown || calStage == CalStage::Collecting;
}
const char *modeName() {
  if (imu.mode == motion::Raw) return "RAW";
  if (imu.mode == motion::Legacy) return "OLD";
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
  chromeDirty = true;
  logMessage("CAL FAIL: %s. Previous calibration preserved.\n", calError);
}
void startCalibration() {
  if (!imu.online) { logMessage("CAL: no MPU6050 connected.\n"); return; }
  if (calibrationActive()) {
    failCalibration("Cancelled");
    return;
  }
  previousMode = imu.mode;
  if (!imu.setMode(motion::Fast)) { logMessage("CAL: MPU configuration failed.\n"); return; }
  calibrator.reset();
  calStage = CalStage::Countdown;
  calStageMs = millis();
  chromeDirty = true;
  logMessage("CAL starts in 3s. MPU FLAT, COMPONENT SIDE UP. Do not move it.\n");
}
void serviceMotion() {
  bool fresh = imu.update();
  if (imu.online != previousOnline) {
    previousOnline = imu.online;
    chromeDirty = true;
    logMessage("MPU %s address=0x%02X\n", imu.online ? "ONLINE" : "OFFLINE", imu.address);
  }
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
        chromeDirty = true;
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
    chromeDirty = true;
  }
}

void centerText(Adafruit_GFX &target, int cx, int y, const char *text, uint16_t color, int size = 1) {
  target.setTextSize(size);
  target.setTextWrap(false);
  target.setTextColor(color, BLACK);
  target.setCursor(cx - (int)strlen(text) * 3 * size, y);
  target.print(text);
}
void drawChrome() {
  centerText(tft, 119, 17, "MPU MOTION", WHITE, 1);
  tft.fillCircle(119, 5, 2, imu.online ? GREEN : RED);
  tft.fillRect(50, 28, 140, 10, BLACK);
  const char *profile = imu.mode == motion::Raw ? "RAW 200Hz, no SW filter" :
                        imu.mode == motion::Legacy ? "OLD 100Hz, tau 40ms" : "FAST 200Hz, tau 8ms";
  centerText(tft, 119, 29, profile, GOLD);
  tft.fillRoundRect(60, 202, 54, 20, 5, DIM);
  tft.fillRoundRect(124, 202, 54, 20, 5, calibrationActive() ? DIM : 0x0165);
  centerText(tft, 87, 209, calibrationActive() ? "STOP" : "CAL", WHITE);
  centerText(tft, 151, 209, modeName(), WHITE);
  chromeDirty = false;
}
void drawScope() {
  scope.fillScreen(BLACK);
  constexpr int cx = SCOPE_W / 2, cy = 51, radius = 45;
  scope.drawCircle(cx, cy, radius, DIM);
  scope.drawCircle(cx, cy, radius / 2, 0x10A2);
  scope.drawFastHLine(cx - radius, cy, radius * 2 + 1, DIM);
  scope.drawFastVLine(cx, cy - radius, radius * 2 + 1, DIM);
  if (!imu.online) {
    scope.fillRect(0, 25, SCOPE_W, 49, BLACK);
    centerText(scope, cx, 30, "NO MPU6050", RED, 2);
    centerText(scope, cx, 53, "SDA D4   SCL D5", WHITE);
    centerText(scope, cx, 66, "3V3 and shared GND", WHITE);
  } else if (calStage == CalStage::Countdown) {
    scope.fillRect(0, 20, SCOPE_W, 65, BLACK);
    centerText(scope, cx, 24, "MPU FLAT / CHIP UP", WHITE);
    char text[28];
    snprintf(text, sizeof(text), "Starting in %lu", (unsigned long)(3 - min((uint32_t)3, (millis() - calStageMs) / 1000)));
    centerText(scope, cx, 44, text, GOLD, 1);
    centerText(scope, cx, 66, "Keep the MPU still", WHITE);
  } else if (calStage == CalStage::Collecting) {
    scope.fillRect(0, 20, SCOPE_W, 67, BLACK);
    centerText(scope, cx, 26, "CALIBRATING", GOLD, 2);
    centerText(scope, cx, 51, "Flat, chip up, still", WHITE);
    char text[24];
    snprintf(text, sizeof(text), "%lu / 400 readings", (unsigned long)calibrator.count());
    centerText(scope, cx, 67, text, WHITE);
    scope.drawRect(10, 83, 132, 8, DIM);
    scope.fillRect(11, 84, calibrator.count() * 130 / 400, 6, GOLD);
  } else if (calStage == CalStage::Success || calStage == CalStage::Failed) {
    scope.fillRect(0, 24, SCOPE_W, 51, BLACK);
    centerText(scope, cx, 27, calStage == CalStage::Success ? "CAL SAVED" : "TRY AGAIN", calStage == CalStage::Success ? GREEN : RED, 2);
    centerText(scope, cx, 51, calStage == CalStage::Success ? "Offsets shared with sand" : calError, WHITE);
    centerText(scope, cx, 65, calStage == CalStage::Success ? "Now tilt the MPU" : "Old offsets kept", WHITE);
  } else {
    motion::Vec2 input = imu.inputGravity, output = imu.gravity;
    float inLength = sqrtf(input.x * input.x + input.y * input.y);
    float outLength = sqrtf(output.x * output.x + output.y * output.y);
    float inScale = radius / max(1.0f, inLength), outScale = radius / max(1.0f, outLength);
    int ix = cx + lroundf(input.x * inScale), iy = cy + lroundf(input.y * inScale);
    int ox = cx + lroundf(output.x * outScale), oy = cy + lroundf(output.y * outScale);
    scope.drawLine(cx, cy, ix, iy, CYAN);
    scope.drawCircle(ix, iy, 5, CYAN);
    scope.drawLine(cx, cy, ox, oy, GOLD);
    scope.fillCircle(ox, oy, 3, GOLD);
    scope.drawPixel(cx, cy, WHITE);
  }
  centerText(scope, 42, 102, "CYAN input", CYAN);
  centerText(scope, 116, 102, "GOLD out", GOLD);
  // Short transfers leave room to acquire fresh MPU readings between bands.
  // This keeps a full scope repaint from blocking the 200Hz sensor sampler.
  tft.startWrite();
  for (int row = 0; row < SCOPE_H; row += 8) {
    int height = min(8, SCOPE_H - row);
    tft.setAddrWindow(SCOPE_X, SCOPE_Y + row, SCOPE_W, height);
    tft.writePixels(scope.getBuffer() + row * SCOPE_W, SCOPE_W * height, true, false);
    serviceMotion();
    pumpLogs();
  }
  tft.endWrite();
  ++frameCount;
}
void drawNumbers() {
  numbers.fillScreen(BLACK);
  char line[64];
  snprintf(line, sizeof(line), "gx%+.3f gy%+.3f", imu.gravity.x, imu.gravity.y);
  centerText(numbers, 86, 0, line, WHITE);
  snprintf(line, sizeof(line), "Z%+.3f |g|%.3f", imu.accel.z, motion::norm(imu.accel));
  centerText(numbers, 86, 10, line, WHITE);
  snprintf(line, sizeof(line), "gyro %+.0f %+.0f %+.0f d/s", imu.gyro.x, imu.gyro.y, imu.gyro.z);
  centerText(numbers, 86, 20, line, DIM);
  snprintf(line, sizeof(line), "%.0fHz %.0ffps CAL:%s", actualHz, actualFPS, imu.calibration.valid ? "SET" : "--");
  centerText(numbers, 86, 31, line, imu.calibration.valid ? GREEN : GOLD);
  tft.startWrite();
  for (int row = 0; row < 43; row += 8) {
    int height = min(8, 43 - row);
    tft.setAddrWindow(34, 156 + row, 172, height);
    tft.writePixels(numbers.getBuffer() + row * 172, 172 * height, true, false);
    serviceMotion();
  }
  tft.endWrite();
}

bool touchReady = false, touching = false, touchCancelled = false;
int touchStartX = 0, touchStartY = 0, touchExcursion2 = 0;
uint32_t touchBeganMs = 0, touchGoodMs = 0, lastTouchPollMs = 0;
void beginTouch() {
  pinMode(TOUCH_RST, OUTPUT); pinMode(TOUCH_INT, INPUT_PULLUP);
  digitalWrite(TOUCH_RST, LOW); delay(10);
  digitalWrite(TOUCH_RST, HIGH); delay(150);
  Wire.beginTransmission(TOUCH_ADDR); Wire.write(0xFA); Wire.write(0x60);
  bool ok = Wire.endTransmission() == 0;
  Wire.beginTransmission(TOUCH_ADDR); Wire.write(0xFE); Wire.write(0x01);
  touchReady = Wire.endTransmission() == 0 && ok;
}
void cycleMode() {
  if (calibrationActive()) return;
  motion::ResponseMode next = imu.mode == motion::Fast ? motion::Raw :
                              imu.mode == motion::Raw ? motion::Legacy : motion::Fast;
  imu.setMode(next);
  chromeDirty = true;
  logMessage("MODE %s\n", modeName());
}
void sampleTouch() {
  uint32_t now = millis();
  if (!touchReady || (uint32_t)(now - lastTouchPollMs) < 20) return;
  lastTouchPollMs = now;
  Wire.beginTransmission(TOUCH_ADDR); Wire.write(0);
  if (Wire.endTransmission(true) != 0 || Wire.requestFrom(TOUCH_ADDR, (uint8_t)7) != 7) {
    while (Wire.available()) Wire.read();
    if (touching && (uint32_t)(now - touchGoodMs) > 250) touchCancelled = true;
    return;
  }
  uint8_t b[7]; for (int i = 0; i < 7; ++i) b[i] = Wire.read();
  if (!(b[2] & 3) || b[3] >> 6 == 1) {
    if (touching && !touchCancelled && (uint32_t)(now - touchGoodMs) <= 250 &&
        (uint32_t)(now - touchBeganMs) <= 500 && touchExcursion2 < 15 * 15) {
      if (touchStartY >= 198 && touchStartY <= 226) {
        if (touchStartX >= 58 && touchStartX <= 116) startCalibration();
        if (touchStartX >= 122 && touchStartX <= 180) cycleMode();
      }
    }
    touching = touchCancelled = false;
    return;
  }
  int x = ((b[3] & 15) << 8) | b[4], y = ((b[5] & 15) << 8) | b[6];
  if (x >= 240 || y >= 240) return;
  if (!touching) {
    touching = true; touchBeganMs = now;
    touchStartX = x; touchStartY = y; touchExcursion2 = 0;
  } else if ((uint32_t)(now - touchGoodMs) > 250) touchCancelled = true;
  int dx = x - touchStartX, dy = y - touchStartY;
  touchExcursion2 = max(touchExcursion2, dx * dx + dy * dy);
  touchGoodMs = now;
}
void applyMapping() {
  imu.setMapping(swapAxes, invertX, invertY);
  if (mountReady) {
    mountSettings.putBool("swap", swapAxes);
    mountSettings.putBool("flipX", invertX);
    mountSettings.putBool("flipY", invertY);
  }
  logMessage("MOUNT swap=%d flipX=%d flipY=%d (shared with sand).\n", swapAxes, invertX, invertY);
}
void showInfo() {
  logMessage("MPU %s addr=0x%02X mode=%s calibration=%s errors=%lu\n", imu.online ? "ONLINE" : "OFFLINE",
    imu.address, modeName(), imu.calibration.valid ? "SAVED" : "NONE", (unsigned long)imu.errors);
  logMessage("BIAS accel=(%.5f,%.5f,%.5f)g gyro=(%.3f,%.3f,%.3f)dps\n",
    imu.calibration.accelBias.x, imu.calibration.accelBias.y, imu.calibration.accelBias.z,
    imu.calibration.gyroBias.x, imu.calibration.gyroBias.y, imu.calibration.gyroBias.z);
}
void showHelp() {
  logMessage("MPU VISUALIZER: c=flat chip-up calibration (3s countdown, 400 fresh samples), q=cancel.\n");
  logMessage("f=FAST (200Hz/94Hz chip filter/tau8ms), u=RAW (no software smoothing), o=OLD (100Hz/21Hz/tau40ms).\n");
  logMessage("s=swap, x/y=flip screen axes (saved/shared with sand). z=clear calibration, p=CSV toggle, i=info.\n");
  logMessage("Cyan=input after bias correction; gold=filtered output. Flat, chip-up: Z~+1g and XY~0g.\n");
}
void handleSerial() {
  for (int n = 0; n < 8 && Serial.available(); ++n) {
    char c = Serial.read();
    if (c == '?') showHelp();
    else if (c == 'c') startCalibration();
    else if (c == 'q' && calibrationActive()) failCalibration("Cancelled");
    else if (c == 'i') showInfo();
    else if (c == 'p') {
      plotCSV = !plotCSV;
      if (plotCSV) logMessage("CSV,ms,ax,ay,az,gx,gy,gz,inputX,inputY,outX,outY\n");
    } else if (!calibrationActive()) {
      if (c == 'f' || c == 'u' || c == 'o') {
        imu.setMode(c == 'f' ? motion::Fast : c == 'u' ? motion::Raw : motion::Legacy);
        chromeDirty = true; logMessage("MODE %s\n", modeName());
      } else if (c == 's' || c == 'x' || c == 'y') {
        if (c == 's') swapAxes = !swapAxes;
        if (c == 'x') invertX = !invertX;
        if (c == 'y') invertY = !invertY;
        applyMapping();
      } else if (c == 'z') {
        bool ok = imu.clearCalibration(); chromeDirty = true;
        logMessage("Calibration clear: %s\n", ok ? "OK" : "FAILED");
      }
    }
  }
}
void setup() {
  Serial.begin(115200); Serial.setTxTimeoutMs(1);
  Wire.begin(5, 6); Wire.setClock(400000); Wire.setTimeOut(10);
  SPI.begin(LCD_SCK, -1, LCD_MOSI, LCD_CS);
  tft.begin(40000000); tft.setRotation(0); tft.setTextWrap(false);
  tft.fillScreen(BLACK); tft.drawCircle(119, 119, 118, DIM);
  if (!scope.getBuffer() || !numbers.getBuffer()) {
    centerText(tft, 119, 110, "NO DRAW BUFFER", RED);
    while (true) delay(1000);
  }
  mountReady = mountSettings.begin("gravity-sand", false);
  if (mountReady) {
    swapAxes = mountSettings.getBool("swap", false);
    invertX = mountSettings.getBool("flipX", false);
    invertY = mountSettings.getBool("flipY", false);
  }
  imu.setMapping(swapAxes, invertX, invertY);
  beginTouch(); imu.begin();
  showHelp(); showInfo();
  previousOnline = imu.online;
  lastFrameUs = micros(); lastStatsMs = millis(); lastSampleCount = imu.samples;
}
void loop() {
  handleSerial(); serviceMotion(); updateCalibration(); sampleTouch();
  if (chromeDirty) drawChrome();
  uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastFrameUs) >= FRAME_US) {
    lastFrameUs += FRAME_US;
    if ((uint32_t)(nowUs - lastFrameUs) >= FRAME_US) lastFrameUs = nowUs;
    drawScope();
  }
  uint32_t now = millis();
  if ((uint32_t)(now - lastNumbersMs) >= 100) { lastNumbersMs = now; drawNumbers(); }
  uint32_t elapsed = now - lastStatsMs;
  if (elapsed >= 1000) {
    actualHz = (imu.samples - lastSampleCount) * 1000.0f / elapsed;
    actualFPS = frameCount * 1000.0f / elapsed;
    logMessage("VIS %s mode=%s cal=%d Hz=%.1f fps=%.1f age=%lums rawA=(%.4f,%.4f,%.4f) input=(%.4f,%.4f) out=(%.4f,%.4f) gyro=(%.2f,%.2f,%.2f) errors=%lu\n",
      imu.online ? "ONLINE" : "OFFLINE", modeName(), imu.calibration.valid, actualHz, actualFPS,
      (unsigned long)((micros() - imu.lastSampleUs) / 1000),
      imu.rawAccel.x, imu.rawAccel.y, imu.rawAccel.z, imu.inputGravity.x, imu.inputGravity.y,
      imu.gravity.x, imu.gravity.y, imu.gyro.x, imu.gyro.y, imu.gyro.z, (unsigned long)imu.errors);
    lastStatsMs = now; lastSampleCount = imu.samples; frameCount = 0;
  }
  if (plotCSV && (uint32_t)(now - lastPlotMs) >= 50) {
    lastPlotMs = now;
    logMessage("CSV,%lu,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.4f\n",
      (unsigned long)now, imu.accel.x, imu.accel.y, imu.accel.z, imu.gyro.x, imu.gyro.y, imu.gyro.z,
      imu.inputGravity.x, imu.inputGravity.y, imu.gravity.x, imu.gravity.y);
  }
  pumpLogs(); delay(1);
}
