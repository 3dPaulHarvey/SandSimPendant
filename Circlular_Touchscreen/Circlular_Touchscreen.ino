/*
  Touch Lab V2: XIAO ESP32-S3 + GC9A01 + CST816D

  Libraries:
    Adafruit GC9A01A
    Adafruit GFX Library
    Adafruit BusIO

  Board: XIAO_ESP32S3
  USB CDC On Boot: Enabled
  Serial Monitor: 115200

  LCD:
    VIN      -> 3V3
    GND      -> GND
    BLK      -> 3V3
    CS       -> D0  / GPIO1
    DC       -> D1  / GPIO2
    RES      -> D3  / GPIO4
    SDA/MOSI -> D10 / GPIO9
    SCL/SCK  -> D8  / GPIO7

  Touch:
    TP_SDA -> D4 / GPIO5
    TP_SCL -> D5 / GPIO6
    TP_RST -> D6 / GPIO43
    TP_INT -> D7 / GPIO44

  Tap NEXT for DRAW / SLIDER / SWIPE.

  Serial commands:
    1 = DRAW
    2 = SLIDER
    3 = SWIPE
    c = clear current page
    r = reset touch controller
    b = toggle I2C read style

  White dots are samples; cyan lines connect samples.
  IRQ/s and MOVE/s are observed events, not sensor scan frequency.
  Read/draw timing is software time, not finger-to-screen latency.
*/

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_GC9A01A.h>

constexpr int LCD_CS = 1;
constexpr int LCD_DC = 2;
constexpr int LCD_RST = 4;
constexpr int LCD_SCK = 7;
constexpr int LCD_MOSI = 9;

constexpr int TOUCH_SDA = 5;
constexpr int TOUCH_SCL = 6;
constexpr int TOUCH_RST = 43;
constexpr int TOUCH_INT = 44;

constexpr uint8_t ADDR = 0x15;

constexpr bool SWAP_XY = false;
constexpr bool INVERT_X = false;
constexpr bool INVERT_Y = false;

constexpr uint16_t BLACK = 0x0000;
constexpr uint16_t WHITE = 0xFFFF;
constexpr uint16_t CYAN  = 0x07FF;
constexpr uint16_t GREY  = 0x4208;
constexpr uint16_t GREEN = 0x07E0;

Adafruit_GC9A01A tft(&SPI, LCD_DC, LCD_CS, LCD_RST);

portMUX_TYPE irqMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t irqCount = 0;

uint32_t handledIRQ = 0;
uint32_t statsIRQ = 0;

// If your successful Recovery test needed 'b', change this to false.
bool stopBetween = true;

int mode = 0;  // 0=DRAW, 1=SLIDER, 2=SWIPE

bool down = false;
bool navigation = false;
bool suppressUntilUp = false;

int x = 0, y = 0;
int startX = 0, startY = 0;
int minX = 0, maxX = 0;
int minY = 0, maxY = 0;
int excursion2 = 0;

int fingers = 0;
int eventCode = 0;
int rawX = 0, rawY = 0;

int sliderValue = 50;
int shownSlider = -1;

uint8_t lastError = 0;

uint32_t beganMs = 0;
uint32_t lastPointMs = 0;
uint32_t lastValidMs = 0;
uint32_t lastPollUs = 0;
uint32_t lastSliderMs = 0;
uint32_t statsMs = 0;
uint32_t contactMoves = 0;

uint32_t good = 0;
uint32_t bad = 0;
uint32_t invalid = 0;
uint32_t moves = 0;
uint32_t maxReadUs = 0;
uint32_t maxDrawUs = 0;
uint32_t droppedLogs = 0;

// Bounded queue: diagnostics never wait for the USB reader.
char logs[8][192];
uint8_t logHead = 0;
uint8_t logTail = 0;
size_t logOffset = 0;


void queueLog(const char *s) {
  uint8_t next = (logHead + 1) % 8;

  if (next == logTail) {
    ++droppedLogs;
    return;
  }

  snprintf(logs[logHead], sizeof(logs[0]), "%s", s);
  logHead = next;
}


void pumpLog() {
  if (!Serial || logHead == logTail) return;

  int room = Serial.availableForWrite();
  if (room <= 0) return;

  size_t count = strlen(logs[logTail]) - logOffset;
  if (count > (size_t)room) count = room;
  if (count > 32) count = 32;

  logOffset += Serial.write(
    (const uint8_t *)(logs[logTail] + logOffset),
    count
  );

  if (logOffset == strlen(logs[logTail])) {
    logTail = (logTail + 1) % 8;
    logOffset = 0;
  }
}


void ARDUINO_ISR_ATTR touchISR() {
  portENTER_CRITICAL_ISR(&irqMux);
  ++irqCount;
  portEXIT_CRITICAL_ISR(&irqMux);
}


uint32_t getIRQcount() {
  portENTER_CRITICAL(&irqMux);
  uint32_t n = irqCount;
  portEXIT_CRITICAL(&irqMux);
  return n;
}


bool readRegs(uint8_t reg, uint8_t *out, uint8_t len) {
  Wire.beginTransmission(ADDR);
  Wire.write(reg);

  lastError = Wire.endTransmission(stopBetween);
  if (lastError) return false;

  size_t got = Wire.requestFrom(ADDR, len);

  if (got != len) {
    lastError = 254;
    while (Wire.available()) Wire.read();
    return false;
  }

  for (uint8_t i = 0; i < len; ++i) {
    out[i] = Wire.read();
  }

  return true;
}


bool writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission(true) == 0;
}


bool decodeTouch(
  const uint8_t *b,
  int &n,
  int &ev,
  int &px,
  int &py
) {
  n = b[2] & 3;
  ev = b[3] >> 6;

  px = ((b[3] & 15) << 8) | b[4];
  py = ((b[5] & 15) << 8) | b[6];

  return n > 0 && px < 240 && py < 240;
}


void resetTouch() {
  down = false;
  suppressUntilUp = false;

  digitalWrite(TOUCH_RST, LOW);
  delay(10);
  digitalWrite(TOUCH_RST, HIGH);
  delay(150);

  // Same configuration as the working Recovery sketch.
  bool m = writeReg(0xFA, 0x60);
  bool a = writeReg(0xFE, 0x01);

  uint8_t id = 0xFF;
  uint8_t cfg = 0xFF;
  uint8_t awake = 0xFF;

  bool i = readRegs(0xA7, &id, 1);
  bool c = readRegs(0xFA, &cfg, 1);
  bool f = readRegs(0xFE, &awake, 1);

  char line[192];

  snprintf(
    line, sizeof(line),
    "INIT ID=%02X/%s FA=%02X/%s FE=%02X/%s "
    "writes=%u,%u bus=%s\n",
    id, i ? "OK" : "FAIL",
    cfg, c ? "OK" : "FAIL",
    awake, f ? "OK" : "FAIL",
    (unsigned)m,
    (unsigned)a,
    stopBetween ? "STOP" : "RESTART"
  );

  queueLog(line);
  handledIRQ = getIRQcount();
  lastValidMs = millis();
}


void textAt(
  int px,
  int py,
  const char *s,
  uint16_t color,
  int size
) {
  tft.setCursor(px, py);
  tft.setTextSize(size);
  tft.setTextColor(color, BLACK);
  tft.print(s);
}


bool nextHit(int px, int py) {
  return px >= 78 && px <= 162 &&
         py >= 200 && py <= 224;
}


void drawSlider() {
  uint32_t began = micros();

  if (shownSlider >= 0) {
    int oldKnob = 28 + shownSlider * 184 / 100;
    tft.fillCircle(oldKnob, 130, 9, BLACK);
  }

  tft.fillRect(28, 128, 185, 5, GREY);

  int knob = 28 + sliderValue * 184 / 100;
  tft.fillCircle(knob, 130, 8, CYAN);

  char value[8];
  snprintf(value, sizeof(value), "%3d%%", sliderValue);
  textAt(96, 93, value, WHITE, 2);

  shownSlider = sliderValue;
  maxDrawUs = max(maxDrawUs, (uint32_t)(micros() - began));
}


void page() {
  tft.fillScreen(BLACK);
  tft.drawCircle(120, 120, 118, GREY);

  const char *titles[] = {
    "DRAW", "SLIDER", "SWIPE"
  };

  const char *hints[] = {
    "Draw slow / fast lines",
    "Drag left / right",
    "Swipe, then lift"
  };

  textAt(
    120 - 6 * (int)strlen(titles[mode]),
    24, titles[mode], CYAN, 2
  );

  textAt(
    120 - 3 * (int)strlen(hints[mode]),
    52, hints[mode], WHITE, 1
  );

  tft.fillRoundRect(78, 200, 85, 25, 6, GREY);
  textAt(102, 209, "NEXT >", WHITE, 1);

  if (mode == 1) {
    shownSlider = -1;
    drawSlider();
    textAt(54, 160, "Start almost anywhere", WHITE, 1);
  }
}


const char *classifyGesture(
  int dx,
  int dy,
  int spread2,
  uint32_t ms
) {
  int ax = abs(dx);
  int ay = abs(dy);

  if (dx * dx + dy * dy >= 45 * 45 && ms <= 800) {
    if (ax * 10 >= ay * 14) {
      return dx > 0 ? "RIGHT" : "LEFT";
    }

    if (ay * 10 >= ax * 14) {
      return dy > 0 ? "DOWN" : "UP";
    }

    return "DIAGONAL";
  }

  if (spread2 <= 12 * 12) {
    if (ms <= 350) return "TAP";
    if (ms >= 600) return "HOLD";
    return "PRESS";
  }

  return "DRAG";
}


void finishContact(bool cancelled) {
  if (!down) return;
  down = false;

  if (cancelled) {
    suppressUntilUp = true;
    queueLog("CANCEL: reports lost; lift finger to resume.\n");
    return;
  }

  if (navigation) {
    if (nextHit(x, y) && excursion2 <= 12 * 12) {
      mode = (mode + 1) % 3;
      page();
    }
    return;
  }

  uint32_t duration = millis() - beganMs;
  int dx = x - startX;
  int dy = y - startY;

  const char *name = classifyGesture(
    dx, dy, excursion2, duration
  );

  char line[192];

  snprintf(
    line, sizeof(line),
    "%s: %lums dx=%d dy=%d span=%dx%dpx changes=%lu\n",
    name,
    (unsigned long)duration,
    dx, dy,
    maxX - minX,
    maxY - minY,
    (unsigned long)contactMoves
  );

  queueLog(line);

  if (mode == 2) {
    tft.fillRect(42, 91, 156, 57, BLACK);

    textAt(
      120 - 6 * (int)strlen(name),
      96, name, GREEN, 2
    );

    snprintf(
      line, sizeof(line),
      "%lums  %d,%dpx",
      (unsigned long)duration, dx, dy
    );

    textAt(
      120 - 3 * (int)strlen(line),
      130, line, WHITE, 1
    );
  }
}


void acceptPoint(int px, int py) {
  uint32_t now = millis();
  bool first = !down;

  int previousX = x;
  int previousY = y;

  if (first) {
    down = true;
    navigation = nextHit(px, py);

    startX = minX = maxX = px;
    startY = minY = maxY = py;

    excursion2 = 0;
    contactMoves = 0;
    beganMs = now;
  }

  x = px;
  y = py;

  minX = min(minX, x);
  maxX = max(maxX, x);
  minY = min(minY, y);
  maxY = max(maxY, y);

  int dx = x - startX;
  int dy = y - startY;
  excursion2 = max(excursion2, dx * dx + dy * dy);

  bool changed = first || x != previousX || y != previousY;

  if (!first && changed) ++contactMoves;
  if (changed) ++moves;

  if (!navigation) {
    if (mode == 1) {
      // No requirement to begin directly on the slider track.
      sliderValue = constrain((x - 28) * 100 / 184, 0, 100);
    } else if (changed) {
      uint32_t began = micros();

      tft.startWrite();

      if (!first && (uint32_t)(now - lastPointMs) < 80) {
        tft.writeLine(previousX, previousY, x, y, CYAN);
      }

      tft.writeFillRect(
        max(0, x - 1),
        max(0, y - 1),
        3, 3, WHITE
      );

      tft.endWrite();

      maxDrawUs = max(
        maxDrawUs,
        (uint32_t)(micros() - began)
      );
    }
  }

  lastPointMs = now;
}


void sampleTouch() {
  uint32_t edges = getIRQcount();
  uint32_t nowUs = micros();

  // Same interrupt + 10 ms fallback polling as Recovery.
  if (
    edges == handledIRQ &&
    (uint32_t)(nowUs - lastPollUs) < 10000
  ) {
    return;
  }

  handledIRQ = edges;
  lastPollUs = nowUs;

  uint8_t b[7];

  uint32_t started = micros();
  bool ok = readRegs(0x00, b, sizeof(b));

  maxReadUs = max(
    maxReadUs,
    (uint32_t)(micros() - started)
  );

  if (!ok) {
    ++bad;
    return;
  }

  ++good;

  bool valid = decodeTouch(
    b, fingers, eventCode, rawX, rawY
  );

  if (fingers == 0) {
    // An outage followed by release must not create a swipe.
    if (
      down &&
      (uint32_t)(millis() - lastValidMs) > 250
    ) {
      finishContact(true);
    } else {
      finishContact(false);
    }

    suppressUntilUp = false;
    lastValidMs = millis();
    return;
  }

  if (!valid) {
    ++invalid;
    return;
  }

  if (
    down &&
    (uint32_t)(millis() - lastValidMs) > 250
  ) {
    finishContact(true);
  }

  lastValidMs = millis();

  if (suppressUntilUp) return;

  int px = rawX;
  int py = rawY;

  if (SWAP_XY) {
    int tmp = px;
    px = py;
    py = tmp;
  }

  if (INVERT_X) px = 239 - px;
  if (INVERT_Y) py = 239 - py;

  acceptPoint(px, py);
}


void reportStats() {
  uint32_t now = millis();
  uint32_t dt = now - statsMs;

  if (dt < 1000) return;

  uint32_t edges = getIRQcount();
  char line[192];

  snprintf(
    line, sizeof(line),
    "IRQ/s=%.1f MOVE/s=%.1f ok=%lu bad=%lu invalid=%lu "
    "readMax=%luus drawMax=%luus dropped=%lu\n",
    (edges - statsIRQ) * 1000.0 / dt,
    moves * 1000.0 / dt,
    (unsigned long)good,
    (unsigned long)bad,
    (unsigned long)invalid,
    (unsigned long)maxReadUs,
    (unsigned long)maxDrawUs,
    (unsigned long)droppedLogs
  );

  queueLog(line);

  statsIRQ = edges;
  statsMs = now;

  good = bad = invalid = moves = maxReadUs = maxDrawUs = 0;
}


void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(1);

  pinMode(TOUCH_RST, OUTPUT);
  digitalWrite(TOUCH_RST, HIGH);
  pinMode(TOUCH_INT, INPUT_PULLUP);

  Wire.begin(TOUCH_SDA, TOUCH_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(10);

  SPI.begin(LCD_SCK, -1, LCD_MOSI, LCD_CS);

  tft.begin(8000000);
  tft.setRotation(0);
  tft.setTextWrap(false);

  page();

  attachInterrupt(
    digitalPinToInterrupt(TOUCH_INT),
    touchISR,
    FALLING
  );

  resetTouch();
  statsMs = millis();
}


void loop() {
  if (Serial.available()) {
    char c = Serial.read();

    if ((c >= '1' && c <= '3') || c == 'c') {
      if (c != 'c') mode = c - '1';

      down = false;
      suppressUntilUp = true;
      page();
    }

    if (c == 'r' || c == 'b') {
      if (c == 'b') stopBetween = !stopBetween;

      resetTouch();
      page();
    }
  }

  sampleTouch();

  if (
    down &&
    (uint32_t)(millis() - lastValidMs) > 250
  ) {
    finishContact(true);
  }

  // Limit slider drawing, without limiting touch acquisition.
  if (
    mode == 1 &&
    sliderValue != shownSlider &&
    (uint32_t)(millis() - lastSliderMs) >= 16
  ) {
    drawSlider();
    lastSliderMs = millis();
  }

  reportStats();
  pumpLog();

  delay(1);
}