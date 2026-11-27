#ifndef SIMCIRCLE_MPU6050_MOTION_H
#define SIMCIRCLE_MPU6050_MOTION_H

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <stddef.h>
#include <string.h>
#include "MotionCalibration.h"

namespace motion {

enum ResponseMode : uint8_t { Fast = 0, Raw = 1, Legacy = 2 };

// The caller owns the I2C bus and its pins, speed and timeout. All
// measurements use g and degrees/sec in the physical chip axes. Calibration
// is applied before mapping acceleration into screen-right/screen-down axes.
class MPU6050Motion {
 public:
  using ResponseMode = motion::ResponseMode;
  static constexpr ResponseMode Fast = motion::Fast;
  static constexpr ResponseMode Raw = motion::Raw;
  static constexpr ResponseMode Legacy = motion::Legacy;

  bool online = false;
  uint8_t address = 0, identity = 0;
  uint32_t errors = 0, samples = 0, lastSampleUs = 0, rejectedSamples = 0;
  static constexpr uint8_t kRejectUniformPacket = 0x01;
  static constexpr uint8_t kRejectClippedAxis = 0x02;
  static constexpr uint8_t kRejectFloatingVector = 0x04;
  static constexpr uint8_t kRejectInterruptStatus = 0x08;
  uint8_t lastRejectReason = 0, lastRejectedPacket[15] = {};
  Vec3 rawAccel, accel, rawGyro, gyro;
  // Raw/calibrated fields above remain unfiltered for calibration/diagnosis.
  // Motion consumers use a synchronized median of three fresh samples instead
  // of allowing a single corrupt I2C burst to become a gravity/shake impulse.
  Vec3 motionAccel, motionGyro;
  bool motionReady = false;
  Vec2 inputGravity, gravity;
  ResponseMode mode = motion::Fast;
  Calibration calibration;

  explicit MPU6050Motion(TwoWire &wire = Wire, uint8_t compatibilityId = 0)
      : wire_(wire), compatibilityId_(compatibilityId) {}

  bool compatibilityMode() const { return online && identity != 0x68; }

  bool begin() {
    started_ = true;
    if (!calibrationLoaded_) loadCalibration();
    online = false;
    address = 0;
    identity = 0;
    lastProbeMs_ = millis();
    resetFilter();
    const uint8_t addresses[] = {0x68, 0x69};
    for (uint8_t candidate : addresses) {
      uint8_t detected = 0;
      if (!readRegisters(candidate, 0x75, &detected, 1)) continue;
      identity = detected;
      if (identity != 0x68 && (!compatibilityId_ || identity != compatibilityId_)) continue;
      if (!writeRegister(candidate, 0x6B, 0x01)) { ++errors; continue; }
      delay(100);  // Wake/PLL stabilization only after an MPU actually responds.
      if (!configure(candidate, mode)) { ++errors; continue; }
      address = candidate;
      online = true;
      lastFreshMs_ = millis();
      lastPollUs_ = micros();
      resetFilter();
      return true;
    }
    return false;
  }

  // True means one fresh DATA_RDY sample was accepted. Polls that find no new
  // conversion, failed transactions and reconnect attempts never increment
  // samples, so a calibration collector cannot count duplicate register data.
  bool update() {
    if (!started_) { begin(); return false; }
    const uint32_t nowMs = millis();
    if (!online) {
      if (static_cast<uint32_t>(nowMs - lastProbeMs_) >= 2000) begin();
      return false;
    }
    if (static_cast<uint32_t>(nowMs - lastFreshMs_) >= 250) {
      ++errors;
      disconnect(nowMs);
      return false;
    }
    const uint32_t nowUs = micros();
    const uint32_t intervalUs = mode == motion::Legacy ? 10000 : 5000;
    if (static_cast<uint32_t>(nowUs - lastPollUs_) < intervalUs) return false;
    lastPollUs_ = nowUs;

    // INT_STATUS followed by accel, temperature and gyro, in one coherent
    // burst. Reading INT_STATUS clears the DATA_RDY latch for the next poll.
    uint8_t bytes[15];
    if (!readRegisters(address, 0x3A, bytes, sizeof(bytes))) {
      ++errors;
      return false;
    }
    if (!(bytes[0] & 0x01)) return false;
    const uint8_t rejection = packetRejectReason(bytes);
    if (rejection) {
      ++errors;
      ++rejectedSamples;
      lastRejectReason = rejection;
      memcpy(lastRejectedPacket, bytes, sizeof(lastRejectedPacket));
      return false;
    }

    rawAccel = Vec3(word(bytes + 1) / 8192.0f,
                    word(bytes + 3) / 8192.0f,
                    word(bytes + 5) / 8192.0f);
    rawGyro = Vec3(word(bytes + 9) / 65.5f,
                   word(bytes + 11) / 65.5f,
                   word(bytes + 13) / 65.5f);
    accel = correctAcceleration(rawAccel, calibration);
    gyro = correctGyro(rawGyro, calibration);
    qualifyMotion();
    inputGravity = mapGravity(mode == motion::Raw ? accel : motionAccel,
                              swap_, flipX_, flipY_);

    const uint32_t sampleUs = micros();
    const float tauSeconds = mode == motion::Fast ? 0.008f :
                             (mode == motion::Legacy ? 0.04025f : 0.0f);
    if (!motionReady && mode != motion::Raw) {
      // Collect two startup samples before publishing a median. Callers must
      // also wait for motionReady before initializing their shake reference.
      gravity = Vec2();
    } else if (firstFilter_) {
      gravity = inputGravity;
      firstFilter_ = false;
    } else {
      const float dt = static_cast<uint32_t>(sampleUs - filterSampleUs_) * 0.000001f;
      const float alpha = smoothingAlpha(dt, tauSeconds);
      gravity.x += alpha * (inputGravity.x - gravity.x);
      gravity.y += alpha * (inputGravity.y - gravity.y);
    }
    filterSampleUs_ = lastSampleUs = sampleUs;
    lastFreshMs_ = millis();
    ++samples;
    return true;
  }

  void setMapping(bool swap, bool flipX, bool flipY) {
    if (swap_ == swap && flipX_ == flipX && flipY_ == flipY) return;
    swap_ = swap; flipX_ = flipX; flipY_ = flipY;
    resetFilter();
  }

  bool setMode(ResponseMode requested) {
    if (!validMode(requested)) return false;
    if (requested == mode) return true;
    if (online && !configure(address, requested)) {
      ++errors;
      disconnect(millis());
      return false;
    }
    mode = requested;
    lastPollUs_ = micros();
    if (online) lastFreshMs_ = millis();
    resetFilter();
    return true;
  }

  bool saveCalibration(const Calibration &candidate) {
    if (!validCalibration(candidate)) return false;
    CalibrationBlob blob = {};
    blob.magic = kMagic;
    blob.version = kVersion;
    blob.bytes = sizeof(blob);
    blob.bias[0] = candidate.accelBias.x;
    blob.bias[1] = candidate.accelBias.y;
    blob.bias[2] = candidate.accelBias.z;
    blob.bias[3] = candidate.gyroBias.x;
    blob.bias[4] = candidate.gyroBias.y;
    blob.bias[5] = candidate.gyroBias.z;
    blob.checksum = checksum(blob);
    Preferences storage;
    if (!storage.begin("motion-cal", false)) return false;
    // One NVS blob commit keeps all six offsets and their validity metadata
    // together. A rejected input or failed store leaves the RAM state intact.
    const bool stored = storage.putBytes("cal", &blob, sizeof(blob)) == sizeof(blob);
    storage.end();
    if (!stored) return false;
    calibration = candidate;
    calibrationLoaded_ = true;
    resetFilter();
    return true;
  }

  bool clearCalibration() {
    Preferences storage;
    if (!storage.begin("motion-cal", false)) return false;
    const bool removed = !storage.isKey("cal") || storage.remove("cal");
    storage.end();
    if (!removed) return false;
    calibration = Calibration();
    calibrationLoaded_ = true;
    resetFilter();
    return true;
  }

  bool loadCalibration() {
    // begin() tries this once. Reconnecting a cable must never overwrite a
    // current in-memory calibration or repeatedly reopen flash preferences.
    calibrationLoaded_ = true;
    Preferences storage;
    if (!storage.begin("motion-cal", true)) return false;
    CalibrationBlob blob = {};
    bool loaded = storage.getBytesLength("cal") == sizeof(blob);
    if (loaded) loaded = storage.getBytes("cal", &blob, sizeof(blob)) == sizeof(blob);
    storage.end();
    if (!loaded || blob.magic != kMagic || blob.version != kVersion ||
        blob.bytes != sizeof(blob) || blob.checksum != checksum(blob)) return false;
    const Calibration candidate(Vec3(blob.bias[0], blob.bias[1], blob.bias[2]),
                                Vec3(blob.bias[3], blob.bias[4], blob.bias[5]), true);
    if (!validCalibration(candidate)) return false;
    calibration = candidate;
    resetFilter();
    return true;
  }

 private:
  struct CalibrationBlob {
    uint32_t magic;
    uint16_t version, bytes;
    float bias[6];
    uint32_t checksum;
  };
  static_assert(sizeof(float) == 4 && sizeof(CalibrationBlob) == 36,
                "Calibration blob layout must remain fixed");
  static constexpr uint32_t kMagic = 0x4D43414CUL;
  static constexpr uint16_t kVersion = 1;
  TwoWire &wire_;
  const uint8_t compatibilityId_;
  bool started_ = false, calibrationLoaded_ = false;
  bool swap_ = false, flipX_ = false, flipY_ = false, firstFilter_ = true;
  uint32_t lastProbeMs_ = 0, lastFreshMs_ = 0, lastPollUs_ = 0, filterSampleUs_ = 0;
  Vec3 accelHistory_[3], gyroHistory_[3];
  uint8_t historyIndex_ = 0, historyCount_ = 0;

  static bool validMode(ResponseMode value) {
    return value == motion::Fast || value == motion::Raw || value == motion::Legacy;
  }
  static bool validCalibration(const Calibration &value) {
    return value.valid && isFinite(value.accelBias) && isFinite(value.gyroBias) &&
           norm(value.accelBias) <= 0.5f && norm(value.gyroBias) <= 60.0f;
  }
  static uint32_t checksum(const CalibrationBlob &blob) {
    const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&blob);
    uint32_t result = 2166136261UL;
    for (size_t i = 0; i < offsetof(CalibrationBlob, checksum); ++i)
      result = (result ^ bytes[i]) * 16777619UL;
    return result;
  }
  static int16_t word(const uint8_t *bytes) {
    return static_cast<int16_t>((static_cast<uint16_t>(bytes[0]) << 8) | bytes[1]);
  }
  static uint8_t packetRejectReason(const uint8_t *bytes) {
    uint8_t reason = 0;
    // Whole zero/FF payloads are common failed-bus patterns, not useful motion
    // measurements. Zero acceleration alone remains valid (real free fall).
    bool allZero = true, allHigh = true;
    for (int i = 1; i < 15; ++i) {
      allZero = allZero && bytes[i] == 0;
      allHigh = allHigh && bytes[i] == 0xFF;
    }
    if (allZero || allHigh) reason |= kRejectUniformPacket;
    // A broken transfer can leave its tail high while earlier acceleration
    // bytes look plausible. On this assembly a complete 0xFFFF gyro vector
    // accompanied false Z drops. Reject whole six-byte FF vectors, but permit
    // individual -1 ADC words and legitimate zero gyro/free-fall readings.
    bool accelHigh = true, gyroHigh = true;
    for (int i = 0; i < 6; ++i) {
      accelHigh = accelHigh && bytes[1 + i] == 0xFF;
      gyroHigh = gyroHigh && bytes[9 + i] == 0xFF;
    }
    if (accelHigh || gyroHigh) reason |= kRejectFloatingVector;
    // MPU-6000/6050 register map rev4.2, INT_STATUS(0x3A): bits1/2 reserved.
    // Compatibility devices must meet this register contract too. This check
    // only runs on a packet claiming DATA_RDY; no other interrupt is inferred.
    if (bytes[0] & 0x06) reason |= kRejectInterruptStatus;
    // A clipped axis has lost magnitude information. Hold the last usable
    // motion sample until a fresh in-range measurement arrives; never turn a
    // full-scale/bit-corrupted gyro value into integrated orientation.
    static const uint8_t axes[] = {1, 3, 5, 9, 11, 13};
    for (uint8_t offset : axes) {
      const int16_t value = word(bytes + offset);
      if (value == INT16_MIN || value == INT16_MAX) reason |= kRejectClippedAxis;
    }
    return reason;
  }
  static float median(float a, float b, float c) {
    if (a > b) { const float temporary = a; a = b; b = temporary; }
    if (b > c) b = c;
    return a > b ? a : b;
  }
  static Vec3 median(const Vec3 (&history)[3]) {
    return Vec3(median(history[0].x, history[1].x, history[2].x),
                median(history[0].y, history[1].y, history[2].y),
                median(history[0].z, history[1].z, history[2].z));
  }
  void qualifyMotion() {
    accelHistory_[historyIndex_] = accel;
    gyroHistory_[historyIndex_] = gyro;
    historyIndex_ = (historyIndex_ + 1) % 3;
    if (historyCount_ < 3) ++historyCount_;
    motionReady = historyCount_ == 3;
    if (motionReady) {
      motionAccel = median(accelHistory_);
      motionGyro = median(gyroHistory_);
    }
  }
  void resetFilter() {
    accel = correctAcceleration(rawAccel, calibration);
    gyro = correctGyro(rawGyro, calibration);
    motionAccel = motionGyro = Vec3();
    motionReady = false;
    historyIndex_ = historyCount_ = 0;
    inputGravity = mode == motion::Raw ? mapGravity(accel, swap_, flipX_, flipY_) : Vec2();
    gravity = inputGravity;
    firstFilter_ = true;
    filterSampleUs_ = 0;
  }
  void disconnect(uint32_t nowMs) {
    online = false;
    address = 0;
    lastProbeMs_ = nowMs;
    resetFilter();
  }
  bool readRegisters(uint8_t device, uint8_t reg, uint8_t *out, uint8_t count) {
    wire_.beginTransmission(device);
    wire_.write(reg);
    if (wire_.endTransmission(false) != 0) return false;
    if (wire_.requestFrom(device, count) != count) {
      while (wire_.available()) wire_.read();
      return false;
    }
    for (uint8_t i = 0; i < count; ++i) {
      const int value = wire_.read();
      if (value < 0) return false;
      out[i] = static_cast<uint8_t>(value);
    }
    return true;
  }
  bool writeRegister(uint8_t device, uint8_t reg, uint8_t value) {
    wire_.beginTransmission(device);
    wire_.write(reg);
    wire_.write(value);
    return wire_.endTransmission(true) == 0;
  }
  bool configure(uint8_t device, ResponseMode requested) {
    if (!validMode(requested)) return false;
    const uint8_t divider = requested == motion::Legacy ? 9 : 4;
    const uint8_t filter = requested == motion::Legacy ? 4 : 2;
    if (!writeRegister(device, 0x6C, 0x00) ||
        !writeRegister(device, 0x1A, filter) ||
        !writeRegister(device, 0x19, divider) ||
        !writeRegister(device, 0x1B, 0x08) ||
        !writeRegister(device, 0x1C, 0x08) ||
        !writeRegister(device, 0x38, 0x01)) return false;
    uint8_t settings[4], power[2], interrupt = 0;
    if (!readRegisters(device, 0x19, settings, sizeof(settings)) ||
        !readRegisters(device, 0x6B, power, sizeof(power)) ||
        !readRegisters(device, 0x38, &interrupt, 1)) return false;
    if (settings[0] != divider || (settings[1] & 0x07) != filter ||
        (settings[2] & 0x18) != 0x08 || (settings[3] & 0x18) != 0x08 ||
        (power[0] & 0x47) != 0x01 || (power[1] & 0x3F) != 0 ||
        (interrupt & 0x01) == 0) return false;
    // Discard any pre-configuration DATA_RDY latch. The next accepted sample
    // then belongs to the selected sample rate and range configuration.
    uint8_t status = 0;
    return readRegisters(device, 0x3A, &status, 1);
  }
};

}  // namespace motion

#endif
