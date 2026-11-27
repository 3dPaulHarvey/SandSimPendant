#include "../shared/MPU6050Motion.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

TwoWire Wire;
static uint64_t fakeTimeUs = 1000000;
uint32_t micros() { return static_cast<uint32_t>(fakeTimeUs); }
uint32_t millis() { return static_cast<uint32_t>(fakeTimeUs / 1000); }
void delay(uint32_t milliseconds) { fakeTimeUs += milliseconds * uint64_t(1000); }
static void advanceUs(uint32_t microseconds) { fakeTimeUs += microseconds; }
static bool near(float actual, float expected) { return fabsf(actual - expected) < 0.0001f; }

static void identityPolicy() {
  // Strict default never configures a responding but unsupported device.
  TwoWire nonstandard(0x75);
  motion::MPU6050Motion strict(nonstandard);
  assert(!strict.begin());
  assert(!strict.online && strict.address == 0 && strict.identity == 0x75);
  assert(!strict.compatibilityMode() && nonstandard.dataWrites == 0);

  // Explicit selection permits only that ID, after normal configuration checks.
  motion::MPU6050Motion compatible(nonstandard, 0x75);
  assert(compatible.begin());
  assert(compatible.online && compatible.address == 0x68);
  assert(compatible.identity == 0x75 && compatible.compatibilityMode());
  assert(nonstandard.dataWrites > 0);

  for (uint8_t address : {uint8_t(0x68), uint8_t(0x69)}) {
    TwoWire genuine(0x68, address);
    motion::MPU6050Motion standard(genuine);
    assert(standard.begin() && standard.address == address);
    assert(standard.identity == 0x68 && !standard.compatibilityMode());
    motion::MPU6050Motion stillStandard(genuine, 0x75);
    assert(stillStandard.begin() && !stillStandard.compatibilityMode());
  }

  for (uint8_t identity : {uint8_t(0), uint8_t(0x70), uint8_t(0x74), uint8_t(0xFF)}) {
    TwoWire unrelated(identity);
    motion::MPU6050Motion selected(unrelated, 0x75);
    assert(!selected.begin() && !selected.online);
    assert(selected.identity == identity && unrelated.dataWrites == 0);
  }
  TwoWire zeroIdentity(0);
  motion::MPU6050Motion zeroIsDisabled(zeroIdentity);
  assert(!zeroIsDisabled.begin() && zeroIdentity.dataWrites == 0);

  TwoWire absent(0x75);
  absent.present = false;
  motion::MPU6050Motion missing(absent, 0x75);
  assert(!missing.begin() && missing.identity == 0 && !missing.online);

  // A selected device on the alternate AD0 address remains supported.
  TwoWire alternate(0x75, 0x69);
  motion::MPU6050Motion alternateDriver(alternate, 0x75);
  assert(alternateDriver.begin() && alternateDriver.address == 0x69);
}

static void configurationFailures() {
  TwoWire badReadback(0x75);
  badReadback.corruptReadRegister = 0x1C;
  badReadback.corruptReadValue = 0;
  motion::MPU6050Motion mismatched(badReadback, 0x75);
  assert(!mismatched.begin());
  assert(!mismatched.online && mismatched.identity == 0x75);
  assert(mismatched.errors > 0 && !mismatched.compatibilityMode());

  TwoWire failedWake(0x75);
  failedWake.rejectWriteRegister = 0x6B;
  motion::MPU6050Motion writeFailure(failedWake, 0x75);
  assert(!writeFailure.begin() && writeFailure.errors > 0);
  assert(!writeFailure.online && writeFailure.identity == 0x75);

  TwoWire failedIdentityRead(0x75);
  failedIdentityRead.failReads = true;
  motion::MPU6050Motion readFailure(failedIdentityRead, 0x75);
  assert(!readFailure.begin() && readFailure.identity == 0);
  assert(!readFailure.online && failedIdentityRead.dataWrites == 0);
}

static void sampleFreshness() {
  TwoWire device(0x75);
  device.publish(8192, 0, 0); // A pre-configuration sample must be discarded.
  motion::MPU6050Motion imu(device, 0x75);
  assert(imu.begin());
  advanceUs(5000);
  assert(!imu.update() && imu.samples == 0);

  device.publish(8192, -4096, 2048, 655, -655, 0);
  advanceUs(4999);
  assert(!imu.update() && imu.samples == 0);
  advanceUs(1);
  assert(imu.update() && imu.samples == 1);
  assert(near(imu.rawAccel.x, 1) && near(imu.rawAccel.y, -0.5f));
  assert(near(imu.rawAccel.z, 0.25f));
  assert(near(imu.rawGyro.x, 10) && near(imu.rawGyro.y, -10));
  const uint32_t firstSampleUs = imu.lastSampleUs;

  advanceUs(5000);
  assert(!imu.update() && imu.samples == 1 && imu.lastSampleUs == firstSampleUs);
  assert(imu.errors == 0); // Same registers without DATA_RDY are not a new sample.

  device.publish(0, 0, 8192);
  device.failReads = true;
  advanceUs(5000);
  assert(!imu.update() && imu.samples == 1 && imu.errors == 1);
  assert(imu.lastSampleUs == firstSampleUs);
  device.failReads = false;
  advanceUs(5000);
  assert(imu.update() && imu.samples == 2 && near(imu.accel.z, 1));

  advanceUs(250000);
  assert(!imu.update() && !imu.online && imu.address == 0);
  assert(imu.identity == 0x75 && !imu.compatibilityMode());
  assert(imu.samples == 2 && imu.errors == 2);

  // Recovery itself does not count a sample; only a fresh DATA_RDY does.
  advanceUs(2000000);
  assert(!imu.update() && imu.online && imu.compatibilityMode());
  assert(imu.samples == 2);
  device.publish(-8192, 0, 0);
  advanceUs(5000);
  assert(imu.update() && imu.samples == 3 && near(imu.accel.x, -1));
}

static void motionQualification() {
  TwoWire device(0x75);
  motion::MPU6050Motion imu(device, 0x75);
  assert(imu.begin() && !imu.motionReady);
  // A single corrupt startup reading cannot establish the motion reference.
  device.publish(24576, 0, 0, 19650, 0, 0);
  advanceUs(5000);
  assert(imu.update() && !imu.motionReady && imu.samples == 1);
  assert(near(imu.rawAccel.x, 3) && near(imu.rawGyro.x, 300));
  assert(near(imu.gravity.x, 0) && near(imu.gravity.y, 0));
  for (int i = 0; i < 2; ++i) {
    device.publish(8192, 0, 0);
    advanceUs(5000);
    assert(imu.update());
    assert(imu.motionReady == (i == 1));
  }
  assert(near(imu.motionAccel.x, 1) && near(imu.motionGyro.x, 0));
  assert(near(imu.gravity.x, -1));

  // Every accepted raw reading remains available to calibration/diagnostics.
  // An isolated interior spike disappears from both qualified vectors.
  for (int i = 0; i < 4; ++i) {
    device.publish(i == 1 ? -24576 : 8192, 0, 0, i == 1 ? -19650 : 0);
    advanceUs(5000);
    assert(imu.update() && imu.motionReady);
    assert(near(imu.rawAccel.x, i == 1 ? -3.0f : 1.0f));
    assert(near(imu.accel.x, imu.rawAccel.x));
    assert(near(imu.motionAccel.x, 1) && near(imu.motionGyro.x, 0));
    assert(near(imu.gravity.x, -1));
  }

  // Real sustained changes pass after one sample of latency on both sensors.
  for (int i = 0; i < 2; ++i) {
    device.publish(0, 8192, 0, 0, 655, 0);
    advanceUs(5000);
    assert(imu.update());
    assert(near(imu.rawAccel.y, 1) && near(imu.rawGyro.y, 10));
    assert(near(imu.motionAccel.y, i == 0 ? 0 : 1));
    assert(near(imu.motionGyro.y, i == 0 ? 0 : 10));
  }
  assert(near(imu.inputGravity.x, 0) && near(imu.inputGravity.y, 1));

  // RAW diagnostic mode keeps immediate gravity, despite qualification warmup.
  assert(imu.setMode(motion::Raw) && !imu.motionReady);
  device.publish(-8192, 0, 0);
  advanceUs(5000);
  assert(imu.update() && !imu.motionReady);
  assert(near(imu.gravity.x, 1));
  imu.setMapping(true, true, false);
  assert(!imu.motionReady);
  device.publish(0, -8192, 0);
  advanceUs(5000);
  assert(imu.update() && near(imu.gravity.x, -1));
}

static void corruptPackets() {
  TwoWire device(0x75);
  motion::MPU6050Motion imu(device, 0x75);
  assert(imu.begin());
  for (int i = 0; i < 3; ++i) {
    device.publish(0, 0, 8192);
    advanceUs(5000);
    assert(imu.update());
  }
  assert(imu.motionReady);
  const uint32_t sampleUs = imu.lastSampleUs;
  uint32_t rejected = 0;
  auto reject = [&]() {
    advanceUs(5000);
    assert(!imu.update());
    assert(imu.rejectedSamples == ++rejected && imu.samples == 3);
    assert(imu.errors == rejected && imu.lastSampleUs == sampleUs);
    assert(near(imu.accel.z, 1) && near(imu.motionAccel.z, 1));
  };
  device.publish(0, 0, 0);
  reject();
  assert(imu.lastRejectReason == motion::MPU6050Motion::kRejectUniformPacket);
  for (int reg = 0x3B; reg <= 0x48; ++reg) device.registers[reg] = 0xFF;
  device.registers[0x3A] = 1;
  reject();
  assert(imu.lastRejectReason == (motion::MPU6050Motion::kRejectUniformPacket |
                                 motion::MPU6050Motion::kRejectFloatingVector));
  // A plausible acceleration prefix cannot make an all-FF gyro tail valid.
  device.publish(-819, -82, -1, -1, -1, -1);
  reject();
  assert(imu.lastRejectReason == motion::MPU6050Motion::kRejectFloatingVector);
  assert(imu.lastRejectedPacket[5] == 0xFF && imu.lastRejectedPacket[14] == 0xFF);
  assert(imu.lastRejectedPacket[1] != 0xFF);
  device.publish(-1, -1, -1, -33, 72, -7);
  reject();
  assert(imu.lastRejectReason == motion::MPU6050Motion::kRejectFloatingVector);
  for (uint8_t status : {uint8_t(0x03), uint8_t(0x05), uint8_t(0x07)}) {
    device.publish(0, 0, 8192);
    device.registers[0x3A] = status;
    reject();
    assert(imu.lastRejectReason == motion::MPU6050Motion::kRejectInterruptStatus);
    assert(imu.lastRejectedPacket[0] == status);
  }
  for (int axis = 0; axis < 6; ++axis) {
    for (int16_t limit : {int16_t(INT16_MIN), int16_t(INT16_MAX)}) {
      int16_t values[] = {0, 0, 8192, 0, 0, 0};
      values[axis] = limit;
      device.publish(values[0], values[1], values[2], values[3], values[4], values[5]);
      reject();
      assert(imu.lastRejectReason == motion::MPU6050Motion::kRejectClippedAxis);
    }
  }
  // An advertised successful read cannot silently substitute 0xFF for -1.
  device.publish(0, 0, 8192);
  device.underflowAfter = 3;
  advanceUs(5000);
  assert(!imu.update() && imu.samples == 3 && imu.errors == rejected + 1);
  assert(imu.rejectedSamples == rejected && imu.lastSampleUs == sampleUs);
  device.underflowAfter = -1;

  // Real free fall is permitted: only the whole all-zero packet is rejected.
  device.publish(0, 0, 0);
  device.registers[0x42] = 1; // A nonzero temperature register, no gravity/gyro.
  advanceUs(5000);
  assert(imu.update() && imu.samples == 4 && near(imu.rawAccel.z, 0));
  assert(imu.rejectedSamples == rejected);
  // A single -1 word, or two -1 axes, is a possible quantized real reading.
  // Only a full three-axis FF vector is classified as a floating-bus pattern.
  device.publish(-1, 0, 8192, 100, -1, -1);
  advanceUs(5000);
  assert(imu.update() && imu.samples == 5 && imu.rejectedSamples == rejected);
  // The reserved-bit rule only qualifies claimed fresh samples.
  device.registers[0x3A] = 0x06;
  advanceUs(5000);
  assert(!imu.update() && imu.samples == 5 && imu.rejectedSamples == rejected);
  // Corrupt traffic never prevents stale disconnection or survives reconnect.
  device.publish(0, 0, 0);
  advanceUs(250000);
  assert(!imu.update() && !imu.online && !imu.motionReady);
  advanceUs(2000000);
  assert(!imu.update() && imu.online && !imu.motionReady);
  for (int i = 0; i < 3; ++i) {
    device.publish(8192, 0, 0);
    advanceUs(5000);
    assert(imu.update());
    assert(imu.motionReady == (i == 2));
  }
  assert(near(imu.motionAccel.x, 1) && near(imu.motionAccel.z, 0));
}

int main() {
  identityPolicy();
  configurationFailures();
  sampleFreshness();
  motionQualification();
  corruptPackets();
  puts("MPU driver: strict/explicit identities, alternate address, configuration rejection,");
  puts("coherent scaled samples, DATA_RDY uniqueness, failed reads, stale disconnect and recovery pass.");
  puts("Median qualification preserves raw diagnostics, rejects isolated spikes, and warms up safely;");
  puts("zero/FF bursts, partial FF vectors, reserved status, clipped axes and read underflow");
  puts("are rejected without refreshing stale state; raw rejected packets remain diagnosable.");
}
