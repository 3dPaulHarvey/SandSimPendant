#ifndef SIMCIRCLE_TEST_WIRE_H
#define SIMCIRCLE_TEST_WIRE_H

#include <array>
#include <stddef.h>
#include <stdint.h>
#include <vector>

// Small register-device model. Pointer writes, sequential reads, NACKs and
// DATA_RDY clearing behave independently of the code under test.
class TwoWire {
 public:
  std::array<uint8_t, 128> registers{};
  uint8_t deviceAddress = 0x68;
  bool present = true;
  bool failReads = false;
  int underflowAfter = -1; // Advertised count succeeds, but a read returns -1.
  int corruptReadRegister = -1;
  uint8_t corruptReadValue = 0;
  int rejectWriteRegister = -1;
  unsigned dataWrites = 0;

  explicit TwoWire(uint8_t identity = 0x68, uint8_t address = 0x68)
      : deviceAddress(address) {
    registers[0x75] = identity;
    registers[0x6B] = 0x40;
  }

  void beginTransmission(uint8_t address) {
    txAddress_ = address;
    tx_.clear();
    repeated_ = false;
  }
  size_t write(uint8_t value) {
    tx_.push_back(value);
    return 1;
  }
  uint8_t endTransmission(bool stop = true) {
    // ESP32 Wire defers the pointer transaction for a repeated start.
    if (!stop) { repeated_ = true; return 0; }
    return commitWrite();
  }
  size_t requestFrom(uint8_t address, uint8_t count) {
    rx_.clear();
    rxIndex_ = 0;
    if (repeated_) {
      repeated_ = false;
      if (address != txAddress_ || commitWrite() != 0) return 0;
    }
    if (!present || address != deviceAddress || failReads) return 0;
    for (uint8_t i = 0; i < count; ++i) {
      const uint8_t reg = pointer_++ & 0x7F;
      rx_.push_back(reg == corruptReadRegister ? corruptReadValue : registers[reg]);
      if (reg == 0x3A) registers[reg] = 0; // Reading interrupt status clears it.
    }
    return rx_.size();
  }
  int available() { return static_cast<int>(rx_.size() - rxIndex_); }
  int read() {
    if (underflowAfter >= 0 && rxIndex_ >= static_cast<size_t>(underflowAfter)) return -1;
    return rxIndex_ < rx_.size() ? rx_[rxIndex_++] : -1;
  }

  void publish(int16_t ax, int16_t ay, int16_t az,
               int16_t gx = 0, int16_t gy = 0, int16_t gz = 0) {
    putWord(0x3B, ax); putWord(0x3D, ay); putWord(0x3F, az);
    putWord(0x41, 0); // Temperature is skipped by this driver.
    putWord(0x43, gx); putWord(0x45, gy); putWord(0x47, gz);
    registers[0x3A] = 1;
  }

 private:
  uint8_t txAddress_ = 0, pointer_ = 0;
  bool repeated_ = false;
  std::vector<uint8_t> tx_, rx_;
  size_t rxIndex_ = 0;

  uint8_t commitWrite() {
    if (!present || txAddress_ != deviceAddress) return 2;
    if (tx_.empty()) return 0;
    pointer_ = tx_[0];
    if (tx_.size() > 1 && pointer_ == rejectWriteRegister) return 3;
    for (size_t i = 1; i < tx_.size(); ++i) {
      const uint8_t reg = pointer_++ & 0x7F;
      if (reg != 0x75) registers[reg] = tx_[i]; // Identity is read-only.
      ++dataWrites;
    }
    return 0;
  }
  void putWord(uint8_t reg, int16_t value) {
    const uint16_t bits = static_cast<uint16_t>(value);
    registers[reg] = static_cast<uint8_t>(bits >> 8);
    registers[reg + 1] = static_cast<uint8_t>(bits);
  }
};

extern TwoWire Wire;

#endif
