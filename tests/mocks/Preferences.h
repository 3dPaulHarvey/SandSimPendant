#ifndef SIMCIRCLE_TEST_PREFERENCES_H
#define SIMCIRCLE_TEST_PREFERENCES_H

#include <stddef.h>

// These driver tests model a board without a saved calibration. Persistence
// and calibration math are outside this fake's scope.
class Preferences {
 public:
  bool begin(const char *, bool = false) { return false; }
  void end() {}
  size_t getBytesLength(const char *) { return 0; }
  size_t getBytes(const char *, void *, size_t) { return 0; }
  size_t putBytes(const char *, const void *, size_t) { return 0; }
  bool isKey(const char *) { return false; }
  bool remove(const char *) { return false; }
};

#endif
