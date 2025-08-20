#ifndef ARDUINO_SHIMS_H
#define ARDUINO_SHIMS_H

#include <chrono>
#include <thread>

//shims for building on native host, currently supports OS X and Linux including WSL

typedef char __FlashStringHelper;

static const __FlashStringHelper *F(const char *p) { return p; }

template <typename T> static const T pgm_read_byte(const T *p) { return *p; }

static int strcmp_P(const char *a, const char *b) { return strcmp(a, b); }

uint64_t millis() {
  static const auto start = std::chrono::steady_clock::now();
  auto now = std::chrono::steady_clock::now();
  return std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
}

void delayMicroseconds(uint16_t us) { std::this_thread::sleep_for(std::chrono::microseconds(us)); }

#endif //ARDUINO_SHIMS_H
