#ifndef ARDUINO_SHIMS_H
#define ARDUINO_SHIMS_H

#include <time.h>

//shims for building on native host, currently supports OS X and Linux including WSL

typedef char __FlashStringHelper;

static const __FlashStringHelper *F(const char *p) { return p; }

template <typename T> static const T pgm_read_byte(const T *p) { return *p; }

static int strcmp_P(const char *a, const char *b) { return strcmp(a, b); }

uint64_t millis() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  uint64_t now_ms = ts.tv_sec * 1000ul + ts.tv_nsec / 1000000ul;
  static uint64_t start_ms = now_ms;
  return now_ms - start_ms;
}

void delayMicroseconds(uint16_t us) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  uint64_t start_us = ts.tv_sec * 1000000ul + ts.tv_nsec / 1000ul, now_us;
  do {
    clock_gettime(CLOCK_MONOTONIC, &ts);
    now_us = ts.tv_sec * 1000000ul + ts.tv_nsec / 1000ul;
  } while (now_us - start_us < us);
}

#endif //ARDUINO_SHIMS_H
