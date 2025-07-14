#ifndef ARDUMON_BASE_H
#define ARDUMON_BASE_H

/**
 * ArduMon: Yet another Arduino serial command library.
 *
 * See https://github.com/martyvona/ArduMon/blob/main/README.md
 *
 * Copyright 2025 Marsette A. Vona (martyvona@gmail.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the “Software”), to deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>

#ifndef ARDUINO
#include <sys/time.h>
#endif

#if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "only little endian architectures are supported"
#endif

class ArduMonBase {
public:

  typedef unsigned long millis_t;

#ifndef ARDUINO
  class Stream {
  public:
    virtual ~Stream() {}
    virtual int16_t available() = 0;
    virtual int16_t read() = 0; //-1 if no data available
    virtual int16_t availableForWrite() = 0;
    virtual uint16_t write(uint8_t byte) = 0; //returns 1
  };
#endif

protected:

  static bool copy_bytes(const void *src, void *dest, const uint8_t num_bytes) {
    for (uint8_t i = 0; i < num_bytes; i++) {
      reinterpret_cast<uint8_t*>(dest)[i] = reinterpret_cast<const uint8_t*>(src)[i];
    }
    return true;
  }

#ifndef ARDUINO

  //shims for building on native host, currently supports OS X and Linux

  template <typename T> static T pgm_read_byte(const T *a) { return *a; }

  static int strcmp_P(const char *a, const char *b) { return strcmp(a, b); }

  static const char *PSTR(const char *p) { return p; }

  uint32_t millis() {
    struct timeval tv;
    gettimeofday(&tv,NULL);
    uint64_t now_ms = tv.tv_sec * 1000 + tv.tv_usec / 1000;
    static uint64_t start_ms = now_ms;
    return now_ms - start_ms;
  }

  void delayMicroseconds(uint16_t us) {
    struct timeval tv;
    gettimeofday(&tv,NULL);
    uint64_t start_us = tv.tv_sec * 1000000 + tv.tv_usec, now_us;
    do {
      gettimeofday(&tv,NULL);
      now_us = tv.tv_sec * 1000000 + tv.tv_usec;
    } while (now_us - start_us < us);
  }
#endif
};

#endif
