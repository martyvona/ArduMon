#ifndef DBG_PRINT_H
#define DBG_PRINT_H

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

#ifdef ARDUINO

//always using printf() would be nicer, and should almost work; however it doesn't support float by default on AVR

void println() { Serial.println(); }
void print(const char v) { Serial.print(v); }
void print(const bool v) { Serial.print(v ? 'T' : 'F'); }
void print(const __FlashStringHelper *v) { Serial.print(v); }
void print(const char *v) { Serial.print(v); }
void print(const int v) { Serial.print(v); }
void print(const unsigned int v) { Serial.print(v); }
void print(const long v) { Serial.print(v); }
void print(const unsigned long v) { Serial.print(v); }
void print(const long long v) {
  //Arduino Serial does not implement print(int64_t)
  //ArduMon does have its own implementation which can do it but it's not worth depending on that here
  if (v >= INT32_MIN && v <= INT32_MAX) Serial.print(static_cast<int32_t>(v));
  else Serial.print(F("TOOBIG"));
}
void print(const unsigned long long v) {
  //Arduino Serial does not implement print(uint64_t)
  //ArduMon does have its own implementation which can do it but it's not worth depending on that here
  if (v <= UINT32_MAX) Serial.print(static_cast<uint32_t>(v));
  else Serial.print(F("TOOBIG"));
}
void print(const double v) { Serial.print(v); }

#else

#include <cstdio>

void println() { printf("\n"); }
void print(const char v) { printf("%c", v); }
void print(const bool v) { printf("%c", v ? 'T' : 'F'); }
void print(const char *v) { printf("%s", v); }
void print(const int v) { printf("%d", v); }
void print(const unsigned int v) { printf("%u", v); }
void print(const long v) { printf("%ld", v); }
void print(const unsigned long v) { printf("%lu", v); }
void print(const long long v) { printf("%lld", v); }
void print(const unsigned long long v) { printf("%llu", v); }
void print(const double v) { printf("%f", v); }

#endif //ARDUINO

#endif //DBG_PRINT_H
