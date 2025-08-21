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

void print  (const char v) { Serial.print(v); }
void println(const char v) { Serial.println(v); }
void print  (const bool v) { Serial.print(v ? 'T' : 'F'); }
void println(const bool v) { Serial.println(v ? 'T' : 'F'); }
void print  (const __FlashStringHelper *v) { Serial.print(v); }
void println(const __FlashStringHelper *v) { Serial.println(v); }
void print  (const char *v) { Serial.print(v); }
void println(const char *v) { Serial.println(v); }
void print  (const int v) { Serial.print(v); }
void println(const int v) { Serial.println(v); }
void print  (const unsigned int v) { Serial.print(v); }
void println(const unsigned int v) { Serial.println(v); }
void print  (const long v) { Serial.print(v); }
void println(const long v) { Serial.println(v); }
void print  (const unsigned long v) { Serial.print(v); }
void println(const unsigned long v) { Serial.println(v); }
void print  (const double v) { Serial.print(v); }
void println(const double v) { Serial.println(v); }

#else

#include <cstdio>

void print  (const char v) { printf("%c", v); }
void println(const char v) { printf("%c\n", v); }
void print  (const bool v) { printf("%c", v ? 'T' : 'F'); }
void println(const bool v) { printf("%c\n", v ? 'T' : 'F'); }
void print  (const char *v) { printf("%s", v); }
void println(const char *v) { printf("%s\n", v); }
void print  (const int v) { printf("%d", v); }
void println(const int v) { printf("%d\n", v); }
void print  (const unsigned int v) { printf("%u", v); }
void println(const unsigned int v) { printf("%u\n", v); }
void print  (const long v) { printf("%ld", v); }
void println(const long v) { printf("%ld\n", v); }
void print  (const unsigned long v) { printf("%lu", v); }
void println(const unsigned long v) { printf("%lu\n", v); }
void print  (const double v) { printf("%f", v); }
void println(const double v) { printf("%f\n", v); }

#endif //ARDUINO

#endif //DBG_PRINT_H
