#ifndef DBG_PRINT_H
#define DBG_PRINT_H

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

#define F(p) (p)

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
