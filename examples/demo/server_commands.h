/**
 * ArduMon: Yet another Arduino serial command library.
 *
 * See https://github.com/martyvona/ArduMon/blob/main/README.md
 *
 * These are the ArduMon demo server command implementations.  This file is designed to be included only in demo.h.
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

/* server command handlers ********************************************************************************************/

#ifdef ARDUINO
//the Arduino platform defines strcmp_P() where both arguments are const char *
//this API originated from avr-libc: https://www.nongnu.org/avr-libc/user-manual/group__avr__pgmspace.html
//the second argument is assumed to be in progmem on AVR; non-AVR platforms just make this an alias to strcmp()
//but it can be more convenient for that arg to be a const __FlashStringHelper* made by the F() macro
#ifdef strcmp_P //some platforms like STM32 define strcmp_P as a macro
#undef strcmp_P
#define strcmp_P(a, b) strcmp((a), reinterpret_cast<const char *>(b))
#else //strcmp_P is not a macro: overload it instead
int strcmp_P(const char *a, const __FlashStringHelper *b) { return strcmp_P(a, reinterpret_cast<const char *>(b)); }
#endif
#endif //on native builds instead use strcmp_P() shim defined in native/arduino_shims.h

ArduMonTimer<AM> timer;

bool help(AM &am) { return am.sendCmds().endHandler(); }

bool setQuiet(AM &am) { return am.setTextEcho(false).setTextPrompt(static_cast<const char *>(0)).endHandler(); }

bool argc(AM &am) { return am.send(am.argc()).endHandler(); }

bool gcc(AM &am) { const char *name; return am.skip().recv(name).send(am.getCmdCode(name)).endHandler(); }

bool echoBool(AM &am) {
  bool v; if (!am.skip().recv(v)) return false;
  AM::BoolStyle style = AM::BoolStyle::TRUE_FALSE; bool upper_case = false;
  if (am.isTextMode()) {
    const uint8_t argc = am.argc();
    if (argc > 2) {
      const char *style_str;
      if (!am.recv(style_str)) return false;
      if      (strcmp_P(style_str, F("true_false")) == 0) style = AM::BoolStyle::TRUE_FALSE;
      else if (strcmp_P(style_str, F("tf"))         == 0) style = AM::BoolStyle::TF;
      else if (strcmp_P(style_str, F("yes_no"))     == 0) style = AM::BoolStyle::YES_NO;
      else if (strcmp_P(style_str, F("yn"))         == 0) style = AM::BoolStyle::YN;
      else return false;
    }
    if (argc > 3 && !am.recv(upper_case)) return false;
  }
  return am.send(v, style, upper_case).endHandler();
}

template <typename T> bool echoInt(AM &am) {
  T v = 0; if (!am.skip().recv(v)) return false;
  bool hex = false, pad_zero = false, pad_right = false; uint8_t width = 0;
  if (am.isTextMode()) {
    const uint8_t argc = am.argc();
    if (argc > 2 && !am.recv(hex)) return false;
    if (argc > 3 && !am.recv(width)) return false;
    if (argc > 4 && !am.recv(pad_zero)) return false;
    if (argc > 5 && !am.recv(pad_right)) return false;
    if (width > 31) width = 31;
  }
  const uint8_t fmt =
    width | (hex ? AM::FMT_HEX : 0) | (pad_zero ? AM::FMT_PAD_ZERO : 0) | (pad_right ? AM::FMT_PAD_RIGHT : 0);
  if (!am.isBinaryMode() && pad_right && !pad_zero) return am.sendRaw(v, fmt).sendChar('|').endHandler();
  else return am.send(v, fmt).endHandler();
}

template <typename T> bool echoFloatOrDouble(AM &am) {
  T v = 0; if (!am.skip().recv(v)) return false;
  bool scientific = false; int8_t precision = -1, width = -1;
  if (am.isTextMode()) {
    const uint8_t argc = am.argc();
    if (argc > 2 && !am.recv(scientific)) return false;
    if (argc > 3 && !am.recv(precision)) return false;
    if (argc > 4 && !am.recv(width)) return false;
  }
  return am.send(v, scientific, precision, width).endHandler();
}

bool echoChar(AM &am) { char v = 0; return am.skip().recvChar(v).sendChar(v).endHandler(); }
bool echoStr(AM &am) { const char* v = 0; return am.skip().recv(v).send(v).endHandler(); }
bool echoU8(AM &am) { return echoInt<uint8_t>(am); }
bool echoS8(AM &am) { return echoInt<int8_t>(am); }
bool echoU16(AM &am) { return echoInt<uint16_t>(am); }
bool echoS16(AM &am) { return echoInt<int16_t>(am); }
bool echoU32(AM &am) { return echoInt<uint32_t>(am); }
bool echoS32(AM &am) { return echoInt<int32_t>(am); }
#ifdef WITH_INT64
bool echoU64(AM &am) { return echoInt<uint64_t>(am); }
bool echoS64(AM &am) { return echoInt<int64_t>(am); }
#endif
#ifdef WITH_FLOAT
bool echoFloat(AM &am) { return echoFloatOrDouble<float>(am); }
#ifdef WITH_DOUBLE
bool echoDouble(AM &am) { return echoFloatOrDouble<double>(am); }
#endif
#endif

bool echoMultiple(AM &am) {

  if (!am.skip()) return false; // skip command token/code
  
  const char* format;
  if (!am.recv(format)) return false;
  
  const size_t len = strlen(format);
  for (size_t i = 0; i + 2 < len; i += 3) {

    const char type[4] = {format[i], format[i+1], format[i+2], '\0'};

    if      (strcmp_P(type, F("chr")) == 0) { char v;        if (!am.recvChar(v).sendChar(v)) return false; }
    else if (strcmp_P(type, F("str")) == 0) { const char* v; if (!am.recv(v).send(v)) return false; }
    else if (strcmp_P(type, F("bll")) == 0) { bool v;        if (!am.recv(v).send(v)) return false; }
    else if (strcmp_P(type, F("u08")) == 0) { uint8_t v;     if (!am.recv(v).send(v)) return false; }
    else if (strcmp_P(type, F("i08")) == 0) { int8_t v;      if (!am.recv(v).send(v)) return false; }
    else if (strcmp_P(type, F("u16")) == 0) { uint16_t v;    if (!am.recv(v).send(v)) return false; }
    else if (strcmp_P(type, F("i16")) == 0) { int16_t v;     if (!am.recv(v).send(v)) return false; }
    else if (strcmp_P(type, F("u32")) == 0) { uint32_t v;    if (!am.recv(v).send(v)) return false; }
    else if (strcmp_P(type, F("i32")) == 0) { int32_t v;     if (!am.recv(v).send(v)) return false; }
#ifdef WITH_INT64
    else if (strcmp_P(type, F("u64")) == 0) { uint64_t v;    if (!am.recv(v).send(v)) return false; }
    else if (strcmp_P(type, F("i64")) == 0) { int64_t v;     if (!am.recv(v).send(v)) return false; }
#endif
#ifdef WITH_FLOAT
    else if (strcmp_P(type, F("f32")) == 0) { float v;       if (!am.recv(v).send(v)) return false; }
#ifdef WITH_DOUBLE
    else if (strcmp_P(type, F("f64")) == 0) { double v;      if (!am.recv(v).send(v)) return false; }
#endif
    else return false;
#endif
  }
  return am.endHandler();
}

float float_param = 0;
bool setFloatParam(AM &am) { return am.skip().recv(float_param).endHandler(); }
bool getFloatParam(AM &am) { return am.skip().send(float_param).endHandler(); }

bool quit(AM &am) {
  print(am.isBinaryMode() ? F("binary") : F("text")); print(F(" server done, "));
  print(num_errors); print(F(" total errors")); println();
  demo_done = true;
  return true;
}

void addCmds() {

#define ADD_CMD(func, name, desc) \
  if (!am.addCmd((func), F(name), F(desc))) { print(AM::errMsg(am.clearErr())); println(); }

  ADD_CMD(gcc, "gcc", "name | get command code");
  ADD_CMD(help, "help", "show commands");
  ADD_CMD(setQuiet, "quiet", "disable text echo and prompt");
  ADD_CMD(argc, "argc", "show arg count");
  ADD_CMD(&(timer.start_cmd), "ts", "hours mins secs [accel [sync_throttle_ms|-1 [bin_response_code]]] | start timer");
  ADD_CMD(&(timer.stop_cmd), "to", "stop timer");
  ADD_CMD(&(timer.get_cmd), "tg", "get timer");
  ADD_CMD(echoChar, "ec", "arg | echo char");
  ADD_CMD(echoStr, "es", "arg | echo str");
  ADD_CMD(echoBool, "eb", "arg [style [upper_case]] | echo bool");
  ADD_CMD(echoU8, "eu8", "arg [hex [width [pad_zero [pad_right]]]] | echo uint8");
  ADD_CMD(echoS8, "es8", "arg [hex [width [pad_zero [pad_right]]]] | echo int8");
  ADD_CMD(echoU16, "eu16", "arg [hex [width [pad_zero [pad_right]]]] | echo uint16");
  ADD_CMD(echoS16, "es16", "arg [hex [width [pad_zero [pad_right]]]] | echo int16");
  ADD_CMD(echoU32, "eu32", "arg [hex [width [pad_zero [pad_right]]]] | echo uint32");
  ADD_CMD(echoS32, "es32", "arg [hex [width [pad_zero [pad_right]]]] | echo int32");
#ifdef WITH_INT64
  ADD_CMD(echoU64, "eu64", "arg [hex [width [pad_zero [pad_right]]]] | echo uint64");
  ADD_CMD(echoS64, "es64", "arg [hex [width [pad_zero [pad_right]]]] | echo int64");
#endif
#ifdef WITH_FLOAT
  ADD_CMD(echoFloat, "ef", "arg [scientific [precision [width]]] | echo float");
#ifdef WITH_DOUBLE
  ADD_CMD(echoDouble, "ed", "arg [scientific [precision [width]]] | echo double");
#endif
#endif
  ADD_CMD(echoMultiple, "em", "format_string args... | echo multiple args based on format");
  ADD_CMD(setFloatParam, "sfp", "arg | set float param");
  ADD_CMD(getFloatParam, "gfp", "get float param");
  ADD_CMD(quit, "quit", "quit");

#undef ADD_CMD
}

