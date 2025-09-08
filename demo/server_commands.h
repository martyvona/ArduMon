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

AM_Timer<AM> timer;

bool help(AM &am) { return am.send_cmds().end_handler(); }

bool quiet(AM &am) { return am.set_txt_echo(false).set_txt_prompt(0).end_handler(); }

bool argc(AM &am) { return am.send(am.argc()).end_handler(); }

bool gcc(AM &am) { const char *name; return am.skip().recv(name).send(am.get_cmd_code(name)).end_handler(); }

bool echo_bool(AM &am) {
  bool v; if (!am.skip().recv(v)) return false;
  AM::BoolStyle style = AM::BoolStyle::TRUE_FALSE; bool upper_case = false;
  if (am.is_text_mode()) {
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
  return am.send(v, style, upper_case).end_handler();
}

template <typename T> bool echo_int(AM &am) {
  T v = 0; if (!am.skip().recv(v)) return false;
  bool hex = false, pad_zero = false, pad_right = false; uint8_t width = 0;
  if (am.is_text_mode()) {
    const uint8_t argc = am.argc();
    if (argc > 2 && !am.recv(hex)) return false;
    if (argc > 3 && !am.recv(width)) return false;
    if (argc > 4 && !am.recv(pad_zero)) return false;
    if (argc > 5 && !am.recv(pad_right)) return false;
    if (width > 31) width = 31;
  }
  const uint8_t fmt =
    width | (hex ? AM::FMT_HEX : 0) | (pad_zero ? AM::FMT_PAD_ZERO : 0) | (pad_right ? AM::FMT_PAD_RIGHT : 0);
  if (!am.is_binary_mode() && pad_right && !pad_zero) return am.send_raw(v, fmt).send('|').end_handler();
  else return am.send(v, fmt).end_handler();
}

template <typename T> bool echo_flt(AM &am) {
  T v = 0; if (!am.skip().recv(v)) return false;
  bool scientific = false; int8_t precision = -1, width = -1;
  if (am.is_text_mode()) {
    const uint8_t argc = am.argc();
    if (argc > 2 && !am.recv(scientific)) return false;
    if (argc > 3 && !am.recv(precision)) return false;
    if (argc > 4 && !am.recv(width)) return false;
  }
  return am.send(v, scientific, precision, width).end_handler();
}

bool echo_char(AM &am) { char v = 0; return am.skip().recv_char(v).send_char(v).end_handler(); }
bool echo_str(AM &am) { const char* v = 0; return am.skip().recv(v).send(v).end_handler(); }
bool echo_u8(AM &am) { return echo_int<uint8_t>(am); }
bool echo_s8(AM &am) { return echo_int<int8_t>(am); }
bool echo_u16(AM &am) { return echo_int<uint16_t>(am); }
bool echo_s16(AM &am) { return echo_int<int16_t>(am); }
bool echo_u32(AM &am) { return echo_int<uint32_t>(am); }
bool echo_s32(AM &am) { return echo_int<int32_t>(am); }
#ifdef WITH_INT64
bool echo_u64(AM &am) { return echo_int<uint64_t>(am); }
bool echo_s64(AM &am) { return echo_int<int64_t>(am); }
#endif
#ifdef WITH_FLOAT
bool echo_float(AM &am) { return echo_flt<float>(am); }
#ifdef WITH_DOUBLE
bool echo_double(AM &am) { return echo_flt<double>(am); }
#endif
#endif

bool echo_multiple(AM &am) {

  if (!am.skip()) return false; // skip command token/code
  
  const char* format;
  if (!am.recv(format)) return false;
  
  const size_t len = strlen(format);
  for (size_t i = 0; i + 2 < len; i += 3) {

    const char type[4] = {format[i], format[i+1], format[i+2], '\0'};

    if      (strcmp_P(type, F("chr")) == 0) { char v;        if (!am.recv_char(v).send(v)) return false; }
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
  return am.end_handler();
}

float float_param = 0;
bool set_float_param(AM &am) { return am.skip().recv(float_param).end_handler(); }
bool get_float_param(AM &am) { return am.skip().send(float_param).end_handler(); }

bool quit(AM &am) {
  print(am.is_binary_mode() ? F("binary") : F("text")); print(F(" server done, "));
  print(num_errors); print(F(" total errors")); println();
  demo_done = true;
  return true;
}

void add_cmds() {

#define ADD_CMD(func, name, desc) \
  if (!am.add_cmd((func), F(name), F(desc))) { print(AM::err_msg(am.clear_err())); println(); }

  ADD_CMD(gcc, "gcc", "name | get command code");
  ADD_CMD(help, "help", "show commands");
  ADD_CMD(quiet, "quiet", "disable text echo and prompt");
  ADD_CMD(argc, "argc", "show arg count");
  ADD_CMD(&(timer.start_cmd), "ts", "hours mins secs [accel [sync_throttle_ms|-1 [bin_response_code]]] | start timer");
  ADD_CMD(&(timer.stop_cmd), "to", "stop timer");
  ADD_CMD(&(timer.get_cmd), "tg", "get timer");
  ADD_CMD(echo_char, "ec", "arg | echo char");
  ADD_CMD(echo_str, "es", "arg | echo str");
  ADD_CMD(echo_bool, "eb", "arg [style [upper_case]] | echo bool");
  ADD_CMD(echo_u8, "eu8", "arg [hex [width [pad_zero [pad_right]]]] | echo uint8");
  ADD_CMD(echo_s8, "es8", "arg [hex [width [pad_zero [pad_right]]]] | echo int8");
  ADD_CMD(echo_u16, "eu16", "arg [hex [width [pad_zero [pad_right]]]] | echo uint16");
  ADD_CMD(echo_s16, "es16", "arg [hex [width [pad_zero [pad_right]]]] | echo int16");
  ADD_CMD(echo_u32, "eu32", "arg [hex [width [pad_zero [pad_right]]]] | echo uint32");
  ADD_CMD(echo_s32, "es32", "arg [hex [width [pad_zero [pad_right]]]] | echo int32");
#ifdef WITH_INT64
  ADD_CMD(echo_u64, "eu64", "arg [hex [width [pad_zero [pad_right]]]] | echo uint64");
  ADD_CMD(echo_s64, "es64", "arg [hex [width [pad_zero [pad_right]]]] | echo int64");
#endif
#ifdef WITH_FLOAT
  ADD_CMD(echo_float, "ef", "arg [scientific [precision [width]]] | echo float");
#ifdef WITH_DOUBLE
  ADD_CMD(echo_double, "ed", "arg [scientific [precision [width]]] | echo double");
#endif
#endif
  ADD_CMD(echo_multiple, "em", "format_string args... | echo multiple args based on format");
  ADD_CMD(set_float_param, "sfp", "arg | set float param");
  ADD_CMD(get_float_param, "gfp", "get float param");
  ADD_CMD(quit, "quit", "quit");

#undef ADD_CMD
}

