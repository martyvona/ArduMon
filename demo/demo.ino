#include <ArduMon.h>

//#define BASELINE_MEM

#ifndef WITH_INT64
#define WITH_INT64 true
#endif

#ifndef WITH_FLOAT
#define WITH_FLOAT true
#endif

#ifndef WITH_DOUBLE
#define WITH_DOUBLE true
#endif

#ifndef WITH_BINARY
#define WITH_BINARY true
#endif

#ifndef WITH_TEXT
#define WITH_TEXT true
#endif

#ifndef BAUD
#define BAUD 115200
#endif

#ifndef MAX_CMDS
#define MAX_CMDS 32
#endif

#ifndef RECV_BUF_SZ
#define RECV_BUF_SZ 128
#endif

#ifndef SEND_BUF_SZ
#define SEND_BUF_SZ 128
#endif

typedef ArduMon<MAX_CMDS,RECV_BUF_SZ,SEND_BUF_SZ, WITH_INT64, WITH_FLOAT, WITH_DOUBLE, WITH_BINARY, WITH_TEXT> AM;

#ifdef ARDUINO
void println(const __FlashStringHelper *str) { Serial.println(str); }
#ifndef BASELINE_MEM
AM am;
#endif
#else
#define F(p) (p)
#include <cstdio>
void println(const char *str) { printf("%s\n", str); }
AM am(&STREAM);
#endif

void show_error(AM* am) {
  AM::Error e = am->get_err();
  if (e != AM::Error::NONE) { println(AM::err_msg(e)); am->clear_err(); }
}

bool noop(AM *am) { return am->end_cmd(); }

bool help(AM *am) { return am->send_cmds() && am->end_cmd(); }

bool argc(AM *am) { return am->send(am->argc()) && am->send_CRLF() && am->end_cmd(); }

class Timer {
public:

  Timer(AM *_am, uint8_t h, uint8_t m, uint8_t s, float _accel) :
    am(_am),
    running(true),
    accel(_accel),
    total_ms((h * 3600 + m * 60 + s) * 1000ul),
    start_ms(millis()) {
    am->send_raw(F("counting down from "));
    am->send_raw(h);
    am->send_raw(':');
    am->send_raw(m, 2|AM::FMT_PAD_ZERO);
    am->send_raw(':');
    am->send_raw(s, 2|AM::FMT_PAD_ZERO);
    am->send_raw(F(", accel="));
    am->send_raw(accel);
    am->send_raw(F(", hit any key to cancel..."));
    am->send_CRLF();
    am->send_raw(AM::VT100_CURSOR_HIDDEN);
    //am->vt100_set_attr(AM::VT100_ATTR_REVERSE);
    //am->vt100_set_attr(AM::VT100_ATTR_BLINK);
    //am->vt100_set_attr(AM::VT100_ATTR_BRIGHT);
    //am->vt100_set_attr(AM::VT100_ATTR_UNDERSCORE);
    am->vt100_set_color(AM::VT100_FOREGROUND, AM::VT100_CYAN);
  }

  bool tick() {
    if (!running) return false;
    uint8_t h, m, s; uint16_t ms = 0;
    const unsigned long elapsed_ms = (millis() - start_ms) * accel;
    if (elapsed_ms >= total_ms || am->get_key() != 0) {
      h = m = s = 0; ms = 0;
      running = false;
    } else {
      const unsigned long remaining_ms = total_ms - elapsed_ms;
      h = remaining_ms / (3600 * 1000ul);
      m = (remaining_ms - h * 3600 * 1000ul) / (60 * 1000ul);
      s = (remaining_ms - (h * 3600 + m * 60) * 1000ul) / 1000ul;
      ms = remaining_ms - (h * 3600 + m * 60 + s) * 1000ul;
    }
    am->send_raw(AM::VT100_CLEAR_LINE);
    am->send_raw(h, 3|AM::FMT_PAD_ZERO);
    am->send_raw(':');
    am->send_raw(m, 2|AM::FMT_PAD_ZERO);
    am->send_raw(':');
    am->send_raw(s, 2|AM::FMT_PAD_ZERO);
    am->send_raw('.');
    am->send_raw(ms, 3|AM::FMT_PAD_ZERO);
    if (!running) {
      am->send_CRLF();
      am->send_raw(AM::VT100_CURSOR_VISIBLE);
      am->vt100_set_attr(AM::VT100_ATTR_RESET);
      am->end_cmd();
    }
    return running;
  }

private:

  AM * const am;
  bool running;
  const float accel;
  const unsigned long total_ms, start_ms;

#ifndef ARDUINO
  unsigned long millis() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint64_t now_ms = ts.tv_sec * 1000ul + ts.tv_nsec / 1000000ul;
    static uint64_t start_ms = now_ms;
    return now_ms - start_ms;
  }
#endif
};

Timer *timer = nullptr;

bool start_timer(AM *am) {
  if (!am->recv()) return false; //skip over command token
  uint8_t h, m, s;
  if (!am->recv(&h) || !am->recv(&m) || !am->recv(&s)) return false;
  float accel = 1;
  if (am->argc() > 4 && !am->recv(&accel)) return false;
  timer = new Timer(am, h, m, s, accel);
  return true;
}

template <typename T> bool echo(AM *am) {
  //the first recv() skips over the command token itself
  T v; return am->recv() && am->recv(&v) && am->send(v) && am->send_CRLF() && am->end_cmd();
}

template <typename T> bool echo_int(AM *am) {
  T v; if (!am->recv() || !am->recv(&v)) return false;
  const uint8_t argc = am->argc();
  bool hex = false, pad_zero = false, pad_right = false; uint8_t width = 0;
  if (argc > 2 && !am->recv(&hex)) return false;
  if (argc > 3 && !am->recv(&width)) return false;
  if (argc > 4 && !am->recv(&pad_zero)) return false;
  if (argc > 5 && !am->recv(&pad_right)) return false;
  if (width > 31) width = 31;
  const uint8_t fmt =
    width | (hex ? AM::FMT_HEX : 0) | (pad_zero ? AM::FMT_PAD_ZERO : 0) | (pad_right ? AM::FMT_PAD_RIGHT : 0);
  return am->send_raw(v, fmt) && (!pad_right || pad_zero || am->send_raw('|')) && am->send_CRLF() && am->end_cmd();
}

template <typename T> bool echo_flt(AM *am) {
  T v; if (!am->recv() || !am->recv(&v)) return false;
  const uint8_t argc = am->argc();
  bool scientific = false; int8_t precision = -1, width = -1;
  if (argc > 2 && !am->recv(&scientific)) return false;
  if (argc > 3 && !am->recv(&precision)) return false;
  if (argc > 4 && !am->recv(&width)) return false;
  return am->send(v, scientific, precision, width) && am->send_CRLF() && am->end_cmd();
}

bool echo_char(AM *am) { return echo<char>(am); }
bool echo_str(AM *am) { return echo<const char*>(am); }
bool echo_bool(AM *am) { return echo<bool>(am); }
bool echo_u8(AM *am) { return echo_int<uint8_t>(am); }
bool echo_s8(AM *am) { return echo_int<int8_t>(am); }
bool echo_u16(AM *am) { return echo_int<uint16_t>(am); }
bool echo_s16(AM *am) { return echo_int<int16_t>(am); }
bool echo_u32(AM *am) { return echo_int<uint32_t>(am); }
bool echo_s32(AM *am) { return echo_int<int32_t>(am); }
#ifdef WITH_INT64
bool echo_u64(AM *am) { return echo_int<uint64_t>(am); }
bool echo_s64(AM *am) { return echo_int<int64_t>(am); }
#endif
#ifdef WITH_FLOAT
bool echo_float(AM *am) { return echo_flt<float>(am); }
#ifdef WITH_DOUBLE
bool echo_double(AM *am) { return echo_flt<double>(am); }
#endif
#endif

bool echo_multiple(AM *am) {
  if (!am->recv()) return false; // skip command token
  
  const char* format;
  if (!am->recv(&format)) return false;
  
  const size_t len = strlen(format);
  for (size_t i = 0; i + 2 < len; i += 3) {
    const char type[4] = {format[i], format[i+1], format[i+2], '\0'};
    
    if (strcmp(type, "chr") == 0) {
      char v;
      if (!am->recv(&v) || !am->send(v)) return false;
    } else if (strcmp(type, "str") == 0) {
      const char* v;
      if (!am->recv(&v) || !am->send(v)) return false;
    } else if (strcmp(type, "bll") == 0) {
      bool v;
      if (!am->recv(&v) || !am->send(v)) return false;
    } else if (strcmp(type, "u08") == 0) {
      uint8_t v;
      if (!am->recv(&v) || !am->send(v)) return false;
    } else if (strcmp(type, "s08") == 0) {
      int8_t v;
      if (!am->recv(&v) || !am->send(v)) return false;
    } else if (strcmp(type, "u16") == 0) {
      uint16_t v;
      if (!am->recv(&v) || !am->send(v)) return false;
    } else if (strcmp(type, "s16") == 0) {
      int16_t v;
      if (!am->recv(&v) || !am->send(v)) return false;
    } else if (strcmp(type, "u32") == 0) {
      uint32_t v;
      if (!am->recv(&v) || !am->send(v)) return false;
    } else if (strcmp(type, "s32") == 0) {
      int32_t v;
      if (!am->recv(&v) || !am->send(v)) return false;
#ifdef WITH_INT64
    } else if (strcmp(type, "u64") == 0) {
      uint64_t v;
      if (!am->recv(&v) || !am->send(v)) return false;
    } else if (strcmp(type, "s64") == 0) {
      int64_t v;
      if (!am->recv(&v) || !am->send(v)) return false;
#endif
#ifdef WITH_FLOAT
    } else if (strcmp(type, "f32") == 0) {
      float v;
      if (!am->recv(&v) || !am->send(v)) return false;
#ifdef WITH_DOUBLE
    } else if (strcmp(type, "f64") == 0) {
      double v;
      if (!am->recv(&v) || !am->send(v)) return false;
#endif
#endif
    }
  }
  
  return am->send_CRLF() && am->end_cmd();
}

#ifndef BASELINE_MEM
void add_cmds() {

  am.set_txt_prompt(F("demo>"));
  am.set_txt_echo(true);

#define ADD_CMD(func, name, desc) if (!am.add_cmd(func, F(name), F(desc))) show_error(&am);

  ADD_CMD(noop, "noop", "no operation");
  ADD_CMD(help, "help", "show commands");
  ADD_CMD(help, "?", "show commands");
  ADD_CMD(argc, "argc", "show arg count");
  ADD_CMD(start_timer, "timer", "hours minutes seconds [accel] | countdown timer");
  ADD_CMD(echo_char, "ec", "echo char");
  ADD_CMD(echo_str, "es", "echo str");
  ADD_CMD(echo_bool, "eb", "echo bool");
  ADD_CMD(echo_u8, "eu8", "[hex [width [pad_zero [pad_right]]]] | echo uint8");
  ADD_CMD(echo_s8, "es8", "[hex [width [pad_zero [pad_right]]]] | echo int8");
  ADD_CMD(echo_u16, "eu16", "[hex [width [pad_zero [pad_right]]]] | echo uint16");
  ADD_CMD(echo_s16, "es16", "[hex [width [pad_zero [pad_right]]]] | echo int16");
  ADD_CMD(echo_u32, "eu32", "[hex [width [pad_zero [pad_right]]]] | echo uint32");
  ADD_CMD(echo_s32, "es32", "[hex [width [pad_zero [pad_right]]]] | echo int32");
#ifdef WITH_INT64
  ADD_CMD(echo_u64, "eu64", "[hex [width [pad_zero [pad_right]]]] | echo uint64");
  ADD_CMD(echo_s64, "es64", "[hex [width [pad_zero [pad_right]]]] | echo int64");
#endif
#ifdef WITH_FLOAT
  ADD_CMD(echo_float, "ef", "[scientific [precision [width]]] | echo float");
#ifdef WITH_DOUBLE
  ADD_CMD(echo_float, "ed", "[scientific [precision [width]]] | echo double");
#endif
#endif
  ADD_CMD(echo_multiple, "em", "format_string args... | echo multiple args based on format");

#undef ADD_CMD
}
#endif

void setup() {
#ifdef ARDUINO
  Serial.begin(BAUD);
#endif
#ifndef BASELINE_MEM
  add_cmds();
#endif
}

void loop() {
#ifndef BASELINE_MEM
  am.update();
#endif
  if (timer && !timer->tick()) { delete timer; timer = nullptr; }
}

