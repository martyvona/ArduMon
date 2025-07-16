#include <ArduMonSlave.h>

#define WITH_INT64
#define WITH_DOUBLE

#define BAUD 115200

#ifndef MAX_CMDS
#define MAX_CMDS 64
#endif

#ifndef RECV_BUF_SZ
#define RECV_BUF_SZ 256
#endif

#ifndef SEND_BUF_SZ
#define SEND_BUF_SZ 256
#endif

typedef ArduMonSlave<MAX_CMDS,RECV_BUF_SZ,SEND_BUF_SZ> AMS;

#ifdef ARDUINO
void println(const char *str) { Serial.println(str); }
AMS ams;
#else
#define PSTR(s) (s)
#include <cstdio>
void println(const char *str) { printf("%s\n", str); }
AMS ams(&STREAM);
#endif

bool noop(AMS *ams) { return ams->end_cmd(); }

bool help(AMS *ams) { return ams->send_cmds() && ams->end_cmd(); }

template <typename T> bool echo(AMS *ams) {
  //the first recv() skips over the command token itself
  T v; return ams->recv() && ams->recv(&v) && ams->send(v) && ams->send_CRLF() && ams->end_cmd();
}

template <typename T> bool echo_int(AMS *ams) {
  const char *cmd;
  if (!ams->recv(&cmd)) return false;
  const char *end= cmd; while (*end) ++end;
  const bool hex = *(end-1) == 'h';
  T v; return ams->recv(&v, hex) && ams->send(v, hex) && ams->send_CRLF() && ams->end_cmd();
}

bool echo_char(AMS *ams) { return echo<char>(ams); }
bool echo_str(AMS *ams) { return echo<const char*>(ams); }
bool echo_bool(AMS *ams) { return echo<bool>(ams); }
bool echo_u8(AMS *ams) { return echo_int<uint8_t>(ams); }
bool echo_s8(AMS *ams) { return echo_int<int8_t>(ams); }
bool echo_u16(AMS *ams) { return echo_int<uint16_t>(ams); }
bool echo_s16(AMS *ams) { return echo_int<int16_t>(ams); }
bool echo_u32(AMS *ams) { return echo_int<uint32_t>(ams); }
bool echo_s32(AMS *ams) { return echo_int<int32_t>(ams); }
#ifdef WITH_INT64
bool echo_u64(AMS *ams) { return echo_int<uint64_t>(ams); }
bool echo_s64(AMS *ams) { return echo_int<int64_t>(ams); }
#endif
bool echo_float(AMS *ams) { return echo<float>(ams); }
#ifdef WITH_DOUBLE
bool echo_double(AMS *ams) { return echo<double>(ams); }
#endif

void show_error() {
  AMS::Error e = ams.get_err();
  if (e != AMS::Error::NONE) {
    println(AMS::err_msg_P(e));
    ams.clear_err();
  }
}

void add_cmds() {

  ams.set_txt_prompt("demo>");
  ams.set_txt_echo(true);

  if (!ams.add_cmd_P(noop, PSTR("noop"), "no operation")) show_error();
  if (!ams.add_cmd_P(help, PSTR("help"), "show commands")) show_error();
  if (!ams.add_cmd_P(help, PSTR("?"), "show commands")) show_error();
  if (!ams.add_cmd_P(echo_char, PSTR("ec"), "echo char")) show_error();
  if (!ams.add_cmd_P(echo_str, PSTR("es"), "echo str")) show_error();
  if (!ams.add_cmd_P(echo_bool, PSTR("eb"), "echo bool")) show_error();
  if (!ams.add_cmd_P(echo_u8, PSTR("eu8"), "echo uint8")) show_error();
  if (!ams.add_cmd_P(echo_u8, PSTR("eu8h"), "echo uint8 hex")) show_error();
  if (!ams.add_cmd_P(echo_s8, PSTR("e8"), "echo int8")) show_error();
  if (!ams.add_cmd_P(echo_s8, PSTR("e8h"), "echo int8 hex")) show_error();
  if (!ams.add_cmd_P(echo_u16, PSTR("eu16"), "echo uint16")) show_error();
  if (!ams.add_cmd_P(echo_u16, PSTR("eu16h"), "echo uint16 hex")) show_error();
  if (!ams.add_cmd_P(echo_s16, PSTR("e16"), "echo int16")) show_error();
  if (!ams.add_cmd_P(echo_s16, PSTR("e16h"), "echo int16 hex")) show_error();
  if (!ams.add_cmd_P(echo_u32, PSTR("eu32"), "echo uint32")) show_error();
  if (!ams.add_cmd_P(echo_u32, PSTR("eu32h"), "echo uint32 hex")) show_error();
  if (!ams.add_cmd_P(echo_s32, PSTR("e32"), "echo int32")) show_error();
  if (!ams.add_cmd_P(echo_s32, PSTR("e32h"), "echo int32hex")) show_error();
#ifdef WITH_INT64
  if (!ams.add_cmd_P(echo_u64, PSTR("eu64"), "echo uint64")) show_error();
  if (!ams.add_cmd_P(echo_u64, PSTR("eu64h"), "echo uint64 hex")) show_error();
  if (!ams.add_cmd_P(echo_s64, PSTR("e64"), "echo int64")) show_error();
  if (!ams.add_cmd_P(echo_s64, PSTR("e64h"), "echo int64 hex")) show_error();
#endif
  if (!ams.add_cmd_P(echo_float, PSTR("ef"), "echo float")) show_error();
#ifdef WITH_DOUBLE
  if (!ams.add_cmd_P(echo_double, PSTR("ed"), "echo double")) show_error();
#endif
}

void setup() {
#ifdef ARDUINO
  Serial.begin(BAUD);
#endif
  add_cmds();
}

void loop() { ams.update(); }

