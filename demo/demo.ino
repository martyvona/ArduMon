#include <ArduMonSlave.h>

#define WITH_INT64
#define WITH_DOUBLE

#define BAUD 115200

#ifndef MAX_CMDS
#define MAX_CMDS 32
#endif

#ifndef RECV_BUF_SZ
#define RECV_BUF_SZ 128
#endif

#ifndef SEND_BUF_SZ
#define SEND_BUF_SZ 128
#endif

typedef ArduMonSlave<MAX_CMDS,RECV_BUF_SZ,SEND_BUF_SZ> AMS;

#ifdef ARDUINO
void println(const __FlashStringHelper *str) { Serial.println(str); }
AMS ams;
#else
#define F(p) (p)
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
  if (e != AMS::Error::NONE) { println(AMS::err_msg(e)); ams.clear_err(); }
}

void add_cmds() {

  ams.set_txt_prompt(F("demo>"));
  ams.set_txt_echo(true);

#define ADD_CMD(func, name, desc) if (!ams.add_cmd(func, F(name), F(desc))) show_error();

  ADD_CMD(noop, "noop", "no operation");
  ADD_CMD(help, "help", "show commands");
  ADD_CMD(help, "?", "show commands");
  ADD_CMD(echo_char, "ec", "echo char");
  ADD_CMD(echo_str, "es", "echo str");
  ADD_CMD(echo_bool, "eb", "echo bool");
  ADD_CMD(echo_s8, "es8", "echo int8");
  ADD_CMD(echo_s8, "es8h", "echo int8 hex");
  ADD_CMD(echo_u8, "eu8", "echo uint8");
  ADD_CMD(echo_u8, "eu8h", "echo uint8 hex");
  ADD_CMD(echo_s16, "es16", "echo int16");
  ADD_CMD(echo_s16, "es16h", "echo int16 hex");
  ADD_CMD(echo_u16, "eu16", "echo uint16");
  ADD_CMD(echo_u16, "eu16h", "echo uint16 hex");
  ADD_CMD(echo_s32, "es32", "echo int32");
  ADD_CMD(echo_s32, "es32h", "echo int32 hex");
  ADD_CMD(echo_u32, "eu32", "echo uint32");
  ADD_CMD(echo_u32, "eu32h", "echo uint32 hex");
#ifdef WITH_INT64
  ADD_CMD(echo_s64, "es64", "echo int64");
  ADD_CMD(echo_s64, "es64h", "echo int64 hex");
  ADD_CMD(echo_u64, "eu64", "echo uint64");
  ADD_CMD(echo_u64, "eu64h", "echo uint64 hex");
#endif
  ADD_CMD(echo_float, "ef", "echo float");
#ifdef WITH_DOUBLE
  ADD_CMD(echo_float, "ed", "echo double");
#endif

#undef ADD_CMD
}

void setup() {
#ifdef ARDUINO
  Serial.begin(BAUD);
#endif
  add_cmds();
}

void loop() {
  ams.update();
}

