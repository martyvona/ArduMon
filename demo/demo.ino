#include <ArduMon.h>

//#define BASELINE_MEM

#ifndef WITH_INT64
#define WITH_INT64 true
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

typedef ArduMon<MAX_CMDS,RECV_BUF_SZ,SEND_BUF_SZ, WITH_INT64, WITH_DOUBLE, WITH_BINARY, WITH_TEXT> AM;

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

bool noop(AM *am) { return am->end_cmd(); }

bool help(AM *am) { return am->send_cmds() && am->end_cmd(); }

template <typename T> bool echo(AM *am) {
  //the first recv() skips over the command token itself
  T v; return am->recv() && am->recv(&v) && am->send(v) && am->send_CRLF() && am->end_cmd();
}

template <typename T> bool echo_int(AM *am) {
  const char *cmd;
  if (!am->recv(&cmd)) return false;
  const char *end= cmd; while (*end) ++end;
  const bool hex = *(end-1) == 'h';
  T v; return am->recv(&v, hex) && am->send(v, hex) && am->send_CRLF() && am->end_cmd();
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
bool echo_float(AM *am) { return echo<float>(am); }
#ifdef WITH_DOUBLE
bool echo_double(AM *am) { return echo<double>(am); }
#endif

void show_error(AM* am) {
  AM::Error e = am->get_err();
  if (e != AM::Error::NONE) { println(AM::err_msg(e)); am->clear_err(); }
}

#ifndef BASELINE_MEM
void add_cmds() {

  am.set_txt_prompt(F("demo>"));
  am.set_txt_echo(true);

#define ADD_CMD(func, name, desc) if (!am.add_cmd(func, F(name), F(desc))) show_error(&am);

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
}

