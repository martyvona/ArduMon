#define WITH_INT64
#define WITH_DOUBLE

#define BAUD 115200

#define MAX_CMDS 64
#define RECV_BUF_SZ 256
#define SEND_BUF_SZ 256

#include <ArduMonSlave.h>

typedef ArduMonSlave<MAX_CMDS,RECV_BUF_SZ,SEND_BUF_SZ> AMS;

#ifdef ARDUINO
void println(const char *str) { Serial.println(str); }
AMS ams;
#else
#define PSTR(s) (s)
#include <cstdio>
void println(const char *str) { printf("%s\n", str); }
AMS ams(&demo_stream);
#endif

void show_error() {
  AMS::Error e = ams.get_err();
  if (e != AMS::Error::NONE) {
    println(AMS::err_msg_P(e));
    ams.clear_err();
  }
}

bool noop(AMS *ams) {
  if (!ams->recv()) show_error();
  return true;
}

bool help(AMS *ams) {
  if (!ams->send_cmds()) show_error();
  return true;
}

bool echo_char(AMS *ams) {
  char v;
  if (!ams->recv(&v)) show_error();
  if (!ams->send(v)) show_error();
  if (!ams->end_cmd()) show_error();
  return true;
}

bool echo_str(AMS *ams) {
  const char *v;
  if (!ams->recv(&v)) show_error();
  if (!ams->send(v)) show_error();
  if (!ams->end_cmd()) show_error();
  return true;
}

bool echo_bool(AMS *ams) {
  bool v;
  if (!ams->recv(&v)) show_error();
  if (!ams->send(v)) show_error();
  if (!ams->end_cmd()) show_error();
  return true;
}

bool echo_u8(AMS *ams) {
  uint8_t v;
  if (!ams->recv(&v)) show_error();
  if (!ams->send(v)) show_error();
  if (!ams->end_cmd()) show_error();
  return true;
}

bool echo_s8(AMS *ams) {
  int8_t v;
  if (!ams->recv(&v)) show_error();
  if (!ams->send(v)) show_error();
  if (!ams->end_cmd()) show_error();
  return true;
}

bool echo_u16(AMS *ams) {
  uint16_t v;
  if (!ams->recv(&v)) show_error();
  if (!ams->send(v)) show_error();
  if (!ams->end_cmd()) show_error();
  return true;
}

bool echo_s16(AMS *ams) {
  int16_t v;
  if (!ams->recv(&v)) show_error();
  if (!ams->send(v)) show_error();
  if (!ams->end_cmd()) show_error();
  return true;
}

bool echo_u32(AMS *ams) {
  uint32_t v;
  if (!ams->recv(&v)) show_error();
  if (!ams->send(v)) show_error();
  if (!ams->end_cmd()) show_error();
  return true;
}

bool echo_s32(AMS *ams) {
  int32_t v;
  if (!ams->recv(&v)) show_error();
  if (!ams->send(v)) show_error();
  if (!ams->end_cmd()) show_error();
  return true;
}

#ifdef WITH_INT64
bool echo_u64(AMS *ams) {
  uint64_t v;
  if (!ams->recv(&v)) show_error();
  if (!ams->send(v)) show_error();
  if (!ams->end_cmd()) show_error();
  return true;
}

bool echo_s64(AMS *ams) {
  int64_t v;
  if (!ams->recv(&v)) show_error();
  if (!ams->send(v)) show_error();
  if (!ams->end_cmd()) show_error();
  return true;
}
#endif

bool echo_float(AMS *ams) {
  float f;
  if (!ams->recv(&f)) show_error();
  if (!ams->send(f)) show_error();
  if (!ams->end_cmd()) show_error();
  return true;
}

#ifdef WITH_DOUBLE
bool echo_double(AMS *ams) {
  double d;
  if (!ams->recv(&d)) show_error();
  if (!ams->send(d)) show_error();
  if (!ams->end_cmd()) show_error();
  return true;
}
#endif

void add_cmds() {
  uint8_t code = 0;
  if (!ams.add_cmd_P(noop, PSTR("noop"), code++, "no operation")) show_error();
  if (!ams.add_cmd_P(help, PSTR("help"), code++, "show commands")) show_error();
  if (!ams.add_cmd_P(echo_char, PSTR("ec"), code++, "echo char")) show_error();
  if (!ams.add_cmd_P(echo_str, PSTR("es"), code++, "echo str")) show_error();
  if (!ams.add_cmd_P(echo_bool, PSTR("eb"), code++, "echo bool")) show_error();
  if (!ams.add_cmd_P(echo_u8, PSTR("eu8"), code++, "echo uint8")) show_error();
  if (!ams.add_cmd_P(echo_s8, PSTR("e8"), code++, "echo int8")) show_error();
  if (!ams.add_cmd_P(echo_u16, PSTR("eu16"), code++, "echo uint16")) show_error();
  if (!ams.add_cmd_P(echo_s16, PSTR("e16"), code++, "echo int16")) show_error();
  if (!ams.add_cmd_P(echo_s32, PSTR("eu32"), code++, "echo uint32")) show_error();
  if (!ams.add_cmd_P(echo_s32, PSTR("e32"), code++, "echo int32")) show_error();
#ifdef WITH_INT64
  if (!ams.add_cmd_P(echo_u64, PSTR("eu64"), code++, "echo uint64")) show_error();
  if (!ams.add_cmd_P(echo_s64, PSTR("e64"), code++, "echo int64")) show_error();
#endif
  if (!ams.add_cmd_P(echo_float, PSTR("ef"), code++, "echo float")) show_error();
#ifdef WITH_DOUBLE
  if (!ams.add_cmd_P(echo_double, PSTR("ed"), code++, "echo double")) show_error();
#endif
}

#ifdef ARDUINO
void setup() {
  Serial.begin(BAUD);
  ams.set_txt_prompt("demo>");
  ams.set_txt_echo(true);
  add_cmds();
}

void loop() { ams.update(); }
#endif

