#include <ArduMon.h>

#include "dbg_print.h"
#include "am_timer.h"

//builds text server demo by default
//#define BASELINE_MEM to check memory usage of boilerplate
//#define BINARY true to build binary server
//#define BINARY_CLIENT to build binary client

/* configure ArduMon **************************************************************************************************/

#ifdef BINARY_CLIENT
#define BINARY true
#endif

#ifndef BINARY
#define BINARY false
#endif

#ifndef BIN_RX_PIN
#define BIN_RX_PIN 10
#endif

#ifndef BIN_TX_PIN
#define BIN_TX_PIN 11
#endif

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

//specialize the ArduMon class template and call that AM
typedef ArduMon<MAX_CMDS, RECV_BUF_SZ, SEND_BUF_SZ, WITH_INT64, WITH_FLOAT, WITH_DOUBLE, WITH_BINARY, WITH_TEXT> AM;

/* set up the ArduMon input stream AM_STREAM **************************************************************************/

#ifdef ARDUINO
#if BINARY //for binary demo connect BIN_TX_PIN of client Arduino to BIN_RX_PIN of server Arduino and vice-versa
#ifdef ESP32
#define AM_STREAM Serial1
#else
#include <SoftwareSerial.h>
SoftwareSerial AM_STREAM(BIN_RX_PIN, BIN_TX_PIN);
#endif //ESP32
#else //for text demo connect an Arduino by USB and run minicom or screen on USB serial port as described in README.md
#define AM_STREAM Serial
#endif //BINARY
#endif //ARDUINO

//AM_STREAM is defined externally for native build

/* create the ArduMon instance am *************************************************************************************/

#ifndef BASELINE_MEM
AM am(&AM_STREAM, BINARY);
#ifndef BINARY_CLIENT
Timer<AM> timer;
#endif //BINARY_CLIENT
#endif //BASELINE_MEM

/* server command handlers ********************************************************************************************/

void show_error(AM* am) {
  AM::Error e = am->get_err();
  if (e != AM::Error::NONE) { println(AM::err_msg(e)); am->clear_err(); }
}

bool noop(AM *am) { return am->end_cmd(); }

bool help(AM *am) { return am->send_cmds() && am->end_cmd(); }

bool argc(AM *am) { return am->send(am->argc()) && am->end_cmd(); }

#ifndef BINARY_CLIENT
bool start_timer(AM *am) { return timer.start(am); }
bool stop_timer(AM *am) { return timer.stop(am); }
bool get_timer(AM *am) { return timer.get_time(am); }
#else
bool start_timer(AM *am) { return true; }
bool stop_timer(AM *am) { return true; }
bool get_timer(AM *am) { return true; }
#endif

template <typename T> bool echo(AM *am) {
  //the first recv() skips over the command token itself
  T v; return am->recv() && am->recv(&v) && am->send(v) && am->end_cmd();
}

template <typename T> bool echo_int(AM *am) {
  T v; if (!am->recv() || !am->recv(&v)) return false;
  bool hex = false, pad_zero = false, pad_right = false; uint8_t width = 0;
  if (am->is_txt_mode()) {
    const uint8_t argc = am->argc();
    if (argc > 2 && !am->recv(&hex)) return false;
    if (argc > 3 && !am->recv(&width)) return false;
    if (argc > 4 && !am->recv(&pad_zero)) return false;
    if (argc > 5 && !am->recv(&pad_right)) return false;
    if (width > 31) width = 31;
  }
  const uint8_t fmt =
    width | (hex ? AM::FMT_HEX : 0) | (pad_zero ? AM::FMT_PAD_ZERO : 0) | (pad_right ? AM::FMT_PAD_RIGHT : 0);
  return am->send_raw(v, fmt) && (am->is_binary_mode() || !pad_right || pad_zero || am->send_raw('|')) && am->end_cmd();
}

template <typename T> bool echo_flt(AM *am) {
  T v; if (!am->recv() || !am->recv(&v)) return false;
  bool scientific = false; int8_t precision = -1, width = -1;
  if (am->is_txt_mode()) {
    const uint8_t argc = am->argc();
    if (argc > 2 && !am->recv(&scientific)) return false;
    if (argc > 3 && !am->recv(&precision)) return false;
    if (argc > 4 && !am->recv(&width)) return false;
  }
  return am->send(v, scientific, precision, width) && am->end_cmd();
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
  if (!am->recv()) return false; // skip command token/code
  
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
  return am->end_cmd();
}

float float_param = 0;
bool set_float_param(AM *am) { return am->recv() && am->recv(&float_param) && am->end_cmd(); }
bool get_float_param(AM *am) { return am->recv() && am->send(&float_param) && am->end_cmd(); }

/* binary client state machine ****************************************************************************************/

//in the text demo the user is the client, interacting as desired with the ArduMon server through a serial terminal
//in the binary demo the client is its own separate program, and the interaction is a fixed state machine

#ifdef BINARY_CLIENT

struct CmdRec {
  const uint8_t code; const __FlashStringHelper * const name; CmdRec *prev;
  CmdRec(uint8_t c, const __FlashStringHelper *n) : code(c), name(n) { }
};
CmdRec *last_server_cmd;
uint8_t server_cmd_code(const __FlashStringHelper *name, const CmdRec *rec = last_server_cmd) {
  if (!rec) { print(F("ERROR, unknown command: ")); println(name); return 0; }
  if (AM::strcmp_PP(rec->name, name) == 0) return rec->code;
  return server_cmd_code(name, rec->prev);
}

struct BCStage;
BCStage *first_bc_stage, *last_bc_stage, *bc_stage;

struct BCStage {

  typedef void (*init_t)(void);
  init_t init;

  AM::handler_t send, recv;

  BCStage *next;

  bool started = false;

  BCStage(AM::handler_t s, AM::handler_t r, init_t i = 0) : send(s), recv(r), init(i) {
    if (!first_bc_stage) first_bc_stage = bc_stage = this;
    if (last_bc_stage) last_bc_stage->next = this;
    last_bc_stage = this;
  }

  void run() {
    show_error(&am); //show and clear any error from previous stage
    if (init) init();
    started = true;
    am.set_universal_handler(recv);
    if (send && !send(&am)) show_error(&am);
  }

  bool is_running() { return started && am.get_universal_handler() == recv; }
};

bool send_argc(AM *am) {
  const uint8_t code = server_cmd_code(F("argc"));
  print(F("sending argc (code=")); print(static_cast<int>(code)); println(F(") with 3 args"));
  return am->send(code) && am->send(static_cast<uint8_t>(42)) && am->send(3.14f) && am->send_packet();
}

bool recv_argc(AM *am) {
  am->set_universal_handler(0);
  uint8_t argc;
  if (!am->recv(&argc)) return false;
  if (argc != 3) print(F("ERROR: "));
  print(F("argc received ")); print(static_cast<int>(argc)); println(F(", expected 3"));
}

BCStage *bc_argc = new BCStage(send_argc, recv_argc);

float bc_param = 0.0;

bool send_param(AM *am) {
  uint8_t code = server_cmd_code(F("set_param"));
  print(F("sending set_param (code=")); print(static_cast<int>(code)); print(F(") value=")); println(bc_param);
  if (!am->send(code) || !am->send(bc_param) || !am->send_packet()) return false; //1 + 1 + 4 + 1 = 7 bytes
  code = server_cmd_code(F("get_param"));
  print(F("sending get_param (code=")); print(static_cast<int>(code)); println(F(")"));
  return am->send(code) && am->send_packet(); //1 + 1 + 1 = 3 bytes
  //in setup() we called am.set_send_wait_ms(AM::ALWAYS_WAIT)
  //so the above sends all block until the data can be put into the client's serial send buffer
  //thus, we just rapidly sent two packets totalling 7 + 3 = 10 bytes, which should be OK because the
  //server's serial receive buffer should be at least 64 bytes; now we wait for a response to establish flow control
}

bool recv_param(AM *am) {
  am->set_universal_handler(0);
  float param;
  if (!am->recv(&param)) return false;
  if (param != bc_param) print(F("ERROR: "));
  print(F("get_param received ")); print(param); print(F(", expected ")); println(bc_param);
}

BCStage *bc_param_A = new BCStage(send_param, recv_param, [](){ bc_param = 0.0; });
BCStage *bc_param_B = new BCStage(send_param, recv_param, [](){ bc_param = 1.0; });
//TODO moar

bool send_done(AM *am) { println(F("binary client done")); }
BCStage *bc_last = new BCStage(send_done, 0); //also shows any error from the previous stage

#endif //BINARY_CLIENT

#ifndef BASELINE_MEM
void add_cmds() {

  am.set_txt_prompt(F("demo>"));
  am.set_txt_echo(true);

#ifndef BINARY_CLIENT
#define ADD_CMD(func, name, desc) if (!am.add_cmd(func, F(name), F(desc))) show_error(&am);
#else
#define ADD_CMD(func, name, desc) {                         \
  CmdRec *new_cmd = new CmdRec(am.get_num_cmds(), F(name)); \
  if (last_server_cmd) new_cmd->prev = last_server_cmd;     \
  last_server_cmd = new_cmd;                                \
}
#endif

  ADD_CMD(noop, "noop", "no operation");
  ADD_CMD(help, "help", "show commands");
  ADD_CMD(argc, "argc", "show arg count");
  ADD_CMD(start_timer, "start_timer",
          "hours minutes seconds [accel] [async] [async_cmd_code|sync_throttle_ms] | start timer");
  ADD_CMD(stop_timer, "stop_timer", "stop timer");
  ADD_CMD(get_timer, "get_timer", "get timer");
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
  ADD_CMD(set_float_param, "sfp", "float | set float param");
  ADD_CMD(get_float_param, "gfp", "get float param");

#undef ADD_CMD
}
#endif //BASELINE_MEM

void setup() {
#ifdef ARDUINO
  Serial.begin(BAUD);
#if BINARY
#ifdef ESP32
  AM_STREAM.begin(BAUD, SERIAL_8N1, BIN_RX_PIN, BIN_TX_PIN);
#else
  AM_STREAM.begin(BAUD);
#endif //ESP32
#endif //BINARY
#endif //ARDUINO
#ifndef BASELINE_MEM
  add_cmds();
#endif
#ifdef BINARY_CLIENT
  am.set_send_wait_ms(AM::ALWAYS_WAIT);
#endif
}

void loop() {
#ifndef BASELINE_MEM
  am.update();
#ifndef BINARY_CLIENT
  timer.tick(&am); //text demo or binary server
#else //binary client
  if (bc_stage) {
    if (!bc_stage->started) bc_stage->run();
    else if (!bc_stage->is_running()) bc_stage = bc_stage->next;
  }
#endif //BINARY_CLIENT
#endif //BASELINE_MEM
}

