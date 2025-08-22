/**
 * ArduMon: Yet another Arduino serial command library.
 *
 * See https://github.com/martyvona/ArduMon/blob/main/README.md
 *
 * This is the main code of the demo.  It can compile for Arduino when included in demo.ino, and it can also compile for
 * the native host when included in native/demo.cpp.  By default it implements a text mode ArduMon server supporting a
 * small catalog of demonstration commands, including commands to echo values of various types, as well as a countdown
 * timer (see am_timer.h).  It can also compile as a binary (not text) server with BINARY=true (see
 * binary_server/binary_server.ino), and as a binary client with BINARY_CLIENT defined (see
 * binary_client/binary_client.ino).  The native server build can run in binary or text mode depending on a runtime
 * command line option, and it can run in binary client mode with the compile time flag -DBINARY_CLIENT.
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

#include <ArduMon.h>

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

//comment these out to compile the binary demo (client or server) to use default serial port for binary communication
//this will disable debug prints, but can be useful in particular to run the binary server on an Arduino
//connected by USB to the binary client running natively on the host
#define BIN_RX_PIN 10
#define BIN_TX_PIN 11

#define WITH_INT64 true
#define WITH_FLOAT true
#define WITH_DOUBLE true
#define WITH_BINARY true
#define WITH_TEXT true

#define BAUD 115200

#define MAX_CMDS 32

#define RECV_BUF_SZ 128
#define SEND_BUF_SZ 128

//it's possible to run the binary client or server with the binary communication on the default serial port
//e.g. run the binary server this way and connect the Arduino by USB to a host
//and then run the binary client natively on the host
//in this situation we need to disable the debug prints as they would also use the default serial port
#if defined(ARDUINO) && BINARY && !defined(BIN_RX_PIN)
#define print(v) {}
#define println(v) {}
#else
#include "dbg_print.h"
#endif

//specialize the ArduMon class template and call that AM
typedef ArduMon<MAX_CMDS, RECV_BUF_SZ, SEND_BUF_SZ, WITH_INT64, WITH_FLOAT, WITH_DOUBLE, WITH_BINARY, WITH_TEXT> AM;

/* set up the ArduMon input stream AM_STREAM **************************************************************************/

#ifdef ARDUINO

#if BINARY && defined(BIN_RX_PIN)

//connect BIN_TX_PIN of client Arduino to BIN_RX_PIN of server Arduino and vice-versa
#ifdef ESP32
//use second hardware serial port on ESP32, will configure it to use BIN_RX_PIN and BIN_TX_PIN in setup() below
#define AM_STREAM Serial1
#else
//use software serial if not on ESP32
#include <SoftwareSerial.h>
SoftwareSerial AM_STREAM(BIN_RX_PIN, BIN_TX_PIN);
#endif //ESP32

#else //BINARY && defined(BIN_RX_PIN)

//connect an Arduino by USB and run minicom or screen on USB serial port as described in README.md
#define AM_STREAM Serial

#endif //BINARY && defined(BIN_RX_PIN)

#endif //ARDUINO

//AM_STREAM is defined externally for native build

/* create the ArduMon instance am *************************************************************************************/

#ifndef BASELINE_MEM
AM am(&AM_STREAM, BINARY);
#ifndef BINARY_CLIENT
Timer<AM> timer;
#endif //BINARY_CLIENT
#endif //BASELINE_MEM

void show_error(AM& am) {
  AM::Error e = am.get_err();
  if (e != AM::Error::NONE) { println(AM::err_msg(e)); am.clear_err(); }
}

/* server command handlers ********************************************************************************************/

bool help(AM &am) { return am.send_cmds() && am.end_cmd(); }

bool argc(AM &am) { return am.send(am.argc()) && am.end_cmd(); }

bool gcc(AM &am) {
  const char *name; return am.recv() && am.recv(&name) && am.send(am.get_cmd_code(name)) && am.end_cmd();
}

#ifndef BINARY_CLIENT
bool start_timer(AM &am) { return timer.start(am); }
bool stop_timer(AM &am) { return timer.stop(am) && am.end_cmd(); }
bool get_timer(AM &am) { return timer.get_time(am) && am.end_cmd(); }
#else
bool start_timer(AM &am) { return true; }
bool stop_timer(AM &am) { return true; }
bool get_timer(AM &am) { return true; }
#endif

template <typename T> bool echo(AM &am) {
  //the first recv() skips over the command token itself
  T v; return am.recv() && am.recv(&v) && am.send(v) && am.end_cmd();
}

template <typename T> bool echo_int(AM &am) {
  T v; if (!am.recv() || !am.recv(&v)) return false;
  bool hex = false, pad_zero = false, pad_right = false; uint8_t width = 0;
  if (am.is_txt_mode()) {
    const uint8_t argc = am.argc();
    if (argc > 2 && !am.recv(&hex)) return false;
    if (argc > 3 && !am.recv(&width)) return false;
    if (argc > 4 && !am.recv(&pad_zero)) return false;
    if (argc > 5 && !am.recv(&pad_right)) return false;
    if (width > 31) width = 31;
  }
  const uint8_t fmt =
    width | (hex ? AM::FMT_HEX : 0) | (pad_zero ? AM::FMT_PAD_ZERO : 0) | (pad_right ? AM::FMT_PAD_RIGHT : 0);
  return am.send_raw(v, fmt) && (am.is_binary_mode() || !pad_right || pad_zero || am.send_raw('|')) && am.end_cmd();
}

template <typename T> bool echo_flt(AM &am) {
  T v; if (!am.recv() || !am.recv(&v)) return false;
  bool scientific = false; int8_t precision = -1, width = -1;
  if (am.is_txt_mode()) {
    const uint8_t argc = am.argc();
    if (argc > 2 && !am.recv(&scientific)) return false;
    if (argc > 3 && !am.recv(&precision)) return false;
    if (argc > 4 && !am.recv(&width)) return false;
  }
  return am.send(v, scientific, precision, width) && am.end_cmd();
}

bool echo_char(AM &am) { return echo<char>(am); }
bool echo_str(AM &am) { return echo<const char*>(am); }
bool echo_bool(AM &am) { return echo<bool>(am); }
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
  if (!am.recv()) return false; // skip command token/code
  
  const char* format;
  if (!am.recv(&format)) return false;
  
  const size_t len = strlen(format);
  for (size_t i = 0; i + 2 < len; i += 3) {
    const char type[4] = {format[i], format[i+1], format[i+2], '\0'};
    
    if (strcmp(type, "chr") == 0) {
      char v;
      if (!am.recv(&v) || !am.send(v)) return false;
    } else if (strcmp(type, "str") == 0) {
      const char* v;
      if (!am.recv(&v) || !am.send(v)) return false;
    } else if (strcmp(type, "bll") == 0) {
      bool v;
      if (!am.recv(&v) || !am.send(v)) return false;
    } else if (strcmp(type, "u08") == 0) {
      uint8_t v;
      if (!am.recv(&v) || !am.send(v)) return false;
    } else if (strcmp(type, "s08") == 0) {
      int8_t v;
      if (!am.recv(&v) || !am.send(v)) return false;
    } else if (strcmp(type, "u16") == 0) {
      uint16_t v;
      if (!am.recv(&v) || !am.send(v)) return false;
    } else if (strcmp(type, "s16") == 0) {
      int16_t v;
      if (!am.recv(&v) || !am.send(v)) return false;
    } else if (strcmp(type, "u32") == 0) {
      uint32_t v;
      if (!am.recv(&v) || !am.send(v)) return false;
    } else if (strcmp(type, "s32") == 0) {
      int32_t v;
      if (!am.recv(&v) || !am.send(v)) return false;
#ifdef WITH_INT64
    } else if (strcmp(type, "u64") == 0) {
      uint64_t v;
      if (!am.recv(&v) || !am.send(v)) return false;
    } else if (strcmp(type, "s64") == 0) {
      int64_t v;
      if (!am.recv(&v) || !am.send(v)) return false;
#endif
#ifdef WITH_FLOAT
    } else if (strcmp(type, "f32") == 0) {
      float v;
      if (!am.recv(&v) || !am.send(v)) return false;
#ifdef WITH_DOUBLE
    } else if (strcmp(type, "f64") == 0) {
      double v;
      if (!am.recv(&v) || !am.send(v)) return false;
#endif
#endif
    }
  }
  return am.end_cmd();
}

float float_param = 0;
bool set_float_param(AM &am) { return am.recv() && am.recv(&float_param) && am.end_cmd(); }
bool get_float_param(AM &am) { return am.recv() && am.send(float_param) && am.end_cmd(); }

#ifndef BASELINE_MEM
void add_cmds() {

  am.set_txt_prompt(F("demo>"));
  am.set_txt_echo(true);

#define ADD_CMD(func, name, desc) if (!am.add_cmd(func, F(name), F(desc))) show_error(am);

  ADD_CMD(gcc, "gcc", "name | get command code");
  ADD_CMD(help, "help", "show commands");
  ADD_CMD(argc, "argc", "show arg count");
  ADD_CMD(start_timer, "st", "hours minutes seconds [accel] [async] [async_cmd_code|sync_throttle_ms] | start timer");
  ADD_CMD(stop_timer, "ot", "stop timer");
  ADD_CMD(get_timer, "gt", "get timer");
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

/* binary client state machine ****************************************************************************************/

//in the text demo the user is the client, interacting as desired with the ArduMon server through a serial terminal
//in the binary demo the client is its own separate program, and the interaction is a fixed state machine

#ifdef BINARY_CLIENT

//each BCStage we instantiate will automatically append itself to a linked list
//then in the Arduino loop() method we will advance current_bc_stage through the list
class BCStage;
BCStage *first_bc_stage = 0, *last_bc_stage = 0, *current_bc_stage = 0;

class BCStage : public AM::Runnable {
public:

  BCStage() {
    if (!first_bc_stage) first_bc_stage = current_bc_stage = this;
    if (last_bc_stage) last_bc_stage->next = this;
    last_bc_stage = this;
  }

  //returns pointer to next stage when done, else null
  BCStage* update() {
    if (!started) { start(); return 0; }
    if (done) return next;
    if (am.get_universal_runnable() == this) return 0; //still running
    done = true;
    return next;
  }

  bool run(AM& am) {
    am.set_universal_runnable(0);
    const bool ret = recv();
    if (!ret) show_error(am);
    return ret;
  }

protected:

  virtual bool send() = 0;
  virtual bool recv() = 0;

private:

  BCStage *next;
  bool started = false, done = false;

  void start() {
    started = true;
    am.set_universal_runnable(this); //install ourself as receive runnable
    if (!send()) show_error(am);
  }
};

//there are several ways for the client and server to know that the code for e.g. the "argc" command is 2
//one approach is just to hardcode that into both the client and server, e.g. in a shared header file
//another approach is for the server to implement a "gcc" command that will return the code for a given command name
//and register the gcc command with a well-known command code, e.g. 0; this latter approach is demonstrated here

class BCStage_gcc : public BCStage {
public:
  const char * const cmd_name;
  BCStage_gcc(const char *_cmd_name) : cmd_name(_cmd_name) {}
  uint16_t has_cmd() { return cmd_code >= 0; }
  uint8_t code() { return static_cast<uint8_t>(cmd_code); }
protected:
  bool send() {
    print(F("sending gcc (code=0) for cmd ")); println(cmd_name);
    return am.send(static_cast<uint8_t>(0)) && am.send(cmd_name) && am.send_packet();
  }
  bool recv() {
    if (!am.recv(&cmd_code) || !am.end_cmd()) return false;
    print(F("gcc received ")); println(static_cast<int>(cmd_code));
    return true;
  }
private:
  int16_t cmd_code = -1;
};

BCStage_gcc bc_gcc_argc("argc");

class BCStage_argc : public BCStage {
protected:
  bool send() {
    print(F("sending argc (code=")); print(static_cast<int>(bc_gcc_argc.code())); println(F(") with 6 bytes"));
    return am.send(bc_gcc_argc.code()) && am.send(static_cast<uint8_t>(42)) && am.send(3.14f) && am.send_packet();
  }
  bool recv() {
    uint8_t argc; const uint8_t expected = 6;
    if (!am.recv(&argc) || !am.end_cmd()) return false;
    if (argc != expected) print(F("ERROR: "));
    print(F("argc received ")); print(static_cast<int>(argc)); println(F(", expected expected"));
    return true;
  }
};

BCStage_argc bc_argc;

BCStage_gcc bc_gcc_sfp("sfp");
BCStage_gcc bc_gcc_gfp("gfp");

class BCStage_sfp_gfp : public BCStage {
public:
  BCStage_sfp_gfp(const float _val) : val(_val) {}
protected:
  bool send() {
    print(F("sending sfp (code=")); print(static_cast<int>(bc_gcc_sfp.code())); print(F(") value=")); println(val);
    if (!am.send(bc_gcc_sfp.code()) || !am.send(val) || !am.send_packet()) return false; //1 + 1 + 4 + 1 = 7 bytes
    
    print(F("sending gfp (code=")); print(static_cast<int>(bc_gcc_gfp.code())); println(F(")"));
    return am.send(bc_gcc_gfp.code()) && am.send_packet(); //1 + 1 + 1 = 3 bytes
    
    //in setup() we called am.set_send_wait_ms(AM::ALWAYS_WAIT)
    //so the above sends all block until the data can be put into the client's serial send buffer
    //thus, we just rapidly sent two packets totalling 7 + 3 = 10 bytes, which should be OK because the
    //server's serial receive buffer should be at least 64 bytes; now we wait for a response to establish flow control
  }
  bool recv() {
    float param;
    if (!am.recv(&param) || !am.end_cmd()) return false;
    if (param != val) print(F("ERROR: "));
    print(F("gfp received ")); print(param); print(F(", expected ")); println(val);
    return true;
  }
private:
  const float val;
};

BCStage_sfp_gfp bc_sfp_gfp_pi(3.14);
BCStage_sfp_gfp bc_sfp_gfp_minus_e(-2.71);

//TODO more stages

BCStage_gcc bc_gcc_quit("quit");

class BCStage_done : public BCStage {
protected:
  bool send() {
    println(F("binary client done"));
#ifndef ARDUINO
    quit = true; //quit is defined in demo.cpp; if we are the binary client, this will cause us to terminate
#endif
    //native demo.cpp implements quit command; see if that's present on server, and if so, invoke it to terminate server
    if (!bc_gcc_quit.has_cmd()) return true;
    print(F("sending quit (code=")); print(static_cast<int>(bc_gcc_quit.code())); println(F(")"));
    return am.send(bc_gcc_quit.code()) && am.end_cmd();
  }
  bool recv() { return true; }
};

BCStage_done bc_done;

#undef BC_GCC

#endif //BINARY_CLIENT

/* Arduino setup() and loop() *****************************************************************************************/

void setup() {

//configure Arduino serial ports
#ifdef ARDUINO
  Serial.begin(BAUD); //default hardware serial (i.e. usb port) is used in all cases
  //binary client and server can optionally also use a separate hardware or software serial port
  //for the binary connection between two Arduinos
#if BINARY && defined(BIN_RX_PIN)
#ifdef ESP32
  AM_STREAM.begin(BAUD, SERIAL_8N1, BIN_RX_PIN, BIN_TX_PIN);
#else
  AM_STREAM.begin(BAUD);
#endif //ESP32
#endif //BINARY && defined(BIN_RX_PIN)
#endif //ARDUINO

#ifndef BASELINE_MEM
#ifdef BINARY_CLIENT
  am.set_send_wait_ms(AM::ALWAYS_WAIT);
#else
  add_cmds(); //text or binary server
#endif
#endif //BASELINE_MEM
}

void loop() {
#ifndef BASELINE_MEM
  am.update();
#ifndef BINARY_CLIENT
  timer.tick(am); //text or binary server
#else //binary client: crank the state machine
  BCStage *next; if (current_bc_stage && (next = current_bc_stage->update())) { current_bc_stage = next; }
#endif //BINARY_CLIENT
#endif //BASELINE_MEM
}

