/**
 * ArduMon: Yet another Arduino serial command library.
 *
 * See https://github.com/martyvona/ArduMon/blob/main/README.md
 *
 * This is the binary client state machine, it's designed to be included only in demo.h.  In the basic text mode demo
 * the user is the "client", interacting as desired with the ArduMon "server" through a serial terminal. But in the
 * binary mode demo the client is its own program, running either on the host PC or another Arduino, and the interaction
 * is the fixed state machine defined below.
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

#include "dbg_print.h"

/* binary client state machine ****************************************************************************************/

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

  virtual ~BCStage() {}

  //returns pointer to next stage when ended, else null
  BCStage* update(AM& am) { return !started ? start(am) : done(am) ? next : 0; }

  bool run(AM& am) {
    ++num_receives;
    if (!recv(am)) return false;
    if (done(am) && !remove_handler(am)) return false;
    return true;
  }

protected:

  BCStage *next;
  bool started = false, ended = false;
  uint16_t num_receives = 0;

  BCStage *start(AM& am) {
    if (!add_handler(am) || !send(am) || am.has_err()) { print(AM::err_msg(am.clear_err())); println(); }
    started = true;
    return 0;
  }

  virtual bool send(AM& am) = 0;
  virtual bool recv(AM& am) = 0;
  virtual bool done(AM& am) { return num_receives > 0; }
  virtual bool add_handler(AM& am)    { return am.set_universal_runnable(this); }
  virtual bool remove_handler(AM& am) { return am.set_universal_runnable(0); }
};

//there are several ways for the client and server to know that the code for e.g. the "argc" command is 2
//one approach is just to hardcode that into both the client and server, e.g. in a shared header file
//another approach is for the server to implement a "gcc" command that will return the code for a given command name
//and register the gcc command with a well-known command code, e.g. 0; this latter approach is demonstrated here

class BCStage_gcc : public BCStage {
public:
  const char * const cmd_name;
  BCStage_gcc(const char *_cmd_name) : cmd_name(_cmd_name) {}
  uint8_t code() { return static_cast<uint8_t>(cmd_code); }
protected:
  bool send(AM& am) {
    print(F("sending gcc (0) for cmd ")); print(cmd_name); println();
    return am.send(static_cast<uint8_t>(0)).send(cmd_name).send_packet();
  }
  bool recv(AM& am) {
    if (!am.recv(&cmd_code).end_handler()) return false;
    print(F("gcc received ")); print(static_cast<int>(cmd_code)); println();
    return cmd_code >= 0;
  }
private:
  int16_t cmd_code = -1;
};

BCStage_gcc bc_gcc_argc("argc");

class BCStage_argc : public BCStage {
protected:
  bool send(AM& am) {
    print(F("sending argc (")); print(static_cast<int>(bc_gcc_argc.code())); print(F(") with 6 bytes")); println();
    return am.send(bc_gcc_argc.code()).send(static_cast<uint8_t>(42)).send(3.14f).send_packet();
  }
  bool recv(AM& am) {
    uint8_t argc; const uint8_t expected = 6;
    if (!am.recv(&argc).end_handler()) return false;
    if (argc != expected) print(F("ERROR: "));
    print(F("argc received ")); print(static_cast<int>(argc)); print(F(", expected ")); print(expected); println();
    return argc == expected;
  }
};

BCStage_argc bc_argc;

BCStage_gcc bc_gcc_sfp("sfp");
BCStage_gcc bc_gcc_gfp("gfp");

class BCStage_sfp_gfp : public BCStage {
public:
  BCStage_sfp_gfp(const float _val) : val(_val) {}
protected:
  bool send(AM& am) {
    print(F("sending sfp (")); print(static_cast<int>(bc_gcc_sfp.code())); print(F(") value=")); print(val); println();
    if (!am.send(bc_gcc_sfp.code()).send(val).send_packet()) return false; //1 + 1 + 4 + 1 = 7 bytes
    
    print(F("sending gfp (")); print(static_cast<int>(bc_gcc_gfp.code())); print(F(")")); println();
    return am.send(bc_gcc_gfp.code()).send_packet(); //1 + 1 + 1 = 3 bytes
    
    //in setup() we called am.set_send_wait_ms(AM::ALWAYS_WAIT)
    //so the above sends all block until the data can be put into the client's serial send buffer
    //thus, we just rapidly sent two packets totalling 7 + 3 = 10 bytes, which should be OK because the
    //server's serial receive buffer should be at least 64 bytes; now we wait for a response which gives us flow control
  }
  bool recv(AM& am) {
    float param;
    if (!am.recv(&param).end_handler()) return false;
    if (param != val) print(F("ERROR: "));
    print(F("gfp received ")); print(param); print(F(", expected ")); print(val); println();
    return param == val;
  }
private:
  const float val;
};

BCStage_sfp_gfp bc_sfp_gfp_pi(3.14);
BCStage_sfp_gfp bc_sfp_gfp_minus_e(-2.71);

BCStage_gcc bc_gcc_ts("ts");
BCStage_gcc bc_gcc_tg("tg");

class BCStage_timer : public BCStage {
public:
  BCStage_timer(const uint8_t _h, const uint8_t _m, const uint8_t _s, const float _accel = 1,
                const int16_t _throttle_ms = 500, const uint16_t _resp_code = -1)
    : h(_h), m(_m), s(_s), accel(_accel), throttle_ms(_throttle_ms), resp_code(_resp_code) {}

protected:

  bool send(AM& am) {
    last_send = millis();
    if (!started) {
      print(F("sending ts (")); print(static_cast<int>(bc_gcc_ts.code())); print(F(")"));
      print(", h="); print(h); print(", m="); print(m); print(", s="); print(s); print(", accel="); print(accel);
      print(", sync_throttle_ms="); print(throttle_ms); print(", bin_response_code="); print(resp_code); println();
      return
        am.send(bc_gcc_ts.code()).send(h).send(m).send(s).send(accel).send(throttle_ms).send(resp_code).send_packet();
    } else {
      print(F("sending tg (")); print(static_cast<int>(bc_gcc_tg.code())); print(F(")")); println();
      return am.send(bc_gcc_tg.code()).send_packet();
    }
  }

  bool recv(AM& am) {
    if (resp_code >= 0 && resp_code <= 255 && !am.recv()) return false; //skip command code
    uint32_t total_ms, elapsed_ms;
    if (!am.recv(&total_ms).recv(&elapsed_ms).recv(&remaining_ms).end_handler()) return false;
    print(F("received total_ms=")); print(total_ms); print(F(", elapsed_ms=")); print(elapsed_ms);
    print(F(", remaining_ms=")); print(remaining_ms); println();
    return true;
  }

  bool done(AM& am) {
    if (num_receives > 0 && remaining_ms == 0) return true;
    if (throttle_ms < 0 && (millis() - last_send) >= -throttle_ms && (!send(am) || am.has_err()))
      { print(AM::err_msg(am.clear_err())); println(); }
    return false;
  }

  bool add_handler(AM& am) {
    if (resp_code >= 0 && resp_code <= 255) return am.add_cmd(this, static_cast<uint8_t>(resp_code));
    else return am.set_universal_runnable(this);
  }

  bool remove_handler(AM& am) {
    if (resp_code >= 0 && resp_code <= 255) return am.remove_cmd(static_cast<uint8_t>(resp_code));
    else return am.set_universal_runnable(0);
  }

private:
  const uint8_t h, m, s;
  const float accel;
  const int16_t throttle_ms, resp_code; //throttle_ms >= 0 means sync; otherwise async
  uint32_t remaining_ms = 0;
  uint64_t last_send;
};

BCStage_timer bc_timer(0, 0, 10); //10s timer in synchronous response mode, no accel, no response code
BCStage_timer bc_timer_acc(0, 0, 10, 2); //same as above except 2x accel
BCStage_timer bc_timer_code(0, 0, 10, 2, 1000, 31); //same as above except 1s throttle and resp_code=31
BCStage_timer bc_timer_async(0, 0, 10, 2, -1000, 31); //same as above except async

//TODO more stages

BCStage_gcc bc_gcc_quit("quit");

class BCStage_done : public BCStage {
protected:
  bool send(AM& am) {
    print(F("binary client done, ")); print(num_errors); print(F(" total errors")); println();
    demo_done = true; //we are the binary client, this will cause us to terminate
    print(F("sending quit (")); print(static_cast<int>(bc_gcc_quit.code())); print(F(")")); println();
    return am.send(bc_gcc_quit.code()).send_packet(); //invoke quit command to terminate server
  }
  bool recv(AM& am) { return true; }
};

BCStage_done bc_done;
