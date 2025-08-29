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

  //returns pointer to next stage when ended, else null
  BCStage* update() {
    if (!started) { start(); return 0; }
    if (ended) return next;
    if (am.get_universal_runnable() == this) return 0; //still running
    ended = true;
    return next;
  }

  bool run(AM& am) {
    ++run_num;
    if (is_ended()) am.set_universal_runnable(0);
    const bool ret = recv();
    if (!ret) print_error();
    return ret;
  }

protected:

  virtual bool send() = 0;
  virtual bool recv() = 0;
  virtual bool is_ended() { return run_num > 0; }

private:

  BCStage *next;
  bool started = false, ended = false;
  uint16_t run_num = 0;

  void start() {
    started = true;
    am.set_universal_runnable(this); //install ourself as receive runnable
    if (!send()) print_error();
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
  uint8_t code() { return static_cast<uint8_t>(cmd_code); }
protected:
  bool send() {
    print(F("sending gcc (code=0) for cmd ")); println(cmd_name);
    return am.send(static_cast<uint8_t>(0)).send(cmd_name).send_packet();
  }
  bool recv() {
    if (!am.recv(&cmd_code).end_cmd()) return false;
    print(F("gcc received ")); println(static_cast<int>(cmd_code));
    return cmd_code >= 0;
  }
private:
  int16_t cmd_code = -1;
};

BCStage_gcc bc_gcc_argc("argc");

class BCStage_argc : public BCStage {
protected:
  bool send() {
    print(F("sending argc (code=")); print(static_cast<int>(bc_gcc_argc.code())); println(F(") with 6 bytes"));
    return am.send(bc_gcc_argc.code()).send(static_cast<uint8_t>(42)).send(3.14f).send_packet();
  }
  bool recv() {
    uint8_t argc; const uint8_t expected = 6;
    if (!am.recv(&argc).end_cmd()) return false;
    if (argc != expected) print(F("ERROR: "));
    print(F("argc received ")); print(static_cast<int>(argc)); print(F(", expected ")); println(expected);
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
  bool send() {
    print(F("sending sfp (code=")); print(static_cast<int>(bc_gcc_sfp.code())); print(F(") value=")); println(val);
    if (!am.send(bc_gcc_sfp.code()).send(val).send_packet()) return false; //1 + 1 + 4 + 1 = 7 bytes
    
    print(F("sending gfp (code=")); print(static_cast<int>(bc_gcc_gfp.code())); println(F(")"));
    return am.send(bc_gcc_gfp.code()).send_packet(); //1 + 1 + 1 = 3 bytes
    
    //in setup() we called am.set_send_wait_ms(AM::ALWAYS_WAIT)
    //so the above sends all block until the data can be put into the client's serial send buffer
    //thus, we just rapidly sent two packets totalling 7 + 3 = 10 bytes, which should be OK because the
    //server's serial receive buffer should be at least 64 bytes; now we wait for a response which gives us flow control
  }
  bool recv() {
    float param;
    if (!am.recv(&param).end_cmd()) return false;
    if (param != val) print(F("ERROR: "));
    print(F("gfp received ")); print(param); print(F(", expected ")); println(val);
    return param == val;
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
    print(F("binary client done, ")); print(num_errors); println(F(" total errors"));
    done = true; //we are the binary client, this will cause us to terminate
    //invoke quit command to terminate server
    print(F("sending quit (code=")); print(static_cast<int>(bc_gcc_quit.code())); println(F(")"));
    return am.send(bc_gcc_quit.code()).send_packet();
  }
  bool recv() { return true; }
};

BCStage_done bc_done;
