#ifndef ARDUMON_H
#define ARDUMON_H

/**
 * ArduMon: Yet another Arduino serial command library.
 *
 * See https://github.com/martyvona/ArduMon/blob/main/README.md
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

#include <stddef.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>

//ESP32, STM32, and native do not have dtostre(), use snprintf() instead
#if !defined(ARDUINO) || !defined(__AVR__)
#include <stdio.h>
#endif

#if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "only little endian architectures are supported"
#endif

#ifndef ARDUINO
class ArduMonStream {
public:
  virtual ~ArduMonStream() {}
  virtual int16_t available() = 0;
  virtual int16_t read() = 0; //-1 if no data available
  virtual int16_t peek() = 0; //-1 if no data available
  virtual int16_t availableForWrite() = 0;
  virtual uint16_t write(uint8_t byte) = 0; //returns 1
};
#endif

//max_num_cmds is the maximum number of commands that can be registered
//
//recv_buf_sz is the recieve buffer size in bytes
//in text mode the receive buffer must be large enough to hold the largest commandline
//command history in text mode requires a receive buffer is large enough to hold both the current and the previous cmd
//in binary mode the receive buffer must be large enough to hold the largest incoming packet (limited to 256 bytes)
//recv_buf_sz can be set to 0 for an application that only sends (unusual, but possible)
//(an array cannot have 0 length but we handle that internally by substituting 1 for 0 when we create the buffer)
//
//send_buf_sz is the send buffer size in bytes
//the send buffer is not used in text mode; send_buf_sz can be set to 0 if binary mode will not be used
//in binary mode the send buffer must be large enough to hold the largest outgoing packet (limited to 256 bytes)
//send_buf_sz can be set to 0 for an application that only receives
//
//with_int64 = false saves ~200 bytes on AVR if you don't need (u)int64 support
//with_float = false saves ~3k bytes on AVR if you don't need float or double support
//with_double = false only saves program space if sizeof(double) > sizeof(float) which is not true by default on AVR
//
//with_binary = false saves ~700 bytes on AVR
//with_text = false saves ~8k bytes on AVR
template <uint8_t max_num_cmds = 8, uint16_t recv_buf_sz = 128, uint16_t send_buf_sz = 128,
          bool with_int64 = true, bool with_float = true, bool with_double = true,
          bool with_binary = true, bool with_text = true>
class ArduMon {
public:

  using millis_t = unsigned long;

#ifndef ARDUINO
  using Stream = ArduMonStream;
#endif

  using FSH = __FlashStringHelper;

  //handler_t is a pointer to a function taking reference to ArduMon object and returning void
  //if the return is false then the handler failed, end_handler() will be automatically called if necessary
  //if the return is true then the handler succeded so far; it may or may not have called end_handler()
  //the command is still being handled until end_handler() is called
  using handler_t = bool (*)(ArduMon&);

  //functioniod (https://isocpp.org/wiki/faq/pointers-to-members#functionoids) alternative to handler_t
  struct Runnable { virtual bool run(ArduMon&) = 0; virtual ~Runnable() {} };

  enum class Error : uint8_t {
    NONE,
    CMD_OVERFLOW,   //already have max_num_cmds or duplicate command
    RECV_OVERFLOW,  //received command longer than recv_buf_sz
    RECV_UNDERFLOW, //received command shorter than expected
    RECV_TIMEOUT,   //longer than recv_timeout_ms between receiving the first and last bytes of a command
    SEND_OVERFLOW,  //handler attempted to send while send buffer full
    BAD_CMD,        //received command unknown
    BAD_ARG,        //received data failed to parse as expected type
    BAD_HANDLER,    //handler failed
    BAD_PACKET,     //invalid received checksum or packet length < 2 in binary mode
    PARSE_ERR,      //text command parse error, e.g. unterminated string
    UNSUPPORTED     //unsupported operation, e.g. recv(int64_t) but !with_int64
  };

  static const FSH *err_msg(const Error e) {
    switch (e) {
      case Error::NONE: return F("(none)");
      case Error::CMD_OVERFLOW: return F("command overflow");
      case Error::RECV_OVERFLOW: return F("receive overflow");
      case Error::RECV_UNDERFLOW: return F("receive underflow");
      case Error::RECV_TIMEOUT: return F("receive timeout");
      case Error::SEND_OVERFLOW: return F("send_overflow");
      case Error::BAD_CMD: return F("bad command");
      case Error::BAD_ARG: return F("bad argument");
      case Error::BAD_HANDLER: return F("handler error");
      case Error::BAD_PACKET: return F("bad packet");
      case Error::PARSE_ERR: return F("parse error");
      case Error::UNSUPPORTED: return F("unsupported operation");
      default: return F("(unknown error)");
    }
  }

  explicit ArduMon(Stream *s, const bool binary = !with_text) : stream(s) {
    set_binary_mode_impl(binary, true, false);
    set_universal_handler(0);
    set_fallback_handler(0);
    memset(cmds, 0, sizeof(cmds));
  }

#ifdef ARDUINO
  explicit ArduMon(const bool binary = !with_text) : ArduMon(&Serial, binary) { }
#endif

  ArduMon(const ArduMon&) = delete;
  ArduMon& operator=(const ArduMon&) = delete;
  ArduMon(ArduMon&&) = delete;
  ArduMon& operator=(ArduMon&&) = delete;

  //get the underlying stream, e.g. for direct use in command handlers
  Stream *get_stream() { return stream; }

  //get the number of registered commands
  //this will also be the binary code of the next command that will be added with add_cmd() without an explicit code
  uint8_t get_num_cmds() { return n_cmds; }

  //get the maximum number of commands that could be registered
  uint8_t get_max_num_cmds() { return max_num_cmds; }

  //get the send buffer size in bytes
  uint16_t get_send_buf_size() { return send_buf_sz; }

  //binary mode: get packet size if currently sending a packet, else get number of bytes used so far in send buffer
  //text mode: return 0
  uint16_t get_send_buf_used() { return send_buf_used(); }

  //sugar for get_send_buf_size() - get_send_buf_used()
  uint16_t get_send_buf_free() { return send_buf_sz - send_buf_used(); }

  //get the receive buffer size in bytes
  uint16_t get_recv_buf_size() { return recv_buf_sz; }

  //if not currently receiving or handling a command then return 0
  //if curently receiving a command then get number of bytes received so far
  //if handling in binary mode return received packet size
  //if handling in text mode return length of received command string
  uint16_t get_recv_buf_used() { return recv_buf_used(); }

  //sugar for get_recv_buf_size() - get_recv_buf_used()
  uint16_t get_recv_buf_free() { return recv_buf_sz - recv_buf_used(); }

  bool has_err() const { return err != Error::NONE; }

  //bool conversion operator; returns !has_err()
  operator bool() const { return !has_err(); }

  Error get_err() { return err; }

  //return and clear any current error
  Error clear_err() { Error was = err; err = Error::NONE; return was; }

  //if has_err() and there is an error handler, run it
  ArduMon& handle_err() { return handle_err_impl(); }

  //returns a default error handler that
  //* sends the error message back to the user in text mode
  //* sends the error message to the default Serial port on Arduino in binary mode (unless that Serial port is also the
  //  stream used for binary communication, in which case the error message is swallowed)
  //* prints the error message to the console otherwise (native binary mode)
  handler_t get_default_error_handler() {
    return [](ArduMon &am){
      if (am.is_binary_mode()) {
#if ARDUINO
        if (am.stream != &Serial) Serial.println(am.err_msg(am.clear_err()));
#else
        printf("%s\n", am.err_msg(am.clear_err()));
#endif
      } else {
        const Error e = am.clear_err();
        am.send_CRLF().send_raw(am.err_msg(e)).send_CRLF(true); 
      }
      return true;
    };
  }

  //install the default error handler returned by get_default_error_handler()
  ArduMon& set_default_error_handler() { return set_error_handler(get_default_error_handler()); }

  //set an error handler that will be called automatically during end_handler() if has_err()
  //if the error handler returns true then clear_err() will be automatically called
  ArduMon&  set_error_handler(const handler_t h)  { return set_handler (error_handler,  h, F_ERROR_RUNNABLE); }
  handler_t get_error_handler()                   { return get_handler (error_handler,     F_ERROR_RUNNABLE); }
  ArduMon&  set_error_runnable(Runnable* const r) { return set_runnable(error_runnable, r, F_ERROR_RUNNABLE); }
  Runnable* get_error_runnable()                  { return get_runnable(error_runnable,    F_ERROR_RUNNABLE); }

  //set a universal command handler that will override any other handlers added with add_cmd()
  //this can be useful e.g. in binary mode to handle received packets where byte two is not necessarily a command code
  //set handler to 0 to remove any existing universal handler (and thus re-enable handlers added with add_cmd())
  ArduMon&  set_universal_handler(const handler_t h)  { return set_handler (universal_handler,  h, F_UNIV_RUNNABLE); }
  handler_t get_universal_handler()                   { return get_handler (universal_handler,     F_UNIV_RUNNABLE); }
  ArduMon&  set_universal_runnable(Runnable* const r) { return set_runnable(universal_runnable, r, F_UNIV_RUNNABLE); }
  Runnable* get_universal_runnable()                  { return get_runnable(universal_runnable,    F_UNIV_RUNNABLE); }

  //set a fallback command handler that will handle received commands
  //that did not have a command name (command code in binary mode) matching any handler added with add_cmd()
  //set handler to 0 to remove any existing fallback handler
  ArduMon&  set_fallback_handler(const handler_t h)  { return set_handler (fallback_handler,  h, F_FALLBACK_RUNNABLE); }
  handler_t get_fallback_handler()                   { return get_handler (fallback_handler,     F_FALLBACK_RUNNABLE); }
  ArduMon&  set_fallback_runnable(Runnable* const r) { return set_runnable(fallback_runnable, r, F_FALLBACK_RUNNABLE); }
  Runnable* get_fallback_runnable()                  { return get_runnable(fallback_runnable,    F_FALLBACK_RUNNABLE); }

  //add a command: name may be null, but if not, it must be unique relative to already added commands
  //code must be unique relative to already added commands
  //CMD_OVERFLOW if command with the given name (if any) or code already exists, or if max_num_cmds already added
  ArduMon& add_cmd(const handler_t handler, const char *name, const uint8_t code, const char *description = 0) {
    return add_cmd_impl(handler, name, code, description, false);
  }

  //add a command using the next available command code
  ArduMon& add_cmd(const handler_t handler, const char *name, const char *description = 0) {
    return add_cmd_impl(handler, name, n_cmds, description, false);
  }

  //add a command with null name, for binary mode use only
  ArduMon& add_cmd(const handler_t handler, const uint8_t code, const char *description = 0) {
    return add_cmd_impl(handler, 0, code, description, false);
  }

  //add a command with a Runnable instead of a handler_t
  ArduMon& add_cmd(Runnable* const runnable, const char *name, const uint8_t code, const char *description = 0) {
    return add_cmd_impl(runnable, name, code, description, false);
  }

  //add a command using the next available command code
  ArduMon& add_cmd(Runnable* const runnable, const char *name, const char *description = 0) {
    return add_cmd_impl(runnable, name, n_cmds, description, false);
  }

  //add a command with null name, for binary mode use only
  ArduMon& add_cmd(Runnable* const runnable, const uint8_t code, const char *description = 0) {
    return add_cmd_impl(runnable, 0, code, description, false);
  }

  //remove command registered with given code
  ArduMon& remove_cmd(const uint8_t code) { return remove_cmd_impl(code); }

  //remove command registered with given name
  ArduMon& remove_cmd(const char *name) { return remove_cmd_impl(name); }

  //remove command registered with given handler
  ArduMon& remove_cmd(const handler_t handler) { return remove_cmd_impl(handler); }

  //remove command registered with given runnable
  ArduMon& remove_cmd(Runnable* const runnable) { return remove_cmd_impl(runnable); }

#ifdef ARDUINO
  //add a command with strings from program memory
  ArduMon& add_cmd(const handler_t handler, const FSH *name, const uint8_t code, const FSH *description = 0) {
    return add_cmd_impl(handler, CCS(name), code, CCS(description), true);
  }

  //add a command using the next available command code with strings from program memory
  ArduMon& add_cmd(const handler_t handler, const FSH *name, const FSH *description = 0) {
    return add_cmd_impl(handler, CCS(name), n_cmds, CCS(description), true);
  }

  //add a command with strings from program memory
  ArduMon& add_cmd(Runnable* const runnable, const FSH *name, const uint8_t code, const FSH *description = 0) {
    return add_cmd_impl(runnable, CCS(name), code, CCS(description), true);
  }

  //add a command using the next available command code with strings from program memory
  ArduMon& add_cmd(Runnable* const runnable, const FSH *name, const FSH *description = 0) {
    return add_cmd_impl(runnable, CCS(name), n_cmds, CCS(description), true);
  }

  //remove command registered with given name and return true if it was found, false if it was not found
  ArduMon& remove_cmd(const FSH *name) { return remove_cmd_impl(name); }
#endif

  //get the command code for a command name; returns -1 if not found
  int16_t get_cmd_code(const char *name) { return get_cmd_code_impl<char>(name); }
#ifdef ARDUINO
  int16_t get_cmd_code(const FSH *name) { return get_cmd_code_impl<FSH>(name); }
#endif

  //get the commad name for a command code; returns null if not found
  //the returned pointer will be in program memory on AVR if and only if the command was originally registered that way 
  const char * get_cmd_name(const uint8_t code) {
    for (uint8_t i = 0; i < n_cmds; i++) if (cmds[i].code == code) return cmds[i].name;
    return 0;
  }

  //noop in binary mode
  //in text mode send one line per command: cmd_name cmd_code_hex cmd_description
  ArduMon& send_cmds() { return send_cmds_impl(); }

  //does nothing if already in the requested mode
  //otherwise the command interpreter and send and receive buffers are reset
  //if the new mode is text and there is a prompt it is sent
  //Error::UNSUPPORTED if binary but !with_binary or !binary but !with_text
  ArduMon& set_binary_mode(const bool binary) { return set_binary_mode_impl(binary); }

  //the command interpreter and send and receive buffers are reset
  //in text mode if there is a prompt it is sent
  ArduMon& reset() { return set_binary_mode_impl(binary_mode, true, true); }

  bool is_binary_mode() { return binary_mode; }
  bool is_txt_mode() { return !binary_mode; }

  //enable or disable received character echo in text mode
  ArduMon& set_txt_echo(const bool echo) {
    if (echo) flags |= F_TXT_ECHO; else flags &= ~F_TXT_ECHO;
    return *this;
  }

  //set prompt to NULL to disable it
  //otherwise the new prompt is sent immediately in text mode iff a handler is not currently running
  ArduMon& set_txt_prompt(const char *prompt) {
    txt_prompt = prompt;
    flags &= ~F_TXT_PROMPT_PROGMEM;
    return send_txt_prompt();
  }

#ifdef ARDUINO
  //set text prompt from a program memory string
  ArduMon& set_txt_prompt(const FSH *prompt) {
    txt_prompt = CCS(prompt);
    flags |= F_TXT_PROMPT_PROGMEM;
    return send_txt_prompt();
  }
#endif

  static const millis_t ALWAYS_WAIT = -1; //-1 in base 2 is all 1s as unsigned

  //set receive timeout
  //if a command starts being received but is not finished by this timeout the command interpreter will reset
  //this probably makes more sense for automation than for interactive use
  //if a command is currently being received the new timeout will not apply until the next command
  //set to 0 or ALWAYS_WAIT to disable the timeout (it's disabled by default)
  ArduMon& set_recv_timeout_ms(const millis_t ms) { recv_timeout_ms = ms == ALWAYS_WAIT ? 0 : ms; return *this; }
  millis_t get_recv_timeout_ms() { return recv_timeout_ms; }

  //block for up to this long in send_packet() in binary mode, default 0, use ALWAYS_WAIT to block indefinitely
  //text mode sends always block until space is available in the Arduino serial send buffer
  ArduMon& set_send_wait_ms(const millis_t ms) { send_wait_ms = ms; return *this; }
  millis_t get_send_wait_ms() { return send_wait_ms; }

  //this must be called from the Arduino loop() method
  //receive available input bytes from serial stream
  //if the end of a command is received then dispatch and handle it
  //in binary mode then try to send remaining response packet bytes without blocking
  ArduMon& update() { return update_impl(); }

  //reset the command interpreter and the receive buffer
  //if has_err() and there is an error handler (or Runnable) then run it
  //in text mode then send the prompt, if any
  //in binary mode send_packet()
  ArduMon& end_handler() { return end_handler_impl(); }

  //noop in text mode
  //in binary mode if the send buffer is empty then noop
  //otherwise compute packet checksum and length, disable writing send_buf, enable reading it, and start sending it
  //blocks for up to send_wait_ms
  ArduMon& send_packet() { return send_packet_impl(); }

  //check if a packet is still being sent in binary mode
  //do not write additional data to the send buffer while this is the case
  bool is_sending_packet() { return binary_mode && send_write_ptr == 0; }

  //check if a command handler is currently running
  bool is_handling() { return flags&F_HANDLING; }

  //check if the first byte of a command has been received but not yet the full command
  bool is_receiving() { return flags&F_RECEIVING; }

  //text: return number of command arguments, including the command name itself
  //binary: return number of command packet payload bytes + 1 for the command code
  //in either case the return is only valid while handling a command
  uint8_t argc() { return arg_count; }

  //skip the next n tokens in text mode; skip the next n bytes in binary mode
  ArduMon& recv(const uint8_t n = 1) { next_tok(n); return *this; }

  //receive a character
  ArduMon& recv(char *v) {
    const char *ptr = CCS(next_tok(1));
    if (!ptr || (*(ptr + 1) != 0)) return fail(Error::BAD_ARG);
    *v = *ptr;
    return *this;
  }

  //receive a string
  ArduMon& recv(const char* *v) {
    const char *ptr = CCS(next_tok(0));
    if (!ptr) return fail(Error::BAD_ARG);
    *v = ptr;
    return *this;
  }

  //binary mode: receive a byte with value 0 (false) or nonzero (true)
  //text mode: receive "true", "false", "t", "f", "0", "1", "yes", "no", "y", "n" or uppercase equivalents
  ArduMon& recv(bool *v) { return parse_bool(next_tok(1), v); }

  //binary mode: receive an integer of the indicated size
  //text mode: receive a decimal or hexadecimal integer
  //if hex == true then always interpret as hex in text mode, else interpret as hex iff prefixed with 0x or 0X
  //Error::UNSUPPORTED if recv([u]int64_t) but !with_int64
  ArduMon& recv( uint8_t *v, const bool hex = false) { return parse_int(next_tok(1), BP(v), false, 1, hex); }
  ArduMon& recv(  int8_t *v, const bool hex = false) { return parse_int(next_tok(1), BP(v), true,  1, hex); }
  ArduMon& recv(uint16_t *v, const bool hex = false) { return parse_int(next_tok(2), BP(v), false, 2, hex); }
  ArduMon& recv( int16_t *v, const bool hex = false) { return parse_int(next_tok(2), BP(v), true,  2, hex); }
  ArduMon& recv(uint32_t *v, const bool hex = false) { return parse_int(next_tok(4), BP(v), false, 4, hex); }
  ArduMon& recv( int32_t *v, const bool hex = false) { return parse_int(next_tok(4), BP(v), true,  4, hex); }
  ArduMon& recv(uint64_t *v, const bool hex = false) { return parse_int(next_tok(8), BP(v), false, 8, hex); }
  ArduMon& recv( int64_t *v, const bool hex = false) { return parse_int(next_tok(8), BP(v), true,  8, hex); }

  //binary mode: receive float or double
  //text mode: receive a decimal or scientific float or double
  //on AVR double is synonymous with float by default, both are 4 bytes; otherwise double may be 8 bytes
  //Error::UNSUPPORTED if sizeof(float) != sizeof(double) and recv(double) but !with_double
  ArduMon& recv(float *v) { return parse_float(next_tok(4), v); }
  ArduMon& recv(double *v) { return parse_float(next_tok(sizeof(double)), v); }

  //sends carriage return and line feed in text mode; noop in binary mode
  ArduMon& send_CRLF(bool force = false) {
    if (binary_mode || !with_text) return *this;
    if (force || (flags&F_SPACE_PENDING)) write_char('\r').write_char('\n');
    flags &= ~F_SPACE_PENDING;
    return *this;
  }

  //binary mode: send a single character (8 bit clean)
  //text mode: send space separator if necessary, then send character with quote and escape iff nessary
  ArduMon& send(const char v) { return send_txt_sep().write_char(v, true); }

  //binary and text mode: send a single character (8 bit clean)
  ArduMon& send_raw(const char v) { return write_char(v); }

  //binary mode: append null terminated string to send buffer, including terminating null
  //text mode: send space sep if necessary, then send string with quote and escape iff necessary, w/o terminating null
  ArduMon& send(const char* v) { return send_txt_sep().write_str(v, false, true); }
#ifdef ARDUINO
  ArduMon& send(const FSH* v) { return send_txt_sep().write_str(CCS(v), true, true); }
#endif

  //binary mode: append null terminated string to send buffer, including terminating null
  //text mode: append string to send buffer, not including terminating null
  //if len >= 0 then send len bytes instead of checking for null terminator in either mode
  //8 bit clean if len >= 0, otherwise mostly 8 bit clean except for value 0 which is interpreted as a terminating null
  ArduMon& send_raw(const char* v, const int16_t len = -1) { return write_str(v, false, false, len); }
#ifdef ARDUINO
  ArduMon& send_raw(const FSH* v, const int16_t len = -1) { return write_str(CCS(v), true, false, len); }
#endif

  enum class BoolStyle : uint8_t { TRUE_FALSE, TF, ZERO_ONE, YES_NO, YN };

  //binary mode: send one byte with value 0 (false) or 1 (true)
  //text mode: send space separator if necessary, then send boolean value in indicated style
  ArduMon& send(const bool v, const BoolStyle style = BoolStyle::TRUE_FALSE, const bool upper_case = false) {
    return send_txt_sep().send_raw(v, style, upper_case);
  }

  //binary mode: send one byte with value 0 (false) or 1 (true)
  //text mode: send boolean value in indicated style
  ArduMon& send_raw(const bool v, const BoolStyle style = BoolStyle::TRUE_FALSE, const bool upper_case = false) {
    return write_bool(v, style, upper_case);
  }

  static const uint8_t FMT_HEX = 0x80, FMT_PAD_ZERO = 0x40, FMT_PAD_RIGHT = 0x20;

  //binary mode: send a little-endian integer of the indicated size
  //text mode: send space separator if necessary, then send decimal or hexadecimal integer
  //fmt ignored in binary; in text mode it is a bitmask of FMT_* flags with low bits specifying the minimum field width
  //with FMT_HEX width can be at most 31
  //otherwise width will be clamped to 21 for [u]int64_t and 11 for the other int types
  ArduMon& send(const  uint8_t v, const uint8_t fmt = 0) { return send_txt_sep().send_raw(v, fmt); }
  ArduMon& send(const   int8_t v, const uint8_t fmt = 0) { return send_txt_sep().send_raw(v, fmt); }
  ArduMon& send(const uint16_t v, const uint8_t fmt = 0) { return send_txt_sep().send_raw(v, fmt); }
  ArduMon& send(const  int16_t v, const uint8_t fmt = 0) { return send_txt_sep().send_raw(v, fmt); }
  ArduMon& send(const uint32_t v, const uint8_t fmt = 0) { return send_txt_sep().send_raw(v, fmt); }
  ArduMon& send(const  int32_t v, const uint8_t fmt = 0) { return send_txt_sep().send_raw(v, fmt); }
  ArduMon& send(const uint64_t v, const uint8_t fmt = 0) { return send_txt_sep().send_raw(v, fmt); }
  ArduMon& send(const  int64_t v, const uint8_t fmt = 0) { return send_txt_sep().send_raw(v, fmt); }

  //binary mode: send an integer of the indicated size
  //text mode: send decimal or hexadecimal integer
  //fmt ignored in binary; in text mode it is a bitmask of FMT_* flags with low bits specifying the minimum field width
  //field width will be clamped to 21 for [u]int64_t and 11 for the other int types
  ArduMon& send_raw(const  uint8_t v, const uint8_t fmt = 0) { return write_int(BP(&v), false, 1, fmt); }
  ArduMon& send_raw(const   int8_t v, const uint8_t fmt = 0) { return write_int(BP(&v), true,  1, fmt); }
  ArduMon& send_raw(const uint16_t v, const uint8_t fmt = 0) { return write_int(BP(&v), false, 2, fmt); }
  ArduMon& send_raw(const  int16_t v, const uint8_t fmt = 0) { return write_int(BP(&v), true,  2, fmt); }
  ArduMon& send_raw(const uint32_t v, const uint8_t fmt = 0) { return write_int(BP(&v), false, 4, fmt); }
  ArduMon& send_raw(const  int32_t v, const uint8_t fmt = 0) { return write_int(BP(&v), true,  4, fmt); }
  ArduMon& send_raw(const uint64_t v, const uint8_t fmt = 0) { return write_int(BP(&v), false, 8, fmt); }
  ArduMon& send_raw(const  int64_t v, const uint8_t fmt = 0) { return write_int(BP(&v), true,  8, fmt); }

  //binary mode: send little-endian float or double bytes
  //text mode: send space separator if necessary, then send float or double as decimal or scientific
  //on AVR both double and float are 4 bytes by default; on other platforms double may be 8 bytes
  //precision is the minimum number of fraction digits (i.e. after the decimal point), right padded with 0s
  //if precision is negative then automatically use the minimum number of fraction digits
  //width only applies to non-scientific; if positive then left-pad the result with spaces to the specified minimum
  //width is limited to 10 for 4 byte float, 18 for 8 byte double
  ArduMon& send(const float v, bool scientific = false, int8_t precision = -1, int8_t width = -1) {
    return send_txt_sep().send_raw(v, scientific, precision, width);
  }
  ArduMon& send(const double v, bool scientific = false, int8_t precision = -1, int8_t width = -1) {
    return send_txt_sep().send_raw(v, scientific, precision, width);
  }

  //binary mode: send little-endian float or double bytes
  //text mode: send float or double as decimal or scientific string
  //see further comments for send(float)
  ArduMon& send_raw(const float v, bool scientific = false, int8_t precision = -1, int8_t width = -1) {
    return write_float(v, scientific, precision, width);
  }
  ArduMon& send_raw(const double v, bool scientific = false, int8_t precision = -1, int8_t width = -1) {
    return write_float(v, scientific, precision, width);
  }

  //there are no actual ASCII codes for these keys
  //instead we re-purpose the ASCII codes for the non-printing characters device control 1-4 and bell
  static const char UP_KEY = 17, DOWN_KEY = 18, RIGHT_KEY = 19, LEFT_KEY = 20, UNKNOWN_KEY = 7;

  //these are actually ASCII "start of heading" and "enquiry"
  //but apparently are what we receive for HOME and END keypresses
  static const char HOME_KEY = 1, END_KEY = 5;

  //get keypress, non-blocking; can be useful in interactive command handlers
  //skips the ArduMon receive buffer and reads directly from the Arduino serial receive buffer
  //returns 0 the receive buffer is empty
  //otherwise returns the first available character from the receive buffer
  //if that character was 27 (ESC) and there are at least two more characters available with the first being '['
  //then attempt to intrpret the triplet as a VT100 escape sequence, e.g. UP_KEY, DOWN_KEY, RIGHT_KEY, LEFT_KEY
  char get_key() { return get_key_impl(); }

  //below are conveniences for a partial set of ANSI/VT100 control codes; for more details see
  //https://vt100.net
  //https://github.com/martyvona/ArduMon/blob/main/vtansi.htm
  //https://github.com/martyvona/ArduMon/blob/main/VT100_Escape_Codes.html

  ArduMon& vt100_clear_right()    { return write_str(F("\x1B[0K")); }
  ArduMon& vt100_clear_line()     { return write_str(F("\r\x1B[2K")); }
  ArduMon& vt100_cursor_visible() { return write_str(F("\x1B[?25h")); }
  ArduMon& vt100_cursor_hidden()  { return write_str(F("\x1B[?25l")); }

  static const char VT100_UP = 'A', VT100_DOWN = 'B', VT100_RIGHT = 'C', VT100_LEFT = 'D';

  //binary mode: noop
  //text mode: move VT100 cursor n places in dir
  ArduMon& vt100_move_rel(const uint16_t n, const char dir) {
    if (binary_mode || !with_text) return *this;
    if (!write_char('\x1B').write_char('[')) return *this;
    if (n < 10 && !send_raw(static_cast<char>('0' + n))) return *this;
    if (n >= 10 && !send_raw(n)) return *this;
    return write_char(dir);
  }

  //binary mode: noop
  //text mode: move VT100 cursor to (row, col)
  ArduMon& vt100_move_abs(const uint16_t row, const uint16_t col) {
    if (binary_mode || !with_text) return *this;
    return write_char('\x1B').write_char('[').send_raw(row).write_char(';').send_raw(col).write_char('H');
  }

  static const char VT100_ATTR_RESET = '0', VT100_ATTR_BRIGHT = '1', VT100_ATTR_UNDERSCORE = '4';
  static const char VT100_ATTR_BLINK = '5', VT100_ATTR_REVERSE = '7';

  //binary mode: noop
  //text mode: set one of the VT100_ATTR_* attributes
  ArduMon& vt100_set_attr(const char attr) {
    if (binary_mode || !with_text) return *this;
    return write_char('\x1B').write_char('[').write_char(attr).write_char('m');
  }

  static const char VT100_FOREGROUND = '3', VT100_BACKGROUND = '4';
  static const char VT100_BLACK = '0', VT100_RED = '1', VT100_GREEN = '2', VT100_YELLOW = '3', VT100_BLUE = '4';
  static const char VT100_MAGENTA = '5', VT100_CYAN = '6', VT100_WHITE = '7';

  //binary mode: noop
  //text mode: set one of the VT100_* colors
  ArduMon& vt100_set_color(const char fg_bg, const char color) {
    if (binary_mode || !with_text) return *this;
    return write_char('\x1B').write_char('[').write_char(fg_bg).write_char(color).write_char('m');
  }

  //Arduino platform includes strcmp_P() but not strcmp_PP() where both arguments are in program memory on AVR
  //adapted from https://github.com/bxparks/AceCommon/blob/develop/src/pstrings/pstrings.cpp
  static int strcmp_PP(const char* a, const char* b) {
    if (a == b) return 0;
    if (!a) return -1;
    if (!b) return 1;
    while (true) {
      const char ca = pgm_read_byte(a++), cb = pgm_read_byte(b++);
      if (ca != cb) return ca - cb;
      if (!ca) return 0;
    }
  }

#ifdef ARDUINO
  static int strcmp_PP(const FSH* a, const FSH* b) { return strcmp_PP(CCS(a), CCS(b)); }
#endif

  //convert the low nybble of i to a hex char 0-9A-F
  static char to_hex(const uint8_t i) { return (i&0x0f) < 10 ? ('0' + (i&0x0f)) : ('A' + ((i&0x0f) - 10)); }

private:

  Stream *stream; //underlying serial stream

  Error err = Error::NONE; //most recent error

  bool binary_mode; //binary mode if true, text mode otherwise

  enum {
    F_TXT_ECHO = 1 << 0, //echo received characters in text mode, typically for interactive terminal use
    F_TXT_PROMPT_PROGMEM = 1 << 1, //text mode prompt string is in program memory on AVR
    F_RECEIVING = 1 << 2, //received the first but not yet last byte of a command
    F_HANDLING = 1 << 3, //a command handler is currently running
    F_SPACE_PENDING = 1 << 4, //a space should be sent before the next returned value in text mode
    F_ERROR_RUNNABLE = 1 << 5, //error_handler is a runnable
    F_UNIV_RUNNABLE = 1 << 6, //universal_handler is a runnable
    F_FALLBACK_RUNNABLE = 1 << 7 //fallback_handler is a runnable
  };
  uint8_t flags = 0;

  const char *txt_prompt = 0; //prompt string in text mode, 0 if none

  millis_t recv_deadline = 0, recv_timeout_ms = 0; //receive timeout, disabled by default

  uint8_t arg_count = 0;

  //unfortunately zero length arrays are technically not allowed
  //though many compilers won't complain unless in pedantic mode
  //send_buf is not used in text mode, and receive-only applications are possible in binary mode
  //recv_buf is required to receive commands in both text and binary mode, but send-only applications are possible
  uint8_t recv_buf[recv_buf_sz > 0 ? recv_buf_sz : 1], send_buf[send_buf_sz > 0 ? send_buf_sz : 1];

  //next available position in recv_buf while receiving a command
  //last received character when beginning to handle a command
  //start of next read while handling command
  uint8_t *recv_ptr = recv_buf;

  //send_buf is only used in binary mode
  //send_read_ptr is the next unsent byte; sending is disabled iff send_read_ptr is 0
  //send_write_ptr is the next free spot; writing to send_buf is disabled if send_write_ptr is 0
  //SEND_OVERFLOW iff send when send_write_ptr == send_buf + send_buf_sz - 1 (reserved for checksum)
  uint8_t *send_read_ptr = 0, *send_write_ptr = send_buf;

  //block for up to this long in pump_send_buf() in binary mode
  millis_t send_wait_ms = 0;

  union { handler_t error_handler; Runnable* error_runnable; };
  union { handler_t universal_handler; Runnable* universal_runnable; };
  union { handler_t fallback_handler; Runnable* fallback_runnable; };

  uint8_t n_cmds = 0;

  struct Cmd {

    const char *name, *description;
    uint8_t code;

    union { handler_t handler; Runnable* runnable; };

    enum { F_PROGMEM = 1 << 0, F_RUNNABLE = 1 << 1 };
    uint8_t flags = 0;

    const bool is(const char *n) { return (flags&F_PROGMEM ? strcmp_P(n, name) : strcmp(n, name)) == 0; }

#ifdef ARDUINO
    const bool is(const FSH *n) {
      return (flags&F_PROGMEM ? ArduMon::strcmp_PP(name, CCS(n)) : strcmp_P(name, CCS(n))) == 0;
    }
#endif

    const bool is(const uint8_t c) { return code == c; }

    const bool is(const handler_t h) { return !(flags&F_PROGMEM) && handler == h; }

    const bool is(Runnable* const r) { return (flags&F_PROGMEM) && runnable == r; }
  };

  Cmd cmds[max_num_cmds > 0 ? max_num_cmds : 1];

  ArduMon& fail(Error e) { if (err == Error::NONE) err = e; return *this; }

  ArduMon& set_handler(handler_t &which, const handler_t handler, const uint8_t runnable_flag)  {
    which = handler;
    flags &= ~runnable_flag;
    return *this;
  }

  handler_t get_handler(const handler_t handler, const uint8_t runnable_flag) {
    return (flags&runnable_flag) ? 0 : handler;
  }

  ArduMon& set_runnable(Runnable* &which, Runnable* const runnable, const uint8_t runnable_flag)  {
    which = runnable;
    flags |= runnable_flag;
    return *this;
  }

  Runnable* get_runnable(Runnable* const runnable, const uint8_t runnable_flag) {
    return (flags&runnable_flag) ? runnable : 0;
  }

  template <typename T>
  ArduMon& add_cmd_impl(const T func, const char *name, const uint8_t code, const char *desc, const bool progmem) {
    if (n_cmds == max_num_cmds) return fail(Error::CMD_OVERFLOW);
    for (uint8_t i = 0; i < n_cmds; i++) {
      if (name && ((progmem && cmds[i].is(reinterpret_cast<const FSH*>(name))) || (!progmem && cmds[i].is(name)))) {
        return fail(Error::CMD_OVERFLOW);
      }
      if (cmds[i].code == code) return fail(Error::CMD_OVERFLOW);
    }
    cmds[n_cmds].name = name;
    cmds[n_cmds].description = desc;
    cmds[n_cmds].code = code;
    cmds[n_cmds].flags = progmem ? Cmd::F_PROGMEM : 0;
    set_func(cmds[n_cmds], func);
    ++n_cmds;
    return *this;
  }

  template <typename T> ArduMon& remove_cmd_impl(T &key) {
    for (uint8_t i = 0; i < n_cmds; i++) {
      if (cmds[i].is(key)) {
        for (i++; i < n_cmds; i++) cmds[i - 1] = cmds[i];
        --n_cmds;
        break;
      }
    }
    return *this;
  }

  template <typename T> int16_t get_cmd_code_impl(const T *name) {
    for (uint8_t i = 0; i < n_cmds; i++) if (cmds[i].is(name)) return cmds[i].code;
    return -1;
  }

  void set_func(Cmd& cmd, const handler_t func) { cmd.handler = func; }
  void set_func(Cmd& cmd, Runnable* const func) { cmd.runnable = func; cmd.flags |= Cmd::F_RUNNABLE; }

  //does nothing in binary mode, if txt_prompt is null, or if currently handling
  //otherwise sends optional CRLF followed by txt_prompt and a space
  //this cannot cause an ArduMon error
  ArduMon& send_txt_prompt(const bool with_crlf = false) {
    if (!with_text || binary_mode || !txt_prompt || flags&F_HANDLING) return *this;
    if (with_crlf) write_char('\r').write_char('\n');  //write_char() and write_str() cannot error in text mode
    write_str(txt_prompt, flags&F_TXT_PROMPT_PROGMEM).write_char(' ');
    return *this;
  }

  //noop in binary mode or if has_err()
  //in text mode send a space iff F_SPACE_PENDING
  //in text mode F_SPACE_PENDING is always set before successful return
  ArduMon& send_txt_sep() {
    if (binary_mode || !with_text || has_err()) return *this;
    if (flags&F_SPACE_PENDING) write_char(' ');
    flags |= F_SPACE_PENDING;
    return *this;
  }

  //can't use std::function on AVR
  template<typename T> bool dispatch(const T&pred) {

    bool retval;
    if (invoke(universal_handler, universal_runnable, flags, F_UNIV_RUNNABLE, retval)) return retval;

    for (uint8_t i = 0; i < n_cmds; i++) {
      if (pred(cmds[i])) {
        if (invoke(cmds[i].handler, cmds[i].runnable, cmds[i].flags, Cmd::F_RUNNABLE, retval)) return retval;
        else return false;
      }
    }

    if (invoke(fallback_handler, fallback_runnable, flags, F_FALLBACK_RUNNABLE, retval)) return retval;

    return fail(Error::BAD_CMD).end_handler_impl();
  }

  bool invoke(const handler_t handler, Runnable * const runnable, const uint8_t flags, const uint8_t runnable_flag,
              bool &retval) {
    if      ((flags&runnable_flag) && runnable) { retval = runnable->run(*this); return true; }
    else if (!(flags&runnable_flag) && handler) { retval = handler(*this); return true; }
    return false;
  }

  //upon call, recv_ptr is the last received byte, which should be the checksum
  //if the checksum is invalid then BAD_PACKET
  //otherwise lookup command code, if not found then BAD_CMD
  //otherwise set recv_ptr = recv_buf + 1 and call command handler
  bool handle_bin_command() {
    const uint8_t len = recv_buf[0];
    uint8_t sum = 0; for (uint8_t i = 0; i < len; i++) sum += recv_buf[i];
    if (sum != 0) return fail(Error::BAD_PACKET);
    recv_ptr = recv_buf + 1; //skip over length
    arg_count = len - 2; //don't include length or checksum bytes, but include command code byte in arg count
    return dispatch([&](Cmd& cmd){ return arg_count && cmd.code == recv_buf[1]; });
  }

  //upon call, recv_ptr is the last received character, which will be either '\r' or '\n'
  //tokenize recv_buf, parsing quoted characters and strings with escapes, and discarding any line end comment
  //ignore empty commands
  //otherwise lookup first token as command name, if not found then BAD_CMD
  //otherwise set recv_ptr = recv_buf and call command handler
  bool handle_txt_command() {

    const uint16_t len = recv_ptr - recv_buf + 1;

    if (len <= 1) return end_handler_impl(); //ignore empty command, e.g. if received just '\r' or '\n'

    const bool save_cmd = (len + 1) <= recv_buf_sz/2; //save cmd to upper half of recv_buf if possible for history
    if (save_cmd) recv_buf[recv_buf_sz/2] = '\n'; //saved command is signaled by recv_buf[recv_buf_sz/2] = '\n'

    bool in_str = false, in_chr = false;
    uint16_t j = 0; //write index
    for (uint16_t i = 0; i < len; i++, j++) {

      char c = reinterpret_cast<char *>(recv_buf)[i];

      const bool comment_start = !in_str && !in_chr && c == '#';

      if (save_cmd) recv_buf[recv_buf_sz/2 + i + 1] = (comment_start || c == '\n' || c == '\r')  ? 0 : c;

      if ((in_str || in_chr) && c == '\\') {
        if (i == len - 1) return fail(Error::PARSE_ERR);
        c = unescape(recv_buf[++i]);
      }
      else if (!in_chr && c == '"') { in_str = !in_str; c = 0; } //start/end of string
      else if (!in_str && c == '\'') { in_chr = !in_chr;  c = 0; } //start/end of char
      else if (!in_str && !in_chr && isspace(c)) c = 0; //split on whitespace including terminating '\r' or '\n'

      if (comment_start) break;
      else recv_buf[j] = c;
    }

    if (in_str || in_chr) return fail(Error::PARSE_ERR);

    while (j < recv_buf_sz/2) recv_buf[j++] = 0;
    if (!save_cmd) while (j < recv_buf_sz) recv_buf[j++] = 0;

    uint8_t * const end = recv_buf + (save_cmd ? recv_buf_sz/2 : recv_buf_sz);

    recv_ptr = recv_buf;

    while (*recv_ptr == 0) { //skip leading spaces, which are now 0s
      if (++recv_ptr == end) return end_handler_impl(); //ignore empty command
    }

    uint8_t *tmp = recv_ptr;
    arg_count = 0;
    while (++recv_ptr <= end) { if ((recv_ptr == end || !(*recv_ptr)) && *(recv_ptr - 1)) ++arg_count; }
    recv_ptr = tmp;

    const char *cmd_name = CCS(next_tok(0));
    recv_ptr = tmp; //first token returned to command handler should be the command token itself

    return dispatch([&](Cmd& cmd){ return cmd.is(cmd_name); });
  }

  //advance recv_ptr to the start of the next input token in text mode and return the current token
  //return 0 if there are no more input tokens or already has_err()
  //advance recv_ptr by binary_bytes in binary mode and return its previous value
  //unless there are not that many bytes remaining in the received packet, in which case return 0
  //if binary_bytes is 0 in binary mode then advance to the end of null terminated string
  const uint8_t *next_tok(const uint8_t binary_bytes) {

    if (has_err()) return 0;

#define FAIL { fail(Error::RECV_UNDERFLOW); return 0; }

    if (recv_ptr >= recv_buf + recv_buf_sz) FAIL;

    const uint8_t *ret = recv_ptr;

    if (binary_mode && binary_bytes > 0) {
      //recv_buf + recv_buf[0] - 1 is the checksum byte which can't itself be received
      if (recv_ptr + binary_bytes >= recv_buf + recv_buf[0]) FAIL;
      recv_ptr += binary_bytes;
    } else { //!binary_mode || !binary_bytes
      if (!binary_mode && *ret == '\n') FAIL; //start of saved command
      while (*recv_ptr) if (++recv_ptr == recv_buf + recv_buf_sz) FAIL; //skip non-null characters of current token
      while (*recv_ptr == 0) if (++recv_ptr == recv_buf + recv_buf_sz) break; //skip null separators
    }

#undef FAIL

    return ret;
  }

  //binary mode: receive a byte with value 0 or nonzero
  //text mode: receive "true", "false", "t", "f", "0", "1", "yes", "no", "y", "n" or uppercase equivalents
  ArduMon& parse_bool(const uint8_t *v, bool *dst) {
    if (has_err()) return *this;
    if (!v) return fail(Error::RECV_UNDERFLOW);
    if (binary_mode || !with_text) { *dst = *v != 0; return *this; }
    switch (*v++) {
      case '0': if (*v == 0) { *dst = false; return *this; } else return fail(Error::BAD_ARG);
      case '1': if (*v == 0) { *dst = true; return *this; } else return fail(Error::BAD_ARG);
      case 't': case 'T': if (*v == 0) { *dst = true; return *this; } else return chk_sfx(F("RUE"), true, v, dst);
      case 'f': case 'F': if (*v == 0) { *dst = false; return *this; } else return chk_sfx(F("ALSE"), false, v, dst);
      case 'y': case 'Y': if (*v == 0) { *dst = true; return *this; } else return chk_sfx(F("ES"), true, v, dst);
      case 'n': case 'N': if (*v == 0) { *dst = false; return *this; } else return chk_sfx(F("O"), false, v, dst);
      default: return fail(Error::BAD_ARG);
    }
  }

  ArduMon& chk_sfx(const FSH *sfx, const bool ret, const uint8_t *v, bool *dest) {
    const char *s_sfx = CCS(sfx);
    while (true) {
      const char s = pgm_read_byte(s_sfx++), c = *CCS(v++);
      if (s == 0) { if (c == 0) { *dest = ret; return *this; } else return fail(Error::BAD_ARG); }
        if (s != c && !((s + 32) == c)) return fail(Error::BAD_ARG); //'A' + 32 == 'a'
    }
  }

  //binary mode: copy num_bytes int from v to dest
  //text mode: parse a null terminated decimal or hexadecimal int from v to num_bytes at dest
  //if hex == true then always interpret as hex, else interpret as hex iff prefixed with 0x or 0X
  ArduMon& parse_int(const uint8_t *v, uint8_t *dest, const bool sgnd, const uint8_t num_bytes, bool hex) {

    if (has_err()) return *this;

    if (!v) return fail(Error::RECV_UNDERFLOW);

    if (binary_mode || !with_text) { copy_bytes(v, dest, num_bytes); return *this; }

    for (uint8_t i = 0; i < num_bytes; i++) dest[i] = 0;

    if (v[0] == '0' && (v[1] == 'x' || v[1] == 'X')) { hex = true; v += 2; }

    const char *s = CCS(v);
    if (hex) {
      const char *dig = s;
      for (uint8_t i = 0; *dig; i++, dig++) if (i >= 2 * num_bytes) return fail(Error::RECV_OVERFLOW);
      for (uint8_t i = 0; dig > s; i++) {
        const char d = *--dig; uint8_t p;
        if (d >= '0' && d <= '9') p = d - '0';
        else if (d >= 'A' && d <= 'F') p = 10 + d - 'A';
        else if (d >= 'a' && d <= 'f') p = 10 + d - 'a';
        else return fail(Error::BAD_ARG);
        if ((i&1) == 0) dest[i/2] = p;
        else dest[i/2] |= p << 4;
      }
      return *this;
    }

    constexpr uint8_t max_bytes = with_int64 ? 8 : 4;
    if (num_bytes > max_bytes) return fail(Error::UNSUPPORTED);

    if (num_bytes <= sizeof(long) || sizeof(long) <= max_bytes) {
      long unsigned int ret = 0;
      char *e;
      const char *expected_e = s; while (*expected_e) ++expected_e;
      errno = 0;
      if (sgnd) {
        //atol() saves a few hundred bytes vs strtol() but has no error checking
        long int tmp = strtol(s, &e, 10);
        if (e != expected_e || errno == ERANGE) return fail(Error::BAD_ARG);
        *(reinterpret_cast<long int*>(&ret)) = tmp;
        switch (num_bytes) {
          case 1: if (tmp < INT8_MIN || tmp > INT8_MAX) return fail(Error::BAD_ARG); else break;
          case 2: if (tmp < INT16_MIN || tmp > INT16_MAX) return fail(Error::BAD_ARG); else break;
          case 4: if (tmp < INT32_MIN || tmp > INT32_MAX) return fail(Error::BAD_ARG); else break;
          //otherwise rely on errno == ERANGE
        }
      } else {
        ret = strtoul(s, &e, 10); //there is no atoul()
        if (e != expected_e || errno == ERANGE) return fail(Error::BAD_ARG);
        switch (num_bytes) {
          case 1: if (ret > UINT8_MAX) return fail(Error::BAD_ARG); else break;
          case 2: if (ret > UINT16_MAX) return fail(Error::BAD_ARG); else break;
          case 4: if (ret > UINT32_MAX) return fail(Error::BAD_ARG); else break;
          //otherwise rely on errno == ERANGE
        }
      }
      copy_bytes(&ret, dest, num_bytes);
      return *this;
    }

    //sizeof(long) is typically 4 on both AVR and ESP32, so typically only get here if num_bytes == 8
    //but just in case, handle the case that sizeof(long) < 4 as well, though compiler will usually optimize it out
    if (num_bytes <= 4 && sizeof(long) < 4) return parse_dec<int32_t, uint32_t>(s, dest, sgnd, num_bytes);

    if (with_int64) return parse_dec<int64_t, uint64_t>(s, dest, sgnd, num_bytes);

    return fail(Error::UNSUPPORTED);
  }

  //parse a null terminated decimal int from s to num_bytes at dest
  template <typename big_int, typename big_uint> //supports int32_t/uint32_t, int64_t/uint64_t
  ArduMon& parse_dec(const char *s, uint8_t *dest, const bool sgnd, const uint8_t num_bytes) {

    if (binary_mode || !with_text) return fail(Error::UNSUPPORTED);

    const int8_t sign = *s == '-' ? -1 : +1;

    const bool neg = sign < 0;
    if (neg && !sgnd) return fail(Error::BAD_ARG);
    
    if (*s == '-' || *s == '+') ++s; //skip leading sign

    while (*s == '0' && *(s+1) != 0) ++s; //skip leading zeros

    uint8_t max_digits, last_chunk;
    switch (num_bytes) {
      case 1: { max_digits = 3; last_chunk = 0; break; }
      case 2: { max_digits = 5; last_chunk = 1; break; }
      case 4: { max_digits = 10; last_chunk = 2; break; }
      case 8: { max_digits = sgnd ? 19 : 20; last_chunk = 4; break; }
    }

    const char *dig = s;
    for (uint8_t i = 0; *dig; i++, dig++) if (i > max_digits) return fail(Error::BAD_ARG);

    //read digits from least to most significant in 4 digit chunks
    constexpr uint8_t num_chunks = sizeof(big_uint) > 4 ? 5 : 3;
    int16_t chunk[num_chunks];
    for (uint8_t i = 0; i < num_chunks; i++) chunk[i] = 0;
    for (uint8_t c = 0; c <= last_chunk && dig > s; c++) {
      for (uint16_t place = 1; place <= 1000; place *= 10) {
        if (--dig < s) break; //no more digits to read
        const char d = *dig;
        if (d < '0' || d > '9') return fail(Error::BAD_ARG);
        chunk[c] += (d - '0') * place;
      }
    }
    
    for (int8_t c = last_chunk; c >= 0; c--) {
      int16_t max_chunk;
      switch (num_bytes) {
        case 1: {
          //unsigned 0 to 255, signed -128 to 127
          max_chunk = neg ? 128 : sgnd ? 127 : 255;
          break;
        }
        case 2: {
          //unsigned 0 to 6:5535, signed -3:2768 to 3:2767
          if (c == 0) max_chunk = neg ? 2768 : sgnd ? 2767 : 5535;
          else max_chunk = sgnd ? 3 : 6;
          break;
        }
        case 4: {
          //unsigned 0 to 42:9496:7295, signed -21:4748:3648 to 21:4748:3647
          switch (c) {
            case 0: max_chunk = neg ? 3648 : sgnd ? 3647 : 7295; break;
            case 1: max_chunk = sgnd ? 4748 : 9496; break;
            default: max_chunk = sgnd ? 21 : 42; break;
          }
          break;
        }
        case 8: {
          //unsigned 0 to 1844:6744:0737:0955:1615
          //signed -922:3372:0368:5477:5808 to 922:3372:0368:5477:5807
          switch (c) {
            case 0: max_chunk = neg ? 5808 : sgnd ? 5807 : 1615; break;
            case 1: max_chunk = sgnd ? 5477 : 955; break;
            case 2: max_chunk = sgnd ? 368 : 737; break;
            case 3: max_chunk = sgnd ? 3372 : 6744; break;
            default: max_chunk = sgnd ? 922 : 1844; break;
          }
          break;
        }
      }
      if (chunk[c] > max_chunk) return fail(Error::BAD_ARG);
      if (chunk[c] < max_chunk) break;
    }
    
    big_uint ret = 0;
    if (sgnd) {
      big_int place = sign, *sret = reinterpret_cast<big_int*>(&ret);
      for (uint8_t c = 0; c <= last_chunk; c++, place *= 10000) *sret += chunk[c] * place;
    } else {
      big_uint place = 1;
      for (uint8_t c = 0; c <= last_chunk; c++, place *= 10000) ret += chunk[c] * place;
    }

    copy_bytes(&ret, dest, num_bytes);

    return *this;
  }

  //binary mode: copy a 4 byte float or 8 byte double from v to dest
  //text mode: parse a null terminated decimal or scientific number from v to a 4 byte float or 8 byte double at dest
  template <typename T> ArduMon& parse_float(const uint8_t *v, T *dest) {
    if (has_err()) return *this;
    if (!with_float) return fail(Error::UNSUPPORTED);
    if (!v) return fail(Error::RECV_UNDERFLOW);
    if (binary_mode || !with_text) { copy_bytes(v, dest, sizeof(T)); return *this; }
    const char *s = CCS(v);
    char *e;
    const char *expected_e = s; while (*expected_e) ++expected_e;
    errno = 0;
    double d = strtod(s, &e); //atof() is also available but is just sugar for strtod(s, 0) on AVR
    if (e != expected_e || errno == ERANGE) return fail(Error::BAD_ARG);
    *dest = static_cast<T>(d);
    return *this;
  }

  //binary mode: send one byte with value 0 or 1
  //text mode: send boolean value in indicated style
  ArduMon& write_bool(const bool v, const BoolStyle style, const bool upper_case) {
    if (has_err()) return *this;
    if (binary_mode || !with_text) return write_char(v ? 1 : 0);
    switch (style) {
      case BoolStyle::TRUE_FALSE: return write_case_str(v ? F("TRUE") : F("FALSE"), !upper_case);
      case BoolStyle::TF: return write_case_str(v ? F("T") : F("F"), !upper_case);
      case BoolStyle::ZERO_ONE: return write_char(v ? '1' : '0');
      case BoolStyle::YES_NO: return write_case_str(v ? F("YES") : F("NO"), !upper_case);
      case BoolStyle::YN: return write_case_str(v ? F("Y") : F("N"), !upper_case);
			default: return fail(Error::UNSUPPORTED);
    }
  }

  //binary mode: Error::UNSUPPORTED
  //text mode: append string to send buffer, not including terminating null
  //assume string is supplied in program memory in uppercase, convert to lowercase iff to_lower is true
  ArduMon& write_case_str(const FSH *uppercase, const bool to_lower) {
    if (binary_mode || !with_text) return fail(Error::UNSUPPORTED);
    uint8_t len = 0; while (CCS(uppercase)[len] != 0) ++len;
    uint8_t * const write_start = send_write_ptr;
    for (uint8_t i = 0; i < len + 1; i++) {
      const uint8_t c = pgm_read_byte(CCS(uppercase) + i);
      if (c) put(to_lower ? c + 32 : c); //'A' + 32 = 'a'
    }
    if (!send_read_ptr) send_read_ptr = write_start; //enable sending
    return *this;
  }

  //binary mode: append num_bytes int to send buffer
  //text mode: append num_bytes int starting at v as decimal or hexadecimal string in send buffer
  template <typename big_uint = uint32_t> //supports uint32_t, uint64_t
  ArduMon& write_int(const uint8_t *v, const bool sgnd, const uint8_t num_bytes, const uint8_t fmt) {

    if (has_err()) return *this;

    if (binary_mode || !with_text) {
      if (!check_write(num_bytes)) return fail(Error::SEND_OVERFLOW);
      for (uint8_t i = 0; i < num_bytes; i++) put(v[i]);
      return *this;
    }

    if (fmt&FMT_HEX) {
      uint8_t * const write_start = send_write_ptr;
      const uint8_t width = fmt&(~(FMT_HEX|FMT_PAD_ZERO|FMT_PAD_RIGHT));
      const char c = fmt&FMT_PAD_ZERO ? '0' : ' ';
      const uint8_t pad = width > 2*num_bytes ? width - 2*num_bytes : 0;
      if (pad && !(fmt&FMT_PAD_RIGHT)) for (uint8_t i = 0; i < pad; i++) put(c);
      for (uint8_t i = num_bytes; i > 0; i--) { put(to_hex(v[i-1] >> 4)); put(to_hex(v[i-1] & 0x0f)); }
      if (pad && (fmt&FMT_PAD_RIGHT)) for (uint8_t i = 0; i < pad; i++) put(c);
      if (!send_read_ptr) send_read_ptr = write_start; //enable sending
      return *this;
    }

    constexpr uint8_t max_bytes = with_int64 ? 8 : 4;
    if (num_bytes > max_bytes) return fail(Error::UNSUPPORTED);

    //itoa() and utoa() could be used here if num_bytes <= sizeof(int)
    //but that increases progmem usage, probably not worth it

    if (num_bytes <= sizeof(long) || sizeof(long) <= max_bytes) {
      constexpr uint8_t buf_sz = 2 + (sizeof(long) > 4 ? 20 : 10);
      char buf[buf_sz];
      if (sgnd) {
        //sign extend: if sign bit (high bit of high byte) is set initialize to -1 which is all 1s in binary, else 0
        long i = v[num_bytes-1]&0x80 ? -1 : 0;
        copy_bytes(v, &i, num_bytes);
#ifdef ARDUINO
        ltoa(i, buf, 10);
#else
        snprintf(buf, sizeof(buf), "%ld", i);
#endif
      } else {
        unsigned long i = 0;
        copy_bytes(v, &i, num_bytes);
#ifdef ARDUINO
        ultoa(i, buf, 10);
#else
        snprintf(buf, sizeof(buf), "%lu", i);
#endif
      }
      pad(buf, buf_sz, fmt);
      return write_str(buf);
    }

    //sizeof(long) is typically 4 on both AVR and ESP32, so typically only get here if num_bytes == 8
    //but just in case, handle the case that sizeof(long) < 4 as well, though compiler will usually optimize it out
    if (num_bytes <= 4 && sizeof(long) < 4) return write_dec<uint32_t>(v, sgnd, num_bytes, fmt);

    if (with_int64) return write_dec<uint64_t>(v, sgnd, num_bytes, fmt);

    return fail(Error::UNSUPPORTED);
  }

  template <typename big_uint = uint32_t> //supports uint32_t, uint64_t
  ArduMon& write_dec(const uint8_t *v, const bool sgnd, const uint8_t num_bytes, const uint8_t fmt) {

    if (binary_mode || !with_text) return fail(Error::UNSUPPORTED);

    const bool neg = sgnd && (v[num_bytes - 1] & 0x80);

    uint8_t len = 1 + (neg ? 1 : 0); //terminating null and leading sign
    switch (num_bytes) {
      case 1: len += 3; break;  //2^8-1  = 255 (3 digits)
      case 2: len += 5; break;  //2^16-1 = 65,535 (5 digits)
      case 4: len += 10; break; //2^32-1 = 4,294,967,295 (10 digits)
      case 8: len += 20; break; //2^64-1 = 18,446,744,073,709,551,615 (20 digits)
    }

    //flip negative v to positive num by inverting the bytes of v as we copy them to num and then adding one
    big_uint num = 0;
    for (uint8_t i = 0; i < num_bytes; i++) *(BP(&num) + i) = neg ? ~v[i] : v[i];
    if (neg) ++num;

    constexpr uint8_t buf_sz = 2 + (sizeof(big_uint) > 4 ? 20 : 10);
    char buf[buf_sz];
    uint8_t i = len - 1;
    buf[i] = 0;

    if (!num) buf[--i] = '0';
    else {
      while (num > 10000) { //process 4 digit chunks with one big_uint divide per chunk, plus some 16 bit math
        big_uint q = num / 10000;
        uint16_t r = num - q * 10000;
        num = q;
        for (uint8_t j = 0; j < 4; j++) {
          uint16_t qq = r / 10;
          if (i == 0) return fail(Error::UNSUPPORTED); //shouldn't happen
          buf[--i] = '0' + (r - qq * 10);
          r = qq;
        }
      }
      //if uint16_t is changed to big_uint below then the next loop would be sufficient on its own
      //but the loop above reduces the amount of big_uint math
      while (num) {
        uint16_t q = num / 10;
        if (i == 0) return fail(Error::UNSUPPORTED); //shouldn't happen
        buf[--i] = '0' + (num - q * 10);
        num = q;
      }
    }
  
    if (neg) {
      if (i == 0) return fail(Error::UNSUPPORTED); //shouldn't happen
      buf[--i] = '-';
    }

    if (i > 0) for (uint8_t j = 0; i < len; i++, j++) buf[j] = buf[i];

    pad(buf, buf_sz, fmt);

    return write_str(buf);
  }

  //binary mode: append float or double v to send buffer
  //text mode: append float or double as decimal or scientific number in send buffer
  //on AVR double is synonymous with float by default, both are 4 bytes; otherwise double may be 8 bytes
  template <typename T> //supports float and double
  ArduMon& write_float(const T v, const bool scientific, const int8_t precision, const int8_t width) {

    if (has_err()) return *this;

    if (!with_float) return fail(Error::UNSUPPORTED);

    constexpr uint8_t nb = with_double ? sizeof(T) : sizeof(float); //4 or 8
    if (nb < sizeof(T)) return fail(Error::UNSUPPORTED); //T = double, sizeof(double) > sizeof(float), !with_double

    if (binary_mode || !with_text) {
      if (!check_write(nb)) return fail(Error::SEND_OVERFLOW);
      for (uint8_t i = 0; i < nb; i++) put(*(BP(&v) + i));
      return *this;
    }

    constexpr uint8_t sig_dig = nb == 4 ? 8 : 16;
    constexpr uint8_t exp_dig = nb == 4 ? 3 : 4;
    const int8_t prec = precision < 0 || precision >= sig_dig ? sig_dig - 1 : precision;

    if (scientific) {
      constexpr uint8_t buf_sz =
        1 + 1 + 1 + (sig_dig-1) + 1 + 1 + exp_dig + 1; //sign d . d{sig_dig-1} E sign d{exp_dig} \0
      char buf[buf_sz];
      for (uint8_t i = 0; i < buf_sz; i++) buf[i] = 0;
#ifdef __AVR__
      dtostre(v, buf, prec, DTOSTR_UPPERCASE); //dtostre() is only on AVR, not ESP32
#else
      char fmt[7];
      snprintf(fmt, sizeof(fmt), "%%.%dE", prec);
      snprintf(buf, buf_sz, fmt, v);
#endif
      if (precision < 0) {
        uint8_t j = strchr(buf, 'E') - buf;
        uint8_t k = trim_backwards(buf, j - 1);
        while (buf[j]) buf[++k] = buf[j++];
        buf[++k] = 0;
      }
      return write_str(buf, false, false, -1);
    } else {
      constexpr uint8_t buf_sz = 1 + sig_dig + 1 + 1; //sign d{n} . d{sig_dig - n} \0
      char buf[buf_sz];
      for (uint8_t i = 0; i < buf_sz; i++) buf[i] = 0;
      const int8_t wid = width < 0 ? -(buf_sz - 1) : //negative width = left align
        width >= buf_sz ? buf_sz - 1 : width;
#ifdef ARDUINO
      dtostrf(v, wid, prec, buf);
#else
      char fmt[10];
      snprintf(fmt, sizeof(fmt), "%%%d.%df", wid, prec);
      snprintf(buf, buf_sz, fmt, v);
#endif
      if (precision < 0) trim_backwards(buf, buf_sz - 1);
      return write_str(buf);
    }
  }

  //binary mode: append char to send buffer
  //text mode: if cook quote and escape iff necessary, then append char to send buffer
  ArduMon& write_char(const char c, const bool cook = false) {
    if (has_err()) return *this;
    if (binary_mode || !with_text) {
      if (!check_write(1)) return fail(Error::SEND_OVERFLOW);
      put(c);
    } else {
      const char esc = cook ? escape(c, '\'') : 0;
      const bool quote = cook && (isspace(c) || esc);
      uint8_t * const write_start = send_write_ptr;
      if (quote) put('\'');
      if (esc) { put('\\'); put(esc); } else put(c);
      if (quote) put('\'');
      if (!send_read_ptr) send_read_ptr = write_start; //enable sending
    }
    return *this;
  }

  //binary mode: append null terminated string to send buffer, including terminating null
  //text mode: if cook quote and escape iff necessary, then append string to send buffer, not including terminating null
  //if len >= 0 then send len raw bytes instead of checking for null terminator in either mode
  ArduMon& write_str(const uint8_t *v, const bool progmem = false, const bool cook = false, const int16_t len = -1) {

    if (has_err() || len == 0) return *this;

    bool quote = false;
    uint16_t n = 0, n_esc = 0;
    while (len < 0 && v[n]) {
      if (!binary_mode && cook && escape(v[n], '"')) { ++n_esc; quote = true; }
      else if (!binary_mode && cook && isspace(v[n])) quote = true;
      ++n;
    }

    if (binary_mode && !check_write((len > 0) ? len : (n + n_esc + (quote ? 2 : 0) + 1))) {
      return fail(Error::SEND_OVERFLOW);
    }

    uint8_t * const write_start = send_write_ptr;

    if (quote) put('"');

    for (uint16_t i = 0; len < 0 || i < len; i++) {
      const uint8_t c = progmem ? pgm_read_byte(v + i) : v[i];
      const char esc = (c && !binary_mode && cook && len < 0) ? escape(c, '"') : 0;
      if (esc) { put('\\'); put(esc); } else if (c || binary_mode) put(c);
      if (len < 0 && !c) break; //wrote terminating null
    }

    if (quote) put('"');

    if (!binary_mode && !send_read_ptr) send_read_ptr = write_start; //enable sending

    return *this;
  }

  ArduMon& write_str(const char *v, const bool progmem = false, const bool cook = false, const int16_t len = -1) {
    return write_str(BP(v), progmem, cook, len);
  }

#ifdef ARDUINO
  ArduMon& write_str(const FSH *v, const bool cook = false, const int16_t len = -1) {
    return write_str(CCS(v), true, cook, len);
  }
#endif

  //text mode: noop
  //binary mode: check if there are at least n free bytes available in send_buf
  bool check_write(const uint16_t n) {
    if (!binary_mode || !with_binary) return true;
    if (!send_write_ptr) return false;
    if (send_write_ptr + n >= send_buf + send_buf_sz) return false; //reserve byte for checksum
    return true;
  }

  //text mode: send byte to output serial stream, blocking if necessary
  //binary mode: append a byte to send_buf and advance send_write_ptr
  //(send_read_ptr is updated in the write(...) functions which call this)
  //assumes check_write() already returned true
  void put(const uint8_t c) {
    if (binary_mode || !with_text) *send_write_ptr++ = c;
    else stream->write(c);
  }

  //see get_send_buf_used()
  uint16_t send_buf_used() {
    if (!binary_mode || !with_binary) return 0;
    return send_read_ptr ? send_buf[0] : (send_write_ptr - send_buf + 1); //+1 for checksum
  }

  //see get_recv_buf_used()
  uint16_t recv_buf_used() {
    if (!(flags&F_RECEIVING) && !(flags&F_HANDLING)) return 0;
    if (flags&F_RECEIVING) return recv_ptr - recv_buf;
    if (binary_mode || !with_text) return recv_buf[0];
    uint8_t *last_non_null = recv_buf + recv_buf_sz - 1;
    while (last_non_null >= recv_buf && *last_non_null == 0) --last_non_null;
    return last_non_null - recv_buf + 1;
  }

  //see set_binary_mode(), reset()
  ArduMon& set_binary_mode_impl(const bool binary, const bool force = false, const bool with_crlf = false) {
    if ((binary && !with_binary) || (!binary && !with_text)) return fail(Error::UNSUPPORTED);
    if (!force && binary_mode == binary) return *this;
    binary_mode = binary;
    flags &= ~(F_SPACE_PENDING | F_HANDLING | F_RECEIVING);
    recv_ptr = recv_buf;
    send_read_ptr = 0;
    arg_count = 0;
    err = Error::NONE;
    if (binary_mode || !with_text) send_write_ptr = send_buf + 1; //enable writing send buf, first byte for length
    else { send_write_ptr = send_buf; send_txt_prompt(with_crlf); }
    return *this;
  }

  //see update()
  ArduMon& update_impl() {

    if ((flags&F_RECEIVING) && recv_timeout_ms > 0 && millis() > recv_deadline) fail(Error::RECV_TIMEOUT);

    while (!has_err() && !(flags&F_HANDLING) && stream->available()) { //pump receive buffer

      if (recv_ptr - recv_buf >= recv_buf_sz) { fail(Error::RECV_OVERFLOW); break; }
      
      *recv_ptr = static_cast<uint8_t>(stream->read());
      
      if (recv_ptr == recv_buf) { //received first command byte
        flags |= F_RECEIVING;
        recv_deadline = millis() + recv_timeout_ms;
      }

      if (binary_mode || !with_text) {
        
        if (recv_ptr == recv_buf) { //received length
          if (*recv_ptr < 3) fail(Error::BAD_PACKET);
          else ++recv_ptr;
        } else if (recv_ptr - recv_buf + 1 == recv_buf[0]) { //received full packet
          flags &= ~F_RECEIVING; flags |= F_HANDLING;
          if (!handle_bin_command()) fail(Error::BAD_HANDLER).end_handler_impl();
          break; //handle at most one command per update()
        } else ++recv_ptr;

        continue;
      }

      //text mode

      if (*recv_ptr == '\r' || *recv_ptr == '\n') { //text mode end of command
        //interactive terminal programs like minicom will send '\r'
        //but if we only echo that, then the cursor will not advance to the next line
        if (flags&F_TXT_ECHO) { write_char('\r'); write_char('\n'); } //ignore echo errors
        //we also want to handle cases where automation is sending commands e.g. from a script or canned text file
        //in that situation the newline could be platform dependent, e.g. '\n' on Unix and OS X, "\r\n" on Windows
        //if we receive "\r\n" that will just incur an extra empty command
        //automation would typically not turn on F_TXT_ECHO
        //if it does, it can deal with the separate "\r\n" echo for both '\r' and '\n'
        //(Also remember that only one command can be handled at a time and that data received while handling a
        //command will fill the Arduino serial receive buffer, which is typically 64 bytes.  So e.g. at 115200 8N1 a
        //each command handler has about 5ms to complete before the next command will overflow the receive buffer if
        //a script is being piped into the serial port.)
        flags &= ~F_RECEIVING; flags |= F_HANDLING;
        if (!handle_txt_command()) fail(Error::BAD_HANDLER).end_handler_impl();
        break; //handle at most one command per update() 
      }

      if (*recv_ptr == '\b') { //text mode backspace
        if (recv_ptr > recv_buf) {
          if (flags&F_TXT_ECHO) { vt100_move_rel(1, VT100_LEFT); vt100_clear_right(); }
          --recv_ptr;
        }
        continue;
      }

      //text mode command character
      
      //catch and ignore VT100 movement codes: up <ESC>[A, down <ESC>[B, right <ESC>[C, left <ESC>[D
      //these are sent by minicom when the user hits the arrow keys on the keyboard
      //we unfortunately don't support command line editing with these
      //except for possibly 1 line of command history, if there was room in the upper half of recv_buf
      bool esc_seq_end = (*recv_ptr >= 'A' && *recv_ptr <= 'D') &&
        recv_ptr > (recv_buf+1) && *(recv_ptr-1) == '[' && *(recv_ptr-2) == 27;
      
      bool esc_seq_pending = *recv_ptr == 27 || (*recv_ptr == '[' && recv_ptr > recv_buf && *(recv_ptr-1) == 27);
      
      if ((flags&F_TXT_ECHO) && !(esc_seq_end || esc_seq_pending)) write_char(*recv_ptr);
      
      if (!esc_seq_end) ++recv_ptr; //common case
      else if (*recv_ptr == 'A' && (recv_ptr - recv_buf) < recv_buf_sz/2 && recv_buf[recv_buf_sz/2] == '\n') {
        //user hit up arrow and we have a saved previous command: switch to it
        vt100_clear_line();
        send_txt_prompt();
        recv_ptr = recv_buf;
        for (uint16_t i = recv_buf_sz/2 + 1; i < recv_buf_sz && recv_buf[i]; i++) {
          write_char(recv_buf[i]);
          *recv_ptr++ = recv_buf[i];
        }
      } else recv_ptr -= 2;  //esc_seq_end but wasn't up arrow or we didn't have saved command: ignore escape sequence

    } //pump receive buffer

    //RECV_OVERFLOW, RECV_TIMEOUT, BAD_CMD, BAD_PACKET, PARSE_ERR, UNSUPPORTED
    if (!is_handling() && has_err() && handle_err_impl()) end_handler_impl().send_txt_prompt();

    if (binary_mode) pump_send_buf(0);

    return *this;
  }

  void pump_send_buf(const millis_t wait_ms) {
    if (!binary_mode || !with_binary) return;
    const millis_t deadline = millis() + wait_ms;
    do {
      while (send_read_ptr != 0 && stream->availableForWrite()) {
        stream->write(*send_read_ptr++);
        if (send_read_ptr == send_buf + send_buf[0]) { //sent entire packet
          send_read_ptr = 0; //disable reading from send buf
          send_write_ptr = send_buf + 1; //enable writing to send buf, reserve first byte for length
        }
      }
      //reduce repetitive calls to millis() which temporarily disables interrupts
      if (send_read_ptr != 0 && wait_ms > 0) delayMicroseconds(10);
    } while (send_read_ptr != 0 && wait_ms > 0 && (wait_ms == ALWAYS_WAIT || millis() < deadline));
  }

  void pump_send_buf() { pump_send_buf(send_wait_ms); }

  ArduMon& handle_err_impl() {
    if (err != Error::NONE &&
        ((flags&F_ERROR_RUNNABLE && error_runnable && error_runnable->run(*this)) ||
         (!(flags&F_ERROR_RUNNABLE) && error_handler && error_handler(*this)))) {
      err = Error::NONE;
    }
    return *this;
  }

  //see end_handler()
  ArduMon& end_handler_impl() {

    const bool was_handling = flags&F_HANDLING; //tolerate being called when not actually handling

    //BAD_HANDLER, RECV_UNDERFLOW, BAD_ARG, SEND_OVERFLOW, UNSUPPORTED
    handle_err_impl();

    if (!binary_mode) send_CRLF();

    flags &= ~(F_SPACE_PENDING | F_HANDLING);
    recv_ptr = recv_buf;
    arg_count = 0;

    if (!was_handling) return *this;

    else if (!binary_mode || !with_binary) return send_txt_prompt();
    else return send_packet_impl();
  }

  ArduMon& send_packet_impl() {

    if (!binary_mode || !with_binary) return *this;

    const uint16_t len = send_write_ptr - send_buf;

    //check_write() already ensured that len < send_buf_sz, so the next line is redundant
    //also it's better to have send_packet_impl() not be able to generate an error
    //since it's called after handle_err_impl() in end_handler_impl()
    //if (len >= send_buf_sz) return fail(Error::SEND_OVERFLOW); //need 1 byte for checksum

    if (len > 1) { //ignore empty packet, but first byte of send_buf is reserved for length
      send_buf[0] = static_cast<uint8_t>(len + 1); //set packet length including checksum
      uint8_t sum = 0; for (uint8_t i = 0; i < len; i++) sum += send_buf[i];
      send_buf[len] = static_cast<uint8_t>(-sum); //set packet checksum
      send_write_ptr = 0; //disable writing to send buf
      send_read_ptr = send_buf; //enable reading from send buf
      pump_send_buf();
    } //else send_write_ptr must still be send_buf + 1 and send_read_ptr = 0

    return *this;
  }

  ArduMon& send_cmds_impl() {
    if (binary_mode || !with_text) return *this;
    for (uint8_t i = 0; i < n_cmds; i++) { //sending cannot error in text mode
      write_char(to_hex(cmds[i].code >> 4)).write_char(to_hex(cmds[i].code)).write_char(' ');
      if (cmds[i].name) write_str(cmds[i].name, cmds[i].flags&Cmd::F_PROGMEM);
      if (cmds[i].description) write_char(' ').write_str(cmds[i].description, cmds[i].flags&Cmd::F_PROGMEM);
      send_CRLF(true);
    }
    return *this;
  }

  char get_key_impl() {
    if (!stream->available()) return 0;
    char c = static_cast<char>(stream->read());
    if (c == 27 && stream->available() > 1 && stream->peek() == '[') {
      stream->read(); //ignore '['
      c = static_cast<char>(stream->read());
      switch (c) {
        case 'A': return UP_KEY;
        case 'B': return DOWN_KEY;
        case 'C': return RIGHT_KEY;
        case 'D': return LEFT_KEY;
        default: return UNKNOWN_KEY;
      }
    }
    return c;
  }

  //trim trailing whitespace and zeros backwards from start index; returns next un-trimmed index
  static uint8_t trim_backwards(char *s, const uint8_t start) {
    uint8_t i = start;
    while (i >= 0 && (s[i - 1] != '.') && (s[i] == 0 || s[i] == '0' || s[i] == ' ')) s[i--] = '\0';
    return i;
  }

  void pad(char *buf, const uint8_t buf_sz, const uint8_t fmt) {
    uint8_t width = fmt&(~(FMT_HEX|FMT_PAD_ZERO|FMT_PAD_RIGHT)); //shouldn't get here if FMT_HEX
    if (!width) return;
    if (width >= buf_sz) width = buf_sz - 1;
    const char c = fmt&FMT_PAD_ZERO ? '0' : ' ';
    if (fmt&FMT_PAD_RIGHT) {
      uint8_t pad = width, i = 0;
      while (i < buf_sz && pad > 0 && buf[i]) { ++i; --pad; }
      while (i < buf_sz && pad > 0) { buf[i++] = c; --pad; }
      buf[i] = 0;
    } else {
      uint8_t len = 0;
      while (len < buf_sz && buf[len]) ++len;
      if (len < width) {
        int8_t i = len - 1, j = width;
        buf[j--] = 0;
        while (j >= 0) buf[j--] = i >= 0 ? buf[i--] : c;
        if (c == '0' && buf[width - len] == '-') { buf[width - len] = '0'; buf[0] = '-'; }
      }
    }
  }

  //return escape char if c needs to be backslash escaped, else return 0
  static const char escape(const char c, const char quote) {
    if (c == quote) return quote;
    switch (c) {
      case '\\': return '\\';
      case '\a': return 'a';
      case '\b': return 'b';
      case '\f': return 'f';
      case '\n': return 'n';
      case '\r': return 'r';
      case '\t': return 't';
      case '\v': return 'v';
      case 27: return 'e'; //escape (nonstandard)
      case 127: return 'd'; //delete (nonstandard)
      default: return 0;
    }
  }

  //convert escape chare to escape code, if any
  static const char unescape(const char c) {
    switch (c) {
      case 'a': return '\a';
      case 'b': return '\b';
      case 'f': return '\f';
      case 'n': return '\n';
      case 'r': return '\r';
      case 't': return '\t';
      case 'v': return '\v';
      case 'e': return 27; //escape (nonstandard)
      case 'd': return 127; //delete (nonstandard)
      default: return c;
    }
  }

  static bool copy_bytes(const void *src, void *dest, const uint8_t num_bytes) {
    for (uint8_t i = 0; i < num_bytes; i++) {
      reinterpret_cast<uint8_t*>(dest)[i] = reinterpret_cast<const uint8_t*>(src)[i];
    }
    return true;
  }

  static const char * CCS(const void *s) { return reinterpret_cast<const char*>(s); }

  static uint8_t * BP(void *p) { return reinterpret_cast<uint8_t*>(p); }

  static const uint8_t * BP(const void *p) { return reinterpret_cast<const uint8_t*>(p); }
};

#endif
