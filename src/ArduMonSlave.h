#ifndef ARDUMON_CMD_SLAVE_H
#define ARDUMON_CMD_SLAVE_H

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
#include <stdint.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <errno.h>

//ESP32, STM32, and native do have dtostre(), use snprintf() instead
#if !defined(ARDUINO) || !defined(__AVR__)
#include <stdio.h>
#endif

#include <ArduMonBase.h>

//max_num_cmds is the maximum number of commands that can be registered
//
//recv_buf_sz is the recieve buffer size in bytes
//in text mode the receive buffer must be large enough to hold the largest commandline
//in binary mode the receive buffer must be large enough to hold the largest incoming packet
//(all packets are no larger than 256 bytes)
//recv_buf_sz can be set to 0 for an application that only sends (which would be unusual, but is possible)
//(technically an array cannot have 0 length but we handle that by substituting 1 for 0 when we create the buffer)
//
//send_buf_sz is the send buffer size in bytes
//the send buffer is not used in text mode, and send_buf_sz can be set to 0 if binary mode will not be used
//in binary mode the send buffer must be large enough to hold the largest outgoing packet
//send_buf_sz can be set to 0 for an application that only receives
template <uint8_t max_num_cmds = 8, uint16_t recv_buf_sz = 256, uint16_t send_buf_sz = 256>
class ArduMonSlave : public ArduMonBase {
public:

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
    BAD_PACKET      //invalid received checksum in binary mode
  };

  static const char *err_msg_P(const Error e) {
    switch (e) {
      case Error::NONE: return PSTR("(none)");
      case Error::CMD_OVERFLOW: return PSTR("command overflow");
      case Error::RECV_OVERFLOW: return PSTR("receive overflow");
      case Error::RECV_UNDERFLOW: return PSTR("receive underflow");
      case Error::RECV_TIMEOUT: return PSTR("receive timeout");
      case Error::SEND_OVERFLOW: return PSTR("send_overflow");
      case Error::BAD_CMD: return PSTR("bad command");
      case Error::BAD_ARG: return PSTR("bad argument");
      case Error::BAD_HANDLER: return PSTR("error handling command");
      case Error::BAD_PACKET: return PSTR("bad packet");
      default: return PSTR("(unknown error)");
    }
  }

  ArduMonSlave(Stream *s, const bool binary = false) : stream(s) {
    memset(cmds, 0, sizeof(cmds));
    set_binary_mode_impl(binary, true, false);
  }

#ifdef ARDUINO
  ArduMonSlave(const bool binary = false) : ArduMonSlave(&Serial, binary) { }
#endif

  Error get_err() { return err; }

  //once an error state is set it is sticky; any further errors will not overwrite it until clear_err() is called
  void clear_err() { err = Error::NONE; }

  //handler_t is a pointer to a function taking pointer to ArduMonSlave object and returning success (true)/fail (false)
  //if the return is false then the handler should not have called end_cmd()
  //if the return is true then the handler may or may not have called end_cmd()
  //if not, the command is considered still being handled until end_cmd() is called
  typedef bool (*handler_t)(ArduMonSlave*);

  //get the number of registered commands
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

  //add a command; handler and name are required; code is required if binary mode will be used; description is optional
  bool add_cmd(const handler_t handler, const char *name, uint8_t code = 0, const char *description = 0) {
    return add_cmd(handler, name, code, description, false);
  }

  //sugar to add a command using the next available command code
  bool add_cmd(const handler_t handler, const char *name, const char *description = 0) {
    return add_cmd(handler, name, n_cmds, description);
  }

  //sugar to add a command with null name, for binary mode use only
  bool add_cmd(const handler_t handler, uint8_t code, const char *description = 0) {
    return add_cmd(handler, 0, code, description, false);
  }

  //add a command with strings from program memory
  bool add_cmd_P(const handler_t handler, const char *name, uint8_t code = 0, const char *description = 0) {
    return add_cmd(handler, name, code, description, true);
  }

  //sugar to add a command using the next available command code
  bool add_cmd_P(const handler_t handler, const char *name, const char *description = 0) {
    return add_cmd_P(handler, name, n_cmds, description);
  }

  //does nothing if already in the requested mode
  //otherwise the command interpreter and send and receive buffers are reset
  //if the new mode is text and there is a prompt it is sent
  void set_binary_mode(const bool binary) { set_binary_mode_impl(binary); }

  //the command interpreter and send and receive buffers are reset
  //in text mode if there is a prompt it is sent
  void reset() { set_binary_mode_impl(binary_mode, true, true); }

  bool is_binary_mode() { return binary_mode; }
  bool is_txt_mode() { return !binary_mode; }

  //enable or disable received character echo in text mode
  void set_txt_echo(const bool echo) { txt_echo = echo; }

  //set prompt to NULL to disable it
  //otherwise the new prompt is sent immediately in text mode iff a handler is not currently running
  void set_txt_prompt(const char *prompt) { txt_prompt = prompt; txt_prompt_progmem = false; send_txt_prompt(); }

  //set text prompt from a program memory string
  void set_txt_prompt_P(const char *prompt) { txt_prompt = prompt; txt_prompt_progmem = true; send_txt_prompt(); }

  //set receive timeout
  //if a command starts being received but is not finished by this timeout the command interpreter will reset
  //this probably makes more sense for automation than for interactive use
  //if a command is currently being received the new timeout will not apply until the next command
  //set to 0 to disable the timeout (it's disabled by default)
  void set_recv_timeout_ms(const millis_t ms) { recv_timeout_ms = ms; }

  millis_t get_recv_timeout_ms() { return recv_timeout_ms; }

  const millis_t ALWAYS_WAIT = -1ul; //-1 in base 2 is all 1s as unsigned

  //block for up to this long in send_packet() in binary mode, default 0, use ALWAYS_WAIT to block indefinitely
  //text mode sends always block until space is available in the Arduino serial send buffer
  void set_send_wait_ms(const millis_t ms) { send_wait_ms = ms; }

  millis_t get_send_wait_ms() { return send_wait_ms; }

  //this must be called from the Arduino loop() method
  //receive available input bytes from stream
  //if the end of a command is received then dispatch and handle it
  //in binary mode this will also attempt to send any remaining response packet bytes, without blocking
  bool update() { return update_impl(); }

  //reset the command interpreter and the receive buffer
  //in text mode then send the prompt, if any
  //in binary mode send_packet()
  bool end_cmd(const bool ok = true) { return end_cmd_impl(ok); }

  //noop in text mode
  //in binary mode if the command handler has not written any bytes then noop
  //otherwise compute packet checksum and length, disable writing send_buf, enable reading it, and start sending it
  //blocks for up to send_wait_ms
  bool send_packet() { return send_packet_impl(); }

  //check if a response packet is still being sent in binary mode
  //do not write additional data to the send buffer while this is the case
  bool is_sending_packet() { return binary_mode && send_write_ptr == 0; }

  //check if a command handler is currently running
  bool is_handling() { return handling; }

  //check if the first byte of a command has been received but not yet the full command
  bool is_receiving() { return receiving; }

  //skip the next received token in text mode; skip the next received byte in binary mode
  //call this to skip over the received command token in text mode or the command code in binary mode
  bool recv() { return next_tok(1); }

  //receive a character
  bool recv(char *v) {
    const char *ptr = reinterpret_cast<const char*>(next_tok(1));
    if (ptr) { *v = *ptr; return true; }
    return false;
  }

  //receive a string
  bool recv(const char* *v) {
    const char *ptr = reinterpret_cast<const char*>(next_tok(0));
    if (ptr) { *v = ptr; return true; }
    return false;
  }

  //binary mode: receive a byte with value 0 (false) or nonzero (true)
  //text mode: receive "true", "false", "t", "f", "0", "1", "yes", "no", "y", "n" or uppercase equivalents
  bool recv(bool *v) { return parse_bool(next_tok(1), v); }

  //binary mode: receive an integer of the indicated size
  //text mode: receive a decimal or hexadecimal integer
  //if hex == true then always interpret as hex in text mode, else interpret as hex iff prefixed with 0x or 0X
#define BP(p) reinterpret_cast<uint8_t*>(p)
  bool recv( uint8_t *v, const bool hex = false) { return parse_int(next_tok(1), BP(v), false, 1, hex); }
  bool recv(  int8_t *v, const bool hex = false) { return parse_int(next_tok(1), BP(v), true,  1, hex); }
  bool recv(uint16_t *v, const bool hex = false) { return parse_int(next_tok(2), BP(v), false, 2, hex); }
  bool recv( int16_t *v, const bool hex = false) { return parse_int(next_tok(2), BP(v), true,  2, hex); }
  bool recv(uint32_t *v, const bool hex = false) { return parse_int(next_tok(4), BP(v), false, 4, hex); }
  bool recv( int32_t *v, const bool hex = false) { return parse_int(next_tok(4), BP(v), true,  4, hex); }
  bool recv(uint64_t *v, const bool hex = false) { return parse_int(next_tok(8), BP(v), false, 8, hex); }
  bool recv( int64_t *v, const bool hex = false) { return parse_int(next_tok(8), BP(v), true,  8, hex); }
#undef BP

  //binary mode: receive float or double
  //text mode: receive a decimal or scientific float or double
  //on AVR double is synonymous with float by default, both are 4 bytes; otherwise double may be 8 bytes
  bool recv(float *v) { return parse_float(next_tok(4), v); }
  bool recv(double *v) { return parse_float(next_tok(sizeof(double)), v); }

  //sends carriage return and line feed in text mode; noop in binary mode
  bool send_CRLF() {
    if (binary_mode) return true;
    space_pending = false;
    return send_raw('\r') && send_raw('\n');
  }

  //noop in binary mode
  //in text mode send one line per command: cmd_name cmd_code_hex cmd_description
  bool send_cmds() { return send_cmds_impl(); }

  //binary mode: send a single character (8 bit clean)
  //text mode: send space separator if necessary, then send character with quote and escape iff nessary
  bool send(const char v) { return send_txt_sep() && write_char(v, true); }

  //binary and text mode: send a single character (8 bit clean)
  bool send_raw(const char v) { return write_char(v); }

  //binary mode: append null terminated string to send buffer, including terminating null
  //text mode: send space sep if necessary, then send string with quote and escape iff necessary, w/o terminating null
  bool send  (const char* v)  { return send_txt_sep() && write_str(v, false, true); }
  bool send_P(const char* v)  { return send_txt_sep() && write_str(v, true, true); }

  //binary mode: append null terminated string to send buffer, including terminating null
  //text mode: append string to send buffer, not including terminating null
  //if len >= 0 then send len bytes instead of checking for null terminator in either mode
  //8 bit clean if len >= 0, otherwise mostly 8 bit clean except for value 0 which is interpreted as a terminating null
  bool send_raw  (const char* v, const int16_t len = -1)  { return write_str(v, false, false, len); }
  bool send_raw_P(const char* v, const int16_t len = -1)  { return write_str(v, true, false, len); }

  enum class BoolStyle : uint8_t { TRUE_FALSE, TF, ZERO_ONE, YES_NO, YN };

  //binary mode: send one byte with value 0 (false) or 1 (true)
  //text mode: send space separator if necessary, then send boolean value in indicated style
  bool send(const bool v, const BoolStyle style = BoolStyle::TRUE_FALSE, const bool upper_case = false) {
    return send_txt_sep() && send_raw(v, style, upper_case);
  }

  //binary mode: send one byte with value 0 (false) or 1 (true)
  //text mode: send boolean value in indicated style
  bool send_raw(const bool v, const BoolStyle style = BoolStyle::TRUE_FALSE, const bool upper_case = false) {
    return write_bool(v, style, upper_case);
  }

  //binary mode: send a little-endian integer of the indicated size
  //text mode: send space separator if necessary, then send decimal or hexadecimal integer
  bool send(const  uint8_t v, const bool hex = false) { return send_txt_sep() && send_raw(v, hex); }
  bool send(const   int8_t v, const bool hex = false) { return send_txt_sep() && send_raw(v, hex); }
  bool send(const uint16_t v, const bool hex = false) { return send_txt_sep() && send_raw(v, hex); }
  bool send(const  int16_t v, const bool hex = false) { return send_txt_sep() && send_raw(v, hex); }
  bool send(const uint32_t v, const bool hex = false) { return send_txt_sep() && send_raw(v, hex); }
  bool send(const  int32_t v, const bool hex = false) { return send_txt_sep() && send_raw(v, hex); }
  bool send(const uint64_t v, const bool hex = false) { return send_txt_sep() && send_raw(v, hex); }
  bool send(const  int64_t v, const bool hex = false) { return send_txt_sep() && send_raw(v, hex); }

  //binary mode: send an integer of the indicated size
  //text mode: send decimal or hexadecimal integer
#define BP(p) reinterpret_cast<const uint8_t*>(p)
  bool send_raw(const  uint8_t v, const bool hex = false) { return write_int(BP(&v), false, 1, hex); }
  bool send_raw(const   int8_t v, const bool hex = false) { return write_int(BP(&v), true,  1, hex); }
  bool send_raw(const uint16_t v, const bool hex = false) { return write_int(BP(&v), false, 2, hex); }
  bool send_raw(const  int16_t v, const bool hex = false) { return write_int(BP(&v), true,  2, hex); }
  bool send_raw(const uint32_t v, const bool hex = false) { return write_int(BP(&v), false, 4, hex); }
  bool send_raw(const  int32_t v, const bool hex = false) { return write_int(BP(&v), true,  4, hex); }
  bool send_raw(const uint64_t v, const bool hex = false) { return write_int(BP(&v), false, 8, hex); }
  bool send_raw(const  int64_t v, const bool hex = false) { return write_int(BP(&v), true,  8, hex); }
#undef BP

  //binary mode: send little-endian float or double bytes
  //text mode: send space separator if necessary, then send float or double as decimal or scientific
  //on AVR both double and float are 4 bytes by default; on other platforms double may be 8 bytes
  //precision is the minimum number of fraction digits (i.e. after the decimal point), right padded with 0s
  //if precision is negative then automatically use the minimum number of fraction digits
  //width only applies to non-scientific; if positive then left-pad the result with spaces to the specified minimum
  bool send(const float v, bool scientific = false, int8_t precision = -1, int8_t width = -1) {
    return send_txt_sep() && send_raw(v, scientific, precision, width);
  }
  bool send(const double v, bool scientific = false, int8_t precision = -1, int8_t width = -1) {
    return send_txt_sep() && send_raw(v, scientific, precision, width);
  }

  //binary mode: send little-endian float or double bytes
  //text mode: send float or double as decimal or scientific string
  //see further comments for send(float)
  bool send_raw(const float v, bool scientific = false, int8_t precision = -1, int8_t width = -1) {
    return write_float(v, scientific, precision, width);
  }
  bool send_raw(const double v, bool scientific = false, int8_t precision = -1, int8_t width = -1) {
    return write_float(v, scientific, precision, width);
  }

  //some useful VT100 control codes for send_raw()
  //"\x1B" is ASCII 27 which is ESC
#ifndef ARDUINO
#define PROGMEM
#endif
  const char *VT100_INIT PROGMEM = "\x1B\x63";
  const char *VT100_CLEAR PROGMEM = "\x1B[2J";
  const char *VT100_CLEAR_LINE PROGMEM = "\r\x1B[2K"; //and move cursor to beginning of line
  const char *VT100_CURSOR_VISIBLE PROGMEM = "\x1B[?25h";
  const char *VT100_CURSOR_HIDDEN PROGMEM = "\x1B[?25l";
  const char *VT100_CURSOR_HOME PROGMEM = "\x1B[H"; //move to upper left corner
  const char *VT100_CURSOR_SAVE PROGMEM = "\x1B[7"; //save position and attributes
  const char *VT100_CURSOR_RESTORE PROGMEM = "\x1B[8"; //restore position and attributes
#ifndef ARDUINO
#undef PROGMEM
#endif

  const char VT100_UP = 'A', VT100_DOWN = 'B', VT100_RIGHT = 'C', VT100_LEFT = 'D';

  //binary mode: noop
  //text mode: move VT100 cursor n places in dir
  bool vt100_move_rel(const uint16_t n, const char dir) {
    if (binary_mode) return true;
    return write_char('\x1B') && write_char('[') && send_raw(n) && write_char(dir);
  }

  //binary mode: noop
  //text mode: move VT100 cursor to (row, col)
  bool vt100_move_abs(const uint16_t row, const uint16_t col) {
    if (binary_mode) return true;
    return write_char('\x1B') && write_char('[') && send_raw(row) && write_char(';') &&
      send_raw(col) && write_char('H');
  }

private:

  Stream *stream; //underlying serial stream

  Error err = Error::NONE; //most recent error

  bool binary_mode = false; //text mode if false

  bool txt_echo = false; //echo received characters in text mode, typically for interactive terminal use

  const char *txt_prompt = 0; //prompt string in text mode, 0 if none
  bool txt_prompt_progmem = false; //prompt string is in program memory on AVR

  bool receiving = false; //received the first but not yet last byte of a command
  bool handling = false; //a command handler is currently running

  millis_t recv_deadline = 0, recv_timeout_ms = 0; //receive timeout, disabled by default

  bool space_pending = false; //a space should be sent before the next returned value in text mode

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

  struct Cmd {
    handler_t handler; const char *name; uint8_t code; const char *description; bool progmem;
    const bool is(const char *n) { return (progmem ? strcmp_P(n, name) : strcmp(n, name)) == 0; }
    const bool is_P(const char *n) { return (progmem ? ArduMonSlave::strcmp_PP(name, n) : strcmp_P(name, n)) == 0; }
  };

  Cmd cmds[max_num_cmds];
  uint8_t n_cmds = 0;

  bool fail(Error e) { if (err == Error::NONE) err = e; return false; }

  bool add_cmd(const handler_t handler, const char *name, uint8_t code, const char *description, const bool progmem) {
    if (n_cmds == max_num_cmds) return fail(Error::CMD_OVERFLOW);
    for (uint8_t i = 0; i < n_cmds; i++) {
      if (name) {
        if ((progmem && cmds[i].is_P(name)) || (!progmem && cmds[i].is(name))) return fail(Error::CMD_OVERFLOW);
      }
      if (cmds[i].code == code) return fail(Error::CMD_OVERFLOW);
    }
    cmds[n_cmds].handler = handler;
    cmds[n_cmds].name = name;
    cmds[n_cmds].code = code;
    cmds[n_cmds].description = description;
    cmds[n_cmds].progmem = progmem;
    ++n_cmds;
    return true;
  }

  //does nothing if binary_mode, if txt_prompt is null, or if currently handling
  //otherwise sends optional CRLF followed by txt_prompt and a space
  void send_txt_prompt(const bool with_crlf = false) {
    if (!binary_mode && txt_prompt != 0 && !handling) {
      if (with_crlf) { write_char('\r'); write_char('\n'); }
      if (txt_prompt_progmem) write_str_P(txt_prompt); else write_str(txt_prompt);
      write_char(' ');
    }
  }

  //noop in binary mode
  //in text mode send a space iff space_pending
  //then reset space_pending = true
  bool send_txt_sep() {
    if (binary_mode) return true;
    bool ok = space_pending ? write_char(' ') : true;
    space_pending = true;
    return ok;
  }

  //upon call, recv_ptr is the last received byte, which should be the checksum
  //if the checksum is invalid then BAD_PACKET
  //otherwise lookup command code, if not found then BAD_CMD
  //otherwise set recv_ptr = recv_buf + 1 and call command handler
  bool handle_bin_command() {
    const uint8_t len = recv_buf[0], code = recv_buf[1];
    int8_t sum = 0;
    for (uint8_t i = 0; i < len; i++) sum += static_cast<int8_t>(recv_buf[i]);
    if (sum != 0) return fail(Error::BAD_PACKET);
    recv_ptr = recv_buf + 1; //skip over length
    for (uint8_t i = 0; i < n_cmds; i++) if (cmds[i].code == code) return (cmds[i].handler)(this);
    return fail(Error::BAD_CMD);
  }

  //upon call, recv_ptr is the last received character, which will be either '\r' or '\n'
  //tokenize recv_buf, parsing quoted characters and strings with escapes, and discarding any line end comment
  //ignore empty commands
  //otherwise lookup first token as command name, if not found then BAD_CMD
  //otherwise set recv_ptr = recv_buf and call command handler
  bool handle_txt_command() {
    const uint16_t len = recv_ptr - recv_buf + 1;
    if (len == 1) return end_cmd(true); //ignore empty command, e.g. if received just '\r' or '\n'
    bool in_str = false, in_chr = false;
    uint16_t j = 0; //write index
    for (uint16_t i = 0; i < len && j < len; i++, j++) {

      char c = reinterpret_cast<char *>(recv_buf)[i];

      if (c == '\\' && (in_str || in_chr)) {
        if (i == len - 1) return fail(Error::BAD_ARG);
        c = unescape(recv_buf[++i]);
      }
      else if (!in_chr && c == '"') { in_str = !in_str; c = 0; } //start/end of string
      else if (!in_str && c == '\'') { in_chr = !in_chr;  c = 0; } //start/end of char
      else if (!in_str && !in_chr && isspace(c)) c = 0; //split on whitespace including terminating '\r' or '\n'

      if (!in_str && !in_chr && c == '#') while (j < len) recv_buf[j++] = 0; //ignore comment
      else recv_buf[j] = c;
    }
    if (in_str || in_chr) return fail(Error::BAD_ARG);
    while (j < recv_buf_sz) recv_buf[j++] = 0;
    recv_ptr = recv_buf;
    const char *cmd = reinterpret_cast<const char*>(next_tok(0));
    if (!cmd) return end_cmd(true); //ignore empty command
    recv_ptr = recv_buf;
    for (uint8_t i = 0; i < n_cmds; i++) if (cmds[i].is(cmd)) return (cmds[i].handler)(this);
    return fail(Error::BAD_CMD);
  }

  //advance recv_ptr to the start of the next input token in text mode and return the current token
  //or return 0 if there are no more input tokens
  //advance recv_ptr by binary_bytes in binary mode and return its previous value
  //unless there are not that many bytes remaining in the received packet, in which case return 0
  //if binary_bytes is 0 in binary mode then advance to the end of null terminated string
  const uint8_t *next_tok(const uint8_t binary_bytes) {
#define FAIL { fail(Error::RECV_UNDERFLOW); return 0; }
    if (recv_ptr >= recv_buf + recv_buf_sz) FAIL;
    if (!*recv_ptr) FAIL;
    const uint8_t *ret = recv_ptr;
    if (binary_mode && binary_bytes > 0) {
      //recv_buf + recv_buf[0] - 1 is the checksum byte which can't itself be received
      if (recv_ptr + binary_bytes >= recv_buf + recv_buf[0]) FAIL;
      recv_ptr += binary_bytes;
    } else {
      while (*recv_ptr) if (++recv_ptr == recv_buf + recv_buf_sz) FAIL; //skip non-null characters of current token
      while (*recv_ptr == 0) if (++recv_ptr == recv_buf + recv_buf_sz) break; //skip null separators
    }
    return ret;
#undef FAIL
  }

  //binary mode: receive a byte with value 0 or nonzero
  //text mode: receive "true", "false", "t", "f", "0", "1", "yes", "no", "y", "n" or uppercase equivalents
  bool parse_bool(const uint8_t *v, bool *dst) {
    if (!v) return fail(Error::RECV_UNDERFLOW);
    if (binary_mode) { *dst = *v != 0; return true; }
    switch (*v++) {
      case '0': if (*v == 0) { *dst = false; return true; } else return fail(Error::BAD_ARG);
      case '1': if (*v == 0) { *dst = true; return true; } else return fail(Error::BAD_ARG);
      case 't': case 'T': if (*v == 0) { *dst = true; return true; } else return chk_sfx(PSTR("RUE"), true, v, dst);
      case 'f': case 'F': if (*v == 0) { *dst = false; return true; } else return chk_sfx(PSTR("ALSE"), false, v, dst);
      case 'y': case 'Y': if (*v == 0) { *dst = true; return true; } else return chk_sfx(PSTR("ES"), true, v, dst);
      case 'n': case 'N': if (*v == 0) { *dst = false; return true; } else return chk_sfx(PSTR("O"), false, v, dst);
      default: return fail(Error::BAD_ARG);
    }
  }

  bool chk_sfx(const char *sfx, const bool ret, const uint8_t *v, bool *dest) {
    while (true) {
      const char s = pgm_read_byte(sfx++), c = *reinterpret_cast<const char*>(v++);
      if (s == 0) { if (c == 0) { *dest = ret; return true; } else return false; }
      if (s != c && !((s + 32) == c)) return false; //'A' + 32 == 'a'
    }
  }

  //binary mode: copy num_bytes int from v to dest
  //text mode: parse a null terminated decimal or hexadecimal int from v to num_bytes at dest
  //if hex == true then always interpret as hex, else interpret as hex iff prefixed with 0x or 0X
  bool parse_int(const uint8_t *v, uint8_t *dest, const bool sgnd, const uint8_t num_bytes, bool hex) {

    if (!v) return fail(Error::RECV_UNDERFLOW);

    if (binary_mode) { copy_bytes(v, dest, num_bytes); return true; }

    for (uint8_t i = 0; i < num_bytes; i++) dest[i] = 0;

    if (v[0] == '0' && (v[1] == 'x' || v[1] == 'X')) { hex = true; v += 2; }

    const char *s = reinterpret_cast<const char*>(v);
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
      return true;
    }

    if (num_bytes <= sizeof(long)) {
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
          case 1: if (tmp < -128l || tmp > 127l) return fail(Error::BAD_ARG); else break;
          case 2: if (tmp < -32768l || tmp > 32767l) return fail(Error::BAD_ARG); else break;
          case 4: if (tmp < -2147483648l || tmp > 2147483647l) return fail(Error::BAD_ARG); else break;
          //otherwise rely on errno == ERANGE
        }
      } else {
        ret = strtoul(s, &e, 10); //there is no atoul()
        if (e != expected_e || errno == ERANGE) return fail(Error::BAD_ARG);
        switch (num_bytes) {
          case 1: if (ret > 255ul) return fail(Error::BAD_ARG); else break;
          case 2: if (ret > 65535ul) return fail(Error::BAD_ARG); else break;
          case 4: if (ret > 4294967295ul) return fail(Error::BAD_ARG); else break;
          //otherwise rely on errno == ERANGE
        }
      }
      copy_bytes(&ret, dest, num_bytes);
      return true;
    }

    //sizeof(long) is typically 4 on both AVR and ESP32, so typically only get here if num_bytes == 8
    //but just in case, handle the case that sizeof(long) < 4 as well
    //(the compiler should optimize out the int32 specialization of parse_dec() unless it's really used)
    if (num_bytes <= 4) return parse_dec<int32_t, uint32_t>(s, dest, sgnd, num_bytes);
    else return parse_dec<int64_t, uint64_t>(s, dest, sgnd, num_bytes);
  }

  //parse a null terminated decimal int from s to num_bytes at dest
  template <typename big_int, typename big_uint> //supports int32_t/uint32_t, int64_t/uint64_t
  bool parse_dec(const char *s, uint8_t *dest, const bool sgnd, const uint8_t num_bytes) {

    const int8_t sign = s[0] == '-' ? -1 : +1;
    s++;

    const bool neg = sign < 0;
    if (neg && !sgnd) return fail(Error::BAD_ARG);
    
    while (*s == '0') s++; //skip leading zeros

    uint8_t max_digits, last_chunk;
    switch (num_bytes) {
      case 1: { max_digits = 3; last_chunk = 0; break; }
      case 2: { max_digits = 5; last_chunk = 1; break; }
      case 4: { max_digits = 10; last_chunk = 2; break; }
      case 8: { max_digits = sgnd ? 19 : 20; last_chunk = 4; break; }
    }

    const char *dig = s;
    for (uint8_t i = 0; *dig; i++, dig++) if (i > max_digits) return fail(Error::RECV_OVERFLOW);

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
    
    for (uint8_t c = last_chunk; c >= 0; c--) {
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

    return true;
  }

  //binary mode: copy a 4 byte float or 8 byte double from v to dest
  //text mode: parse a null terminated decimal or scientific number from v to a 4 byte float or 8 byte double at dest
  template <typename T> bool parse_float(const uint8_t *v, T *dest) {
    if (!v) return fail(Error::RECV_UNDERFLOW);
    if (binary_mode) { copy_bytes(v, dest, sizeof(T)); return true; }
    const char *s = reinterpret_cast<const char*>(v);
    char *e;
    const char *expected_e = s; while (*expected_e) ++expected_e;
    errno = 0;
    double d = strtod(s, &e); //atof() is also available but is just sugar for strtod(s, 0) on AVR
    if (e != expected_e || errno == ERANGE) return fail(Error::BAD_ARG);
    *dest = static_cast<T>(d);
    return true;
  }

  //binary mode: send one byte with value 0 or 1
  //text mode: send boolean value in indicated style
  bool write_bool(const bool v, const BoolStyle style, const bool upper_case) {
    switch (style) {
      case BoolStyle::TRUE_FALSE: return write_case_str_P(v ? PSTR("TRUE") : PSTR("FALSE"), !upper_case);
      case BoolStyle::TF: return write_case_str_P(v ? PSTR("T") : PSTR("F"), !upper_case);
      case BoolStyle::ZERO_ONE: return write_char(v ? '1' : '0');
      case BoolStyle::YES_NO: return write_case_str_P(v ? PSTR("YES") : PSTR("NO"), !upper_case);
      case BoolStyle::YN: return write_case_str_P(v ? PSTR("Y") : PSTR("N"), !upper_case);
    }
  }

  //binary mode: append null terminated string to send buffer, including terminating null
  //text mode: append string to send buffer, not including terminating null
  //in both cases assume string is supplied in program memory in uppercase, convert to lowercase iff to_lower is true
  bool write_case_str_P(const char *uppercase, const bool to_lower) {
    uint8_t len = 0; while (uppercase[len]) ++len;
    if (binary_mode && !check_write(len + 1)) return fail(Error::SEND_OVERFLOW);
    uint8_t * const write_start = send_write_ptr;
    for (uint8_t i = 0; i < len + 1; i++) {
      uint8_t c = pgm_read_byte(uppercase + i);
      if (c && to_lower) c += 32; //'A' + 32 = 'a'
      if (c || binary_mode) put(c);
    }
    if (!binary_mode && !send_read_ptr) send_read_ptr = write_start; //enable sending
    return true;
  }

  //binary mode: append num_bytes int to send buffer
  //text mode: append num_bytes int starting at v as decimal or hexadecimal string in send buffer
  template <typename big_uint = uint32_t> //supports uint32_t, uint64_t
  bool write_int(const uint8_t *v, const bool sgnd, const uint8_t num_bytes, const bool hex) {

    if (binary_mode) {
      if (!check_write(num_bytes)) return fail(Error::SEND_OVERFLOW);
      for (uint8_t i = 0; i < num_bytes; i++) put(v[i]);
      return true;
    }

    if (hex) {
      if (binary_mode && !check_write(2 * num_bytes)) return fail(Error::SEND_OVERFLOW);
      uint8_t * const write_start = send_write_ptr;
      for (uint8_t i = 0; i < num_bytes; i++) { put(to_hex(v[i])); put(to_hex(v[i] >> 4)); }
      if (!send_read_ptr) send_read_ptr = write_start; //enable sending
      return true;
    }

    //itoa() and utoa() could be used here if num_bytes <= sizeof(int)
    //but that increases progmem usage, probably not worth it

    if (num_bytes <= sizeof(long)) {
      char buf[2 + sizeof(long) > 4 ? 20 : sizeof(long) > 2 ? 10 : 5];
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
      return write_str(buf);
    }

    //sizeof(long) is typically 4 on both AVR and ESP32, so typically only get here if num_bytes == 8
    //but just in case, handle the case that sizeof(long) < 4 as well
    //(the compiler appears to optimize out the int32 specialization of parse_dec() unless it's really used)
    if (num_bytes <= 4) return write_dec<uint32_t>(v, sgnd, num_bytes);
    else return write_dec<uint64_t>(v, sgnd, num_bytes);
  }

  template <typename big_uint = uint32_t> //supports uint32_t, uint64_t
  bool write_dec(const uint8_t *v, const bool sgnd, const uint8_t num_bytes) {

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
    for (uint8_t i = 0; i < num_bytes; i++) *(reinterpret_cast<uint8_t*>(&num) + i) = neg ? ~v[i] : v[i];
    if (neg) ++num;

    char buf[sizeof(big_uint) == 4 ? (1 + 10 + 1) : (1 + 20 + 1)];
    uint8_t i = len - 1;
    buf[i] = '\0';

    while (num > 10000) { //process 4 digit chunks with one big_uint divide per chunk, plus some 16 bit math
      big_uint q = num / 10000;
      uint16_t r = num - q * 10000;
      num = q;
      for (uint8_t j = 0; j < 5; j++) {
        uint16_t qq = r / 10;
        buf[--i] = '0' + (r - qq * 10);
        r = qq;
      }
    }
    //if uint16_t is changed to big_uint below then the next loop would be sufficient on its own
    //but the loop above reduces the amount of big_uint math
    while (num) {
      uint16_t q = num / 10;
      buf[--i] = '0' + (num - q * 10);
      num = q;
    }
  
    if (neg) buf[--i] = '-';

    return write_str(buf + i, false, false, len - i);
  }

  //binary mode: append float or double v to send buffer
  //text mode: append float or double as decimal or scientific number in send buffer
  //on AVR double is synonymous with float by default, both are 4 bytes; otherwise double may be 8 bytes
  template <typename T> //supports float and double
  bool write_float(const T v, const bool scientific, const int8_t precision, const int8_t width) {
    constexpr uint8_t nb = sizeof(T); //4 or 8
    if (binary_mode) {
      if (!check_write(nb)) return fail(Error::SEND_OVERFLOW);
      for (uint8_t i = 0; i < nb; i++) put(*(reinterpret_cast<const uint8_t*>(&v) + i));
      return true;
    }
    constexpr uint8_t sig_dig = nb == 4 ? 8 : 16;
    constexpr uint8_t exp_dig = nb == 4 ? 3 : 4;
    const int8_t prec = precision < 0 ? sig_dig - 1 : precision;
    if (scientific) {
      constexpr uint8_t buf_sz =
        1 + 1 + 1 + (sig_dig-1) + 1 + 1 + exp_dig + 1; //sign d . d{sig_dig-1} E sign d{exp_dig} \0
      char buf[buf_sz];
      for (uint8_t i = 0; i < buf_sz; i++) buf[i] = 0;
#ifdef __AVR__
      dtostre(v, buf, prec, DTOSTR_UPPERCASE); //dtostre() is only on AVR, not ESP32
#else
      char fmt[6];
      snprintf(fmt, sizeof(fmt), "%%.%dE", prec);
      snprintf(buf, buf_sz, fmt, v);
#endif
      if (precision < 0) {
        uint8_t j = strchr(buf, 'E') - buf;
        uint8_t k = trim_trailing(buf, j - 1);
        while (buf[j]) buf[++k] = buf[j++];
        buf[++k] = 0;
      }
      return write_str(buf, false, false, -1);
    } else {
      constexpr uint8_t buf_sz =
        1 + 1 + 1 + 3 + sig_dig + 1; //sign d{7} . d{sig_dig - 7} \0 | sign 0 . 000 d{sig_dig} \0
      char buf[buf_sz];
      for (uint8_t i = 0; i < buf_sz; i++) buf[i] = 0;
      const int8_t wid = width < 0 ? -(buf_sz - 1) : width; //negative width = left align
#ifdef ARDUINO
      dtostrf(v, wid, prec, buf);
#else
      char fmt[9];
      snprintf(fmt, sizeof(fmt), "%%%d.%df", wid, prec);
      snprintf(buf, buf_sz, fmt, v);
#endif
      if (precision < 0) trim_trailing(buf, buf_sz - 1);
      return write_str(buf, false, false, -1);
    }
  }

  //binary mode: append char to send buffer
  //text mode: if cook quote and escape iff necessary, then append char to send buffer
  bool write_char(const char c, const bool cook = false) {
    if (binary_mode) {
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
    return true;
  }

  //binary mode: append null terminated string to send buffer, including terminating null
  //text mode: if cook quote and escape iff necessary, then append string to send buffer, not including terminating null
  //if len >= 0 then send len raw bytes instead of checking for null terminator in either mode
  bool write_str(const uint8_t *v, const bool progmem = false, const bool cook = false, const int16_t len = -1) {

    if (len == 0) return true;

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

    return true;
  }

  bool write_str_P(const uint8_t *v, const bool cook = false, const int16_t len = -1) {
    return write_str_P(v, true, cook, len);
  }

  bool write_str(const char *v, const bool progmem = false, const bool cook = false, const int16_t len = -1) {
    return write_str(reinterpret_cast<const uint8_t*>(v), progmem, cook, len);
  }

  bool write_str_P(const char *v, const bool cook = false, const int16_t len = -1) {
    return write_str(v, true, cook, len);
  }

  //text mode: return true
  //binary mode: check if there are at least n free bytes available in send_buf
  bool check_write(const uint16_t n) {
    if (!binary_mode) return true;
    if (!send_write_ptr) return false;
    if (send_write_ptr + n >= send_buf + send_buf_sz) return false; //reserve byte for checksum
    return true;
  }

  //text mode: send byte to output serial stream, blocking if necessary
  //binary mode: append a byte to send_buf and advance send_write_ptr
  //(send_read_ptr is updated in the write(...) functions which call this)
  //assumes check_write() already returned true
  void put(const uint8_t c) {
    if (binary_mode) *send_write_ptr++ = c;
    else stream->write(c);
  }

  //see get_send_buf_used()
  uint16_t send_buf_used() {
    if (!binary_mode) return 0;
    return send_read_ptr ? send_buf[0] : send_write_ptr - send_buf + 1; //+1 for checksum
  }

  //see get_recv_buf_used()
  uint16_t recv_buf_used() {
    if (!receiving && !handling) return 0;
    if (receiving) return recv_ptr - recv_buf;
    if (binary_mode) return recv_buf[0];
    uint8_t *last_non_null = recv_buf + recv_buf_sz - 1;
    while (last_non_null >= recv_buf && *last_non_null == 0) --last_non_null;
    return last_non_null - recv_buf + 1;
  }

  //see set_binary_mode(), reset()
  void set_binary_mode_impl(const bool binary, const bool force = false, const bool with_crlf = false) {
    if (!force && binary_mode == binary) return;
    binary_mode = binary;
    receiving = handling = space_pending = false;
    err = Error::NONE;
    recv_ptr = recv_buf;
    send_read_ptr = 0;
    if (binary_mode) send_write_ptr = send_buf + 1; //enable writing send buf, reserve first byte for length
    else { send_write_ptr = send_buf; send_txt_prompt(with_crlf); }
  }

  //see update()
  bool update_impl() {

    bool ok = true;

    if (receiving && recv_timeout_ms > 0 && millis() > recv_deadline) { ok = fail(Error::RECV_TIMEOUT); }

    while (ok && !handling && stream->available()) { //pump receive buffer

      if (recv_ptr - recv_buf >= recv_buf_sz) { ok = fail(Error::RECV_OVERFLOW); }
      else {

        *recv_ptr = static_cast<uint8_t>(stream->read());

        if (recv_ptr == recv_buf) { //received first command byte
          receiving = true;
          err = Error::NONE;
          recv_deadline = millis() + recv_timeout_ms;
        }

        if (binary_mode) {

          if (recv_ptr == recv_buf) { //received length
            if (*recv_ptr < 3) { receiving = ok = fail(Error::BAD_CMD); }
          } else if (recv_ptr - recv_buf + 1 == recv_buf[0]) { //received full packet
            receiving = false; handling = true;
            ok = handle_bin_command();
            if (!ok) fail(Error::BAD_HANDLER);
            break;
          } else ++recv_ptr;

        } else if (*recv_ptr == '\r' || *recv_ptr == '\n') { //text mode end of command
          //interactive terminal programs like minicom will send '\r'
          //but if we only echo that, then the cursor will not advance to the next line
          if (txt_echo) { write_char('\r'); write_char('\n'); } //ignore echo errors
          //we also want to handle cases where automation is sending commands e.g. from a script or canned text file
          //in that situation the newline could be platform dependent, e.g. '\n' on Unix and OS X, "\r\n" on Windows
          //if we receive "\r\n" that will just incur an extra empty command
          //automation would typically not turn on txt_echo
          //though if it does, it can deal with the separate "\r\n" echo for both '\r' and '\n'
          receiving = false; handling = true;
          ok = handle_txt_command();
          if (!ok) fail(Error::BAD_HANDLER);
          break;
        } else if (*recv_ptr == '\b') { //text mode backspace
          if (recv_ptr > recv_buf) {
            if (txt_echo) { vt100_move_rel(1, VT100_LEFT); write_char(' '); vt100_move_rel(1, VT100_LEFT); }
            --recv_ptr;
          }
        } else { //text mode command character
          if (txt_echo) write_char(*recv_ptr);
          ++recv_ptr;
        }
      }
    }

    if (!ok) {
      recv_ptr = recv_buf;
      receiving = handling = space_pending = false;
      if (!binary_mode) {
        if (err != Error::NONE) write_str_P(err_msg_P(err));
        send_txt_prompt(true);
      }
    }

    pump_send_buf(0);

    return ok;
  }

  void pump_send_buf(const millis_t wait_ms) {
    if (!binary_mode) return;
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

  //see end_cmd()
  bool end_cmd_impl(const bool ok) {
    if (!ok) fail(Error::BAD_CMD); //but don't return yet
    handling = space_pending = false;
    recv_ptr = recv_buf;
    if (!binary_mode) { send_txt_prompt(); return true; }
    return send_packet_impl();
  }

  bool send_packet_impl() {
    if (!binary_mode) return true;
    uint16_t len = send_write_ptr - send_buf;
    if (len > 254 || len >= send_buf_sz) return fail(Error::SEND_OVERFLOW); //need 1 byte for checksum
    if (len > 1) { //ignore empty packet, but first byte of send_buf is reserved for length
      int8_t sum = 0;
      for (uint8_t i = 0; i < len; i++) sum += static_cast<int8_t>(send_buf[i]);
      send_buf[len] = static_cast<uint8_t>(-sum); //set packet checksum
      send_buf[0] = static_cast<uint8_t>(len + 1); //set packet length including checksum
      send_write_ptr = 0; //disable writing to send buf
      send_read_ptr = send_buf; //enable reading from send buf
      pump_send_buf();
    } //else send_write_ptr must still be send_buf + 1 and send_read_ptr = 0
    return true;
  }

  bool send_cmds_impl() {
    if (binary_mode) return true;
    for (uint8_t i = 0; i < n_cmds; i++) {
      if (!write_char(to_hex(cmds[i].code >> 4))) return false;
      if (!write_char(to_hex(cmds[i].code))) return false;
      if (!write_char(' ')) return false;
      if (cmds[i].name && !write_str(cmds[i].name, cmds[i].progmem)) return false;
      if (!write_char(' ')) return false;
      if (cmds[i].description && !write_str(cmds[i].description, cmds[i].progmem)) return false;
      if (!send_CRLF()) return false;
    }
    return true;
  }

  //trim trailing whitespace and zeros backwards from start index; returns next un-trimmed index
  static uint8_t trim_trailing(char *s, const uint8_t start) {
    uint8_t i = start;
    while (i >= 0 && (s[i - 1] != '.') && (s[i] == 0 || s[i] == '0' || s[i] == ' ')) s[i--] = '\0';
    return i;
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
      case 0x1b: return 'e'; //escape (nonstandard)
      case 0x7f: return 'd'; //delete (nonstandard)
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
      case 'e': return 0x1b; //escape (nonstandard)
      case 'd': return 0x7f; //delete (nonstandard)
      default: return c;
    }
  }

  //convert the low nybble of i to a hex char 0-9A-F
  static char to_hex(uint8_t i) { return (i&0x0f) < 10 ? ('0' + (i&0x0f)) : ('A' + ((i&0x0f) - 10)); }

  //adapted from https://github.com/bxparks/AceCommon/blob/develop/src/pstrings/pstrings.cpp
  static int strcmp_PP(const char* a, const char* b) {
    if (a == b) return 0;
    if (!a) return -1;
    if (!b) return 1;
    while (true) {
      char ca = pgm_read_byte(a++), cb = pgm_read_byte(b++);
      if (ca != cb) return ca - cb;
      if (!ca) return 0;
    }
  }
};

#endif
