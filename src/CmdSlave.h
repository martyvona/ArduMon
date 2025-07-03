#ifndef CMD_SLAVE_H
#define CMD_SLAVE_H

/**
 * Yet another Arduino serial command library.
 *
 * Unlike most other serial command libraries, this one supports both text and binary mode.  Text mode can either be
 * used interactively in a serial terminal or by automation.  Binary mode is intended for automation.  Also see
 * CmdMaster.h (TODO).
 *
 * Header only, 8 bit AVR compatible, low memory footprint, no dynamic allocation.  Most APIs which accept strings have
 * a _P variant indicating the strings are in program memory on AVR (on other platforms these are equivalent to the
 * regular APIs).  The maximum number of commands is 255.
 *
 * The update() API should be "pumped" from the Arduino loop() function. Command handlers are run directly from
 * update(), so if they run long, they will block loop().  A handler may return before handling is complete, as long as
 * end_cmd() is eventually called. In text mode a single handler can return an arbitrary amount of data.  In binary mode
 * a single handler can send an arbitrary number of response packets (see send_packet()).  Such usecases could require
 * breaking the handler up so that loop() is not blocked.
 *
 * Most of the APIs return a boolean error flag: if the return is false then call get_err() to check the error.  There
 * is no built in handling of errors, e.g. an ack/nack protocol, retries, etc.  Error handling could be implemented by
 * applications if desired.
 *
 * Flow control is also up to the application.  In interactive use the operator can wait as appropriate and/or verify a
 * response before sending another command.  Automation can do similar if necessary.  Only one command is handled at a
 * time.  If a new command is sent while one is still being handled then it will start to fill the Arduino serial input
 * buffer, which is typically 64 bytes.  It will not be transferred to the command receive buffer until the currently
 * executing command finishes, since the receive buffer is used to hold the parameters of the current command.  Incoming
 * command bytes that overflow the serial receive buffer will be silently dropped (the Arduino serial interrupt
 * unfortunately does not signal overflow).  One approach to avoid this is for an application to ensure that no command
 * is longer than the serial receive buffer.  Another approach is that the application can ensure, either by
 * timing or by verifying a response, that the current command has completed before starting to send a new command.
 * There is also an optional receive timeout which will reset the command interpreter if too much time has passed
 * between receiving the first and last bytes of a command.  This is disabled by default; it probably makes more sense
 * for automation vs interactive use.
 *
 * In text mode the received command including arguments and their separators must fit in recv_buf_sz.  There is no
 * limit on the amount of data that can be returned by a single command in text mode, though sending may block the
 * handler if enough data is sent fast enough relative to send_buf_sz and the serial baudrate (see set_send_wait_ms()).
 *
 * In binary mode both commands and responses are sent in variable length packets.  The first byte of each packet gives
 * the packet length in bytes (2-255).  The max received payload size per packet is 252 bytes, as there are three
 * overhead bytes: length, command code, and checksum.  The max payload size per response packet is 253 bytes since
 * there are only 2 bytes of overhead: length and checksum.  Multiple packets can be returned from a single handler, see
 * send_packet().
 *
 * If desired an application could layer on additional protocol to send and receive arbitrary length data with some
 * overhead. There is no direct provision for transferring 8 bit clean binary data in text mode, applications could use
 * e.g. base 64 encoding if desired.
 *
 * In many applications handler code can be agnostic to binary or text mode - call the recv(...) APIs to read command
 * parameters and the send(...) APIs to write results, and finally end_cmd().  In some cases it may be desirable for a
 * handler to behave differently in text or binary mode.  For example a text mode handler intended for interactive use
 * could stream a response with VT100 control codes to update a live display on the terminal, or a stream of formatted
 * (e.g JSON, CSV, etc) records for automation.  But in binary mode it could instead send a stream of binary packets.
 *
 * Handlers can also implement their own sub-protocols, reading and optionally writing directly to the serial port
 * (typically via the Arduino serial send and receive buffers).  Command receive is disabled while a handler is running,
 * so during that time a handler can consume serial data that's not intended for the command processor.  For example, an
 * interactive text mode handler that is updating a live display on the terminal could exit when the user sends a
 * keypress.
 *
 * For each command in text mode:
 * 1. An optional prompt is sent, preceded by carriage return and line feed, and suffixed with a space.
 * 2. Characters are read, with optional echo, until a carriage return ('\r', 0x0D) or line feed ('\n', 0x0A) is
 *    received. In echo mode both a carriage return and line feed are sent when the end of command is received.  Both
 *    Unix/OS X style '\r' line endings as well as Windows style "\r\n" are supported, though the latter will incur an
 *    ignored empty command and an extra "\r\n" response if echo is enabled.
 * 3. If the backspace character ('\b', 0x08) is received and there was a previously received character in the command
 *    then (a) the previously received character is ignored and (b) VT100 control codes are sent to erase the previous
 *    character on the terminal if echo is enbled.
 * 4. If the received command is empty then it is ignored and the command interpreter resets.
 * 5. If the received command including the terminating carriage return is larger than recv_buff_sz then RECV_OVERFLOW
 *    and the command interpreter resets.
 * 6. The received command is tokenized on whitespace.  End-of-line comments starting with # are ignored.  Character
 *    tokens can be unquoted or surrounded by single quotes.  In the quoted form the backslash escape sequences below
 *    are supported.  The quoted form also allows '#' where the # will not be interpreted as the start of a comment.
 *    String tokens must be surrounded by double quotes unless they contain no whitespace, quotes, or #, in which case
 *    the quotes are optional.
 * 7. If the first token of the command is not in the command table then BAD_CMD and the command interpreter resets.
 * 8. Otherwise, the command handler is executed.  It may call the recv(...) APIs to parse the command tokens in order.
 *    Call recv() with no arguments to skip a token, including the command token itself.  If there are no more tokens
 *    then RECV_UNDERFLOW.  If the next token is not in the expected form then BAD_ARG.
 * 9. The command handler may also call the send(...) APIs at any point to stream results back to the serial port.  In
 *    text mode returned data is sent incrementally.  There is no limit to the amount of data that can be sent in
 *    response to a command, but the send(...) APIs will block if the send buffer is full. Each sent value after the
 *    first will be prefixed by a space, unless the handler calls send_CRLF() which will replace the space.  Sent
 *    characters will be single quoted and backslash escaped if they contain a quote or whitespace character. Sent
 *    strings will be double quoted and backslash escaped if they contain a quote or whitespace character.  The
 *    send_raw() APIs can be used to avoid the space prefix, quote, and escape.
 * 10. The command handler must end with a call to end_cmd(), which will reset the command interpreter.
 *
 * Backslash escape sequences in text mode:
 * * \' 0x27 single quote
 * * \" 0x22 double quote
 * * \\ 0x5c backslash
 * * \a 0x07 bell
 * * \b 0x08 backspace
 * * \f 0x0c form feed
 * * \n 0x0a line feed
 * * \r 0x0d carriage return
 * * \t 0x09 horizontal tab
 * * \v 0x0b vertical tab
 *
 * Integers are parsed and formatted in decimal or hexadecimal in text mode, and floating point numbers are parsed and
 * formatted with optional scientific notation by default.
 *
 * Each command in binary mode is a packet consisting of
 * 1. A packet length L as a single unsigned byte.  The packet length includes the length byte itself, the command code
 *    byte, and the checksum byte.  If L < 3 BAD_COMMAND and the command interpreter resets.
 * 2. A command code as a single unsigned byte.
 * 3. L-3 payload bytes
 * 4. A single byte checksum such that the 8 bit sum of the bytes of the entire packet from the length byte through the
 *    checksum byte itself is 0.
 *
 * In binary mode a command handler is called when the full length of a packet with a valid checksum (otherwise
 * BAD_PACKET) and known command code (otherwise BAD_CMD) is received.  The command handler may call the recv(...) APIs
 * to access the received data bytes in order.  The first byte returned will be the command code itself; call recv()
 * with no arguments to skip a byte.  Attempts to recv(...) beyond the end of the payload will result in
 * RECV_UNDERFLOW.  The command handler may also call the send(...) APIs at any point to append data to the send buffer.
 * Sending more than min(send_buf_sz - 2, 253) bytes results in SEND_OVERFLOW.  When send_packet() or end_cmd() is
 * called the send buffer is enabled for transfer to the serial port.  As much of it as possible is sent immediately,
 * blocking up to send_wait_ms (0 by default).  Any remaining bytes will be drained in later calls to update(). The sent
 * data will be prefixed with an unsigned length byte which includes itself, and suffixed with a checksum byte, which is
 * also included in the length.  The checksum will be computed such that the 8 bit sum of the bytes of the entire packet
 * from the first (length) byte through the checksum byte itelf is 0.
 *
 * Multibyte quantities are read and written in little endian byte order in binary mode, which matches the endianness of
 * the architectures this library is intended to target.  (Compilation will intentionally fail on a big endian target.)
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
#include <string.h> //memset()
#include <ctype.h> //isspace()
#include <stdlib.h> //strtod()

#ifdef ARDUINO

#include <Arduino.h> //millis(), delayMicroseconds()
#include <Stream.h>

#else //shims for building on non-arduino, including on pc for testing

extern "C" {
  uint32_t millis();
  void delayMicroseconds(uint16_t us);
}

class Stream {
public:
  virtual int16_t available() = 0;
  virtual int16_t read() = 0; //-1 if no data available
  virtual int16_t availableForWrite() = 0;
  virtual uint16_t write(uint8_t byte) = 0; //returns 1
};

#endif //ARDUINO

#ifdef __AVR__

#include "avr/pgmspace.h"

//on AVR stdlib.h defines dtostre() and dtostrf()

#elif defined(ARDUINO) //Arduino but not AVR, e.g. ESP32

#include "deprecated-avr-comp/avr/pgmspace.h"
#include "deprecated-avr-comp/avr/dtostrf.h"
#define NEED_DTOSTRE

#else //not AVR or Arduino, e.g. building on pc for testing

#define PROGMEM
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#define strcmp_P(a, b) strcmp((a), (b))
#define NEED_DTOSTRF
#define NEED_DTOSTRE

#endif //__AVR__

#ifdef NEED_DTOSTRF
#include <stdio.h>
extern "C" char *dtostrf(double val, signed char width, unsigned char prec, char *out) {
    char fmt[20];
    snprintf(fmt, 20, "%%%d.%df", width, prec);
    snprintf(out, 100, fmt, val); //yeah, we don't know size of out here, and sprintf() generates deprecation warnings
    return out;
  }
#endif

#ifdef NEED_DTOSTRE
#include <stdio.h>
#define DTOSTR_ALWAYS_SIGN 0x01 /* put '+' or ' ' for positives */
#define DTOSTR_PLUS_SIGN 0x02 /* put '+' rather than ' ' */
#define DTOSTR_UPPERCASE 0x04 /* put 'E' rather 'e' */
extern "C" char *dtostre(double val, char *out, unsigned char prec, unsigned char flags) {
    char fmt[20];
    char sfx = (flags & DTOSTR_UPPERCASE) ? 'E' : 'e';
    char pfx = (flags & DTOSTR_PLUS_SIGN) ? '+' : (flags & DTOSTR_ALWAYS_SIGN) ? ' ' : 0;
    if (pfx) snprintf(fmt, 20, "%%%c.%d%c", pfx, prec, sfx);
    else snprintf(fmt, 20, "%%.%d%c", prec, sfx);
    snprintf(out, 100, fmt, val); //yeah, we don't know size of out here, and sprintf() generates deprecation warnings
    return out;
  }
#endif

#if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "only little endian architectures are supported"
#endif

template <uint8_t max_num_cmds, uint16_t recv_buf_sz, uint16_t send_buf_sz> class CmdSlave {

public:

  typedef unsigned long millis_t;

  CmdSlave(Stream *s, const bool binary = false) : stream(s) { memset(cmds, 0, sizeof(cmds)); set_binary_mode(binary); }

  enum Error {
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

  Error get_err() { return err; }
  void clear_err() { err = NONE; }

  //handler_t is a pointer to a function taking a pointer to a CmdSlave object and returning success (true)/fail (false)
  //if the return is false then the handler should not have called end_cmd()
  //if the return is true then the handler may or may not have called end_cmd()
  //if not, the command is considered still being handled until end_cmd() is called
  typedef bool (*handler_t)(CmdSlave*);

  uint8_t num_cmds() { return n_cmds; }

  //add a command; handler and name are required; code is required if binary mode will be used; description is optional
  bool add_cmd(const handler_t handler, const char *name, uint8_t code = 0, const char *description = 0) {
    return add_cmd(handler, name, code, description, false);
  }

  //add a command with strings from program memory
  bool add_cmd_P(const handler_t handler, const char *name, uint8_t code = 0, const char *description = 0) {
    return add_cmd(handler, name, code, description, true);
  }

  //does nothing if already in the requested mode
  //otherwise the command interpreter and send and receive buffers are reset
  //if the new mode is text and there is a prompt it is sent
  void set_binary_mode(const bool binary) {
    if (binary_mode == binary) return;
    binary_mode = binary;
    receiving = handling = space_pending = false;
    err = NONE;
    recv_ptr = recv_buf;
    send_read_ptr = 0;
    if (binary_mode) send_write_ptr = send_buf + 1; //enable writing send buf, reserve first byte for length
    else { send_write_ptr = send_buf; send_txt_prompt(); }
  }

  bool is_binary_mode() { return binary_mode; }
  bool is_txt_mode() { return !binary_mode; }

  //enable or disable received character echo in text mode
  void set_txt_echo(const bool echo) { txt_echo = echo; }

  //set prompt to 0 to disable it
  //otherwise the new prompt is sent immediately iff a handler is not currently running
  void set_txt_prompt(const char *prompt) { txt_prompt = prompt; txt_prompt_progmem = false; send_txt_prompt(); }
  void set_txt_prompt_P(const char *prompt) { txt_prompt = prompt; txt_prompt_progmem = true; send_txt_prompt(); }

  //if a command is currently being received the new timeout will not apply until the next command
  //set to 0 to disable the timeout (it's disabled by default)
  void set_recv_timeout_ms(const millis_t ms) { recv_timeout_ms = ms; }

  //block for up to this long in send_packet() or send(...) in text mode
  void set_send_wait_ms(const millis_t ms) { send_wait_ms = ms; }

  //receive available bytes up to end of command, if any, and then handle it
  //then send available bytes, if any, while stream can accept them
  bool update() {

    bool ok = true;

    if (receiving && recv_timeout_ms > 0 && millis() > recv_deadline) { ok = false; err = RECV_TIMEOUT; }

    while (ok && !handling && stream->available()) { //pump receive buffer

      if (recv_ptr - recv_buf >= recv_buf_sz) { err = RECV_OVERFLOW; ok = false; }
      else {

        *recv_ptr = static_cast<uint8_t>(stream->read());

        if (recv_ptr == recv_buf) { //received first command byte
          receiving = true;
          recv_deadline = millis() + recv_timeout_ms;
        }

        if (binary_mode) {

          if (recv_ptr == recv_buf) { //received length
            if (*recv_ptr < 3) { err = BAD_CMD; receiving = ok = false; }
          } else if (recv_ptr - recv_buf + 1 == recv_buf[0]) { //received full packet
            receiving = false; handling = true;
            ok = handle_bin_command();
            if (!ok && err == NONE) err = BAD_HANDLER;
            break;
          } else ++recv_ptr;

        } else if (*recv_ptr == '\r' || *recv_ptr == '\n') { //text mode end of command
          //interactive terminal programs like minicom will send '\r'
          //but if we only echo that, then the cursor will not advance to the next line
          if (txt_echo) { send_raw('\r'); send_raw('\n'); } //ignore echo errors
          //we also want to handle cases where automation is sending commands e.g. from a script or canned text file
          //in that situation the newline could be platform dependent, e.g. '\n' on Unix and OS X, "\r\n" on Windows
          //if we receive "\r\n" that will just incur an extra empty command
          //automation would typically not turn on txt_echo
          //though if it does, it can deal with the separate "\r\n" echo for both '\r' and '\n'
          receiving = false; handling = true;
          ok = handle_txt_command();
          if (!ok && err == NONE) err = BAD_HANDLER;
          break;
        } else if (*recv_ptr == '\b' && recv_ptr > recv_buf) { //text mode backspace
          if (txt_echo) { vt100_move_rel(1, VT100_LEFT); send_raw(' '); vt100_move_rel(1, VT100_LEFT); }
          --recv_ptr;
        } else { //text mode command character
          if (txt_echo) send_raw(*recv_ptr);
          ++recv_ptr;
        }
      }
    }

    if (!ok) {
      recv_ptr = recv_buf;
      receiving = handling = space_pending = false;
      if (!binary_mode) send_txt_prompt();
    }

    pump_send_buf(0);

    return ok;
  }

  //reset the command interpreter and the receive buffer
  //then in text mode send the prompt, if any; in binary mode send_packet()
  bool end_cmd() {
    handling = space_pending = false;
    recv_ptr = recv_buf;
    if (!binary_mode) { send_txt_prompt(); return true; }
    return send_packet();
  }

  //noop in text mode
  //in binary mode if the command handler has not written any bytes then noop
  //otherwise compute packet checksum and length, disable writing send_buf, enable reading it, and pump_send_buf()
  //blocks for up to send_wait_ms
  bool send_packet() {
    if (!binary_mode) return true;
    uint16_t len = send_write_ptr - send_buf;
    if (len > 254 || len >= send_buf_sz) { err = SEND_OVERFLOW; return false; } //need 1 byte for checksum
    if (len > 1) { //ignore empty packet, but first byte of send_buf is reserved for length
      int8_t sum = 0;
      for (uint8_t i = 0; i < len; i++) sum += send_buf[i];
      send_buf[len] = -sum; //set packet checksum
      send_buf[0] = static_cast<uint8_t>(len + 1); //set packet length including checksum
      send_write_ptr = 0; //disable writing to send buf
      send_read_ptr = send_buf; //enable reading from send buf
    }
    pump_send_buf();
    return true;
  }

  //check if a response packet is still being sent in binary mode
  //do not write additional data to the send buffer while this is the case
  bool sending_packet() { return binary_mode && send_write_ptr = 0; }

  //skip the next received token in text mode; skip the next received byte in binary mode
  bool recv() {
    if (!next_token(1)) { err = RECV_UNDERFLOW; return false; }
    return true;
  }

  //receive a character
  bool recv(char *v) {
    const uint8_t *ptr = next_token(1);
    if (ptr) { *v = static_cast<char>(*ptr); return true; }
    else return fail(RECV_UNDERFLOW);
  }

  //receive a string
  bool recv(const char* *v) {
    const char *ptr = next_token(0);
    if (ptr) { *v = ptr; return true; }
    else return fail(RECV_UNDERFLOW);
  }

  //binary mode: receive an integer of the indicated size
  //text mode: receive a decimal or hexadecimal integer
  //if hex == true then always interpret as hex in text mode, else interpret as hex iff prefixed with 0x or 0X
  bool recv( uint8_t *v, const bool hex = false) { return parse(next_token(1), v, false, 1, hex); }
  bool recv(  int8_t *v, const bool hex = false) { return parse(next_token(1), v, true,  1, hex); }
  bool recv(uint16_t *v, const bool hex = false) { return parse(next_token(2), v, false, 2, hex); }
  bool recv( int16_t *v, const bool hex = false) { return parse(next_token(2), v, true,  2, hex); }
  bool recv(uint32_t *v, const bool hex = false) { return parse(next_token(4), v, false, 4, hex); }
  bool recv( int32_t *v, const bool hex = false) { return parse(next_token(4), v, true,  4, hex); }
  bool recv(uint64_t *v, const bool hex = false) { return parse(next_token(8), v, false, 8, hex); }
  bool recv( int64_t *v, const bool hex = false) { return parse(next_token(8), v, true,  8, hex); }

  //binary mode: receive a 4 byte float
  //text mode: receive a decimal or scientific float
  //(on AVR double is synonymous with float by default, both are 4 bytes)
  bool recv(float *v) { return parse(next_token(4), v); }

  //sends carriage return and line feed in text mode; noop in binary mode
  bool send_CRLF() {
    if (binary_mode) return true;
    space_pending = false;
    return send_raw('\r') && send_raw('\n');
  }

  //noop in binary mode
  //in text mode send one line per command: cmd_name cmd_code_hex cmd_description
  bool send_cmds() {
    if (binary_mode) return true;
    for (uint8_t i = 0; i < n_cmds; i++) {
      if (!write(cmds[i].name, cmds[i].progmem)) return false;
      if (!write(' ')) return false;
      if (!write(to_hex(cmds[i].code >> 4))) return false;
      if (!write(to_hex(cmds[i].code))) return false;
      if (!write(' ')) return false;
      if (cmds[i].description && !write(cmds[i].description, cmds[i].progmem)) return false;
      if (!send_CRLF()) return false;
    }
    return true;
  }

  //binary mode: send a single character (8 bit clean)
  //text mode: send space separator if necessary, then send character with quote and escape iff nessary
  bool send(const char v) { return send_txt_sep() && write(v, true); }

  //binary and text mode: send a single character (8 bit clean)
  bool send_raw(const char v) { return write(v); }

  //binary mode: append null terminated string to send buffer, including terminating null
  //text mode: send space sep if necessary, then send string with quote and escape iff necessary, w/o terminating null
  bool send  (const char* v)  { return send_txt_sep() && write(v, false, true); }
  bool send_P(const char* v)  { return send_txt_sep() && write(v, true, true); }

  //binary mode: append null terminated string to send buffer, including terminating null
  //text mode: append string to send buffer, not including terminating null
  //if len >= 0 then send len bytes instead of checking for null terminator in either mode
  bool send_raw  (const char* v, const int16_t len = -1)  { return write(v, false, false, len); }
  bool send_raw_P(const char* v, const int16_t len = -1)  { return write(v, true, false, len); }

  //binary mode: send an integer of the indicated size
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
  bool send_raw(const  uint8_t v, const bool hex = false) { return write(&v, false, 1, hex); }
  bool send_raw(const   int8_t v, const bool hex = false) { return write(&v, true,  1, hex); }
  bool send_raw(const uint16_t v, const bool hex = false) { return write(&v, false, 2, hex); }
  bool send_raw(const  int16_t v, const bool hex = false) { return write(&v, true,  2, hex); }
  bool send_raw(const uint32_t v, const bool hex = false) { return write(&v, false, 4, hex); }
  bool send_raw(const  int32_t v, const bool hex = false) { return write(&v, true,  4, hex); }
  bool send_raw(const uint64_t v, const bool hex = false) { return write(&v, false, 8, hex); }
  bool send_raw(const  int64_t v, const bool hex = false) { return write(&v, true,  8, hex); }

  //binary mode: send 4 byte float
  //text mode: send space separator if necessary, then send float as decimal or scientific
  bool send(const float v) { return send_txt_sep() && send_raw(v); }

  //binary mode: send 4 byte float
  //text mode: send float as decimal or scientific
  bool send_raw(const float v) { return write(v); }

  //"\x1B" is ASCII 27 which is ESC
  const char *VT100_INIT PROGMEM = "\x1B\x63";
  const char *VT100_CLEAR PROGMEM = "\x1B[2J";
  const char *VT100_CURSOR_VISIBLE PROGMEM = "\x1B[?25h";
  const char *VT100_CURSOR_HIDDEN PROGMEM = "\x1B[?25l";
  const char *VT100_CURSOR_HOME PROGMEM = "\x1B[H"; //move to upper left corner
  const char *VT100_CURSOR_SAVE PROGMEM = "\x1B[7"; //save position and attributes
  const char *VT100_CURSOR_RESTORE PROGMEM = "\x1B[8"; //restore position and attributes

  const char VT100_UP = 'A', VT100_DOWN = 'B', VT100_RIGHT = 'C', VT100_LEFT = 'D';

  //binary mode: noop
  //text mode: move VT100 cursor n places in dir
  bool vt100_move_rel(const uint16_t n, const char dir) {
    if (binary_mode) return true;
    return send_raw('\x1B') && send_raw('[') && send_raw(n) && send_raw(dir);
  }

  //binary mode: noop
  //text mode: move VT100 cursor to (row, col)
  bool vt100_move_abs(const uint16_t row, const uint16_t col) {
    if (binary_mode) return true;
    return send_raw('\x1B') && send_raw('[') && send_raw(row) && send_raw(';') && send_raw(col) && send_raw('H');
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

private:

  Stream *stream; //underlying serial stream

  Error err = NONE; //most recent error

  bool binary_mode = false; //text mode if false

  bool txt_echo = false; //echo received characters in text mode, typically for interactive terminal use

  const char *txt_prompt = 0; //prompt string in text mode, 0 if none
  bool txt_prompt_progmem = false; //prompt string is in program memory on AVR

  bool receiving = false; //received the first but not yet last byte of a command
  bool handling = false; //a command handler is currently running

  millis_t recv_deadline = 0, recv_timeout_ms = 0; //receive timeout, disabled by default

  bool space_pending = false; //a space should be sent before the next returned value in text mode

  uint8_t recv_buf[recv_buf_sz], send_buf[send_buf_sz];

  //next available position in recv_buf while receiving a command
  //last received character when beginning to handle a command
  //start of next token while handling command
  uint8_t *recv_ptr = recv_buf;

  //send_read_ptr is the next unsent byte in the send buffer; sending is disabled iff send_read_ptr is 0
  //send_write_ptr is the next free spot in the send buffer; writing to send buffer is disabled if send_write_ptr is 0
  //in txt mode send buf is circular; SEND_OVERFLOW iff send when send_write_ptr == send_read_ptr
  //in binary mode SEND_OVERFLOW iff send when send_write_ptr == send_buf + send_buf_sz - 1 (reserved for checksum)
  uint8_t *send_read_ptr = 0, *send_write_ptr = send_buf;

  //block for up to this long in send_packet() in binary mode or send(...) in text mode
  millis_t send_wait_ms = 0;

  struct Cmd {

    const handler_t handler;
    const char *name;
    const uint8_t code;
    const char *description;
    const bool progmem;

    const bool is(const char *n) { return (progmem ? strcmp_P(n, name) : strcmp(n, name)) == 0; }
    const bool is_P(const char *n) { return (progmem ? CmdSlave::strcmp_PP(name, n) : strcmp_P(name, n)) == 0; }
    const bool is(const uint8_t c) { return c == code; }
  };

  Cmd cmds[max_num_cmds];
  uint8_t n_cmds = 0;

  bool fail(Error e) { err = e; return false; }

  bool add_cmd(const handler_t handler, const char *name, uint8_t code, const char *description, const bool progmem) {
    if (n_cmds == max_num_cmds) return fail(CMD_OVERFLOW);
    for (uint8_t i = 0; i < n_cmds; i++) {
      if ((progmem && cmds[i].is_P(name)) || (!progmem && cmds[i].is(name))) return fail(CMD_OVERFLOW);
    }
    cmds[n_cmds++] = { handler, name, code, description, progmem };
    return true;
  }

  void pump_send_buf(const millis_t wait_ms) {
    const millis_t deadline = millis() + wait_ms;
    do {
      while (send_read_ptr > 0 && stream->availableForWrite()) {
        stream->write(*send_read_ptr++);
        if (binary_mode) {
          if (send_read_ptr == send_buf + send_buf[0]) { //sent entire packet
            send_read_ptr = 0; //disable reading from send buf
            send_write_ptr = send_buf + 1; //enable writing to send buf, reserve first byte for length
          }
        } else {
          if (send_read_ptr == send_buf + send_buf_sz) send_read_ptr = send_buf; //wrap read ptr in circular buffer
          if (send_read_ptr == send_write_ptr) { //circular send buf empty
            send_read_ptr = 0; //disable reading from send buf
            send_write_ptr = send_buf; //needed by check_write
          }
        }
      }
      //reduce repetitive calls to millis() which temporarily disables interrupts
      if (send_read_ptr > 0 && wait_ms > 0) delayMicroseconds(10);
    } while (send_read_ptr > 0 && wait_ms > 0 && millis() < deadline);
  }

  void pump_send_buf() { pump_send_buf(send_wait_ms); }

  //does nothing if binary_mode, if txt_prompt is null, or if currently handling
  //otherwise sends CRLF followed by txt_prompt and a space
  void send_txt_prompt() {
    if (!binary_mode && txt_prompt != 0 && !handling) {
      send_raw('\r'); send_raw('\n');
      if (txt_prompt_progmem) send_raw_P(txt_prompt); else send_raw(txt_prompt);
      send_raw(' ');
    }
  }

  //noop in binary mode
  //in text mode send a space iff space_pending
  //then reset space_pending = true
  bool send_txt_sep() {
    if (binary_mode) return true;
    bool ok = space_pending ? send_raw(' ') : true;
    space_pending = true;
    return ok;
  }

  //upon call, recv_ptr is the last received byte, which should be the checksum
  //if the checksum is invalid then BAD_PACKET
  //otherwise lookup command code, if not found then BAD_CMD
  //otherwise set recv_ptr = recv_buf + 1 and call command handler
  bool handle_bin_command() {
    //TODO
    return false;
  }

  //upon call, recv_ptr is the last received character
  //tokenize recv_buf, parsing quoted characters and strings with escapes, and discarding any line end comment
  //lookup first token as command name, if not found then BAD_CMD
  //otherwise set recv_ptr = recv_buf and call command handler
  bool handle_txt_command() {
    //TODO
    return false;
  }

  //advance recv_ptr to the start of the next input token in text mode and return it
  //or return 0 if there are no more input tokens
  //advance recv_ptr by binary_bytes in binary mode and return its previous value
  //unless there are not that many bytes remaining, in which case return 0
  //if binary_bytes is 0 in binary mode then advance to the end of null terminated string
  const uint8_t *next_token(const uint8_t binary_bytes) {
    if (binary_mode) {
      //recv_buf + recv_buf[0] - 1 is the checksum byte which can't itself be received
      if (recv_ptr + binary_bytes >= recv_buf + recv_buf[0]) return 0;
      const uint8_t *ret = recv_ptr;
      recv_ptr += binary_bytes;
      return ret;
    } else {
      if (recv_ptr >= recv_buf + recv_buf_sz) return 0;
      while (*recv_ptr) if (++recv_ptr == recv_buf + recv_buf_sz) return 0; //skip non-null characters of current token
      while (!*recv_ptr) if (++recv_ptr == recv_buf + recv_buf_sz) return 0; //skip null separators
      return recv_ptr;
    }
  }

  //binary mode: copy num_bytes int from v to dest
  //text mode: parse a null terminated decimal or hexadecimal int from v to num_bytes at dest
  //if hex == true then always interpret as hex, else interpret as hex iff prefixed with 0x or 0X
  bool parse(const uint8_t *v, uint8_t *dest, const bool sgnd, const uint8_t num_bytes, bool hex) {

    if (!v) return fail(RECV_UNDERFLOW);

    if (binary_mode) {
      for (uint8_t i = 0; i < num_bytes; i++) dest[i] = v[i];
      return true;
    }

    for (uint8_t i = 0; i < num_bytes; i++) dest[i] = 0;

    if (v[0] == '0' && (v[1] == 'x' || v[1] == 'X')) {
      hex = true;
      v += 2;
    }

    if (hex) {
      const uint8_t *str = v;
      for (uint8_t i = 0; *str; i++, str++) if (i >= 2 * num_bytes) return fail(RECV_OVERFLOW);
      for (uint8_t i = 0; str >= v; i++) {
        const char c = *--str; uint8_t p;
        if (c >= '0' && c <= '9') p = c - '0';
        else if (c >= 'A' && c <= 'F') p = 10 + c - 'A';
        else if (c >= 'a' && c <= 'f') p = 10 + c - 'a';
        else return fail(BAD_ARG);
        if ((i&1) == 0) dest[i/2] = p;
        else dest[i/2] |= p << 4;
      }
    } else if (sgnd) {
      //TODO strtol() but 64 bit
      return false;
    } else {
      //TODO strtoul() but 64 bit
      return false;
    }
  }

  //binary mode: copy a 4 byte float from v to dest
  //text mode: parse a null terminated decimal or scientific number from v to a 4 byte float at dest
  bool parse(const uint8_t *v, float *dest) {
    if (!v) return fail(RECV_UNDERFLOW);
    if (binary_mode) {
      for (uint8_t i = 0; i < 4; i++) reinterpret_cast<uint8_t*>(dest)[i] = v[i];
      return true;
    }
    const char *s = reinterpret_cast<const char *>(v);
    char *e;
    double d = strtod(s, &e);
    if (s == e) return fail(BAD_ARG);
    *dest = static_cast<float>(d);
    return true;
  }

  //binary mode: append num_bytes int to send buffer
  //text mode: append num_bytes int as decimal or hexadecimal string in send buffer
  bool write(const uint8_t *v, const bool sgnd, const uint8_t num_bytes, const bool hex) {

    if (binary_mode) {
      if (!check_write(num_bytes)) return fail(SEND_OVERFLOW);
      for (uint8_t i = 0; i < num_bytes; i++) put(v[i]);
      return true;
    }

    bool neg = sgnd && (v[num_bytes - 1] & 0x80);

    uint8_t len = 1 + (neg ? 1 : 0); //terminating null and leading sign
    switch (num_bytes) {
      case 1: len += 3; break;  //2^8-1  = 255 (3 digits)
      case 2: len += 5; break;  //2^16-1 = 65,535 (5 digits)
      case 4: len += 10; break; //2^32-1 = 4,294,967,295 (10 digits)
      case 8: len += 20; break; //2^64-1 = 18,446,744,073,709,551,615 (20 digits)
      default: return fail(BAD_ARG);
    }

    //in most cases we flip negative v to positive num here
    //by inverting the bytes of v as we copy them to num and then adding one
    //(if num_bytes < 8 we don't need to explicitly sign extend the high bytes because the flip of 0xff is 0)
    //the one exception is if v is 0x80 00 00 00 00 00 00 00 = -(2^63) as int64_t = 2^63 as uint64_t
    uint64_t num = 0;
    bool flip = neg && !(num_bytes == 8 && v[7] == 0x80 && !v[6] && !v[5] && !v[4] && !v[3] && !v[2] && !v[1] && !v[0]);
    for (uint8_t i = 0; i < num_bytes; i++) *(reinterpret_cast<uint8_t*>(&num) + i) = flip ? ~v[i] : v[i];
    if (flip) ++num;

    char buf[22];
    uint8_t i = len - 1;
    buf[i] = '\0';

    if (num == 0) buf[--i] = '0';
    else if (hex) {
      for (uint8_t i = 0; i < num_bytes; i++) {
        buf[--i] = to_hex(v[i]);
        buf[--i] = to_hex(v[i] >> 4);
      }
    } else {

      //works but uses a lot of 64 bit math
      //while (num) {
      //  uint64_t q = num / 10;
      //  buf[--i] = '0' + (num - 10 * q);
      //  num = q;
      //}

      uint16_t n16;
      while (num > 10000) {
        uint64_t q = num / 10000;
        uint16_t r = num - q * 10000;
        num = q;
        for (uint8_t j = 0; j < 5; j++) {
          n16 = r / 10;
          buf[--i] = '0' + (r - 10 * n16);
          r = n16;
        }
      }

      n16 = num;
      while (n16) {
        uint16_t q = n16 / 10;
        buf[--i] = '0' + (n16 - 10 * q);
        n16 = q;
      }
    }

    if (neg) buf[--i] = '-';

    if (!check_write(len - i)) return fail(SEND_OVERFLOW);
    const uint8_t *write_start = send_write_ptr;
    while (i < len) put(buf[i++]);
    if (!send_read_ptr) send_read_ptr = write_start; //enable sending
  }

  //binary mode: append 4 byte float v to send buffer
  //text mode: append 4 byte float as decimal or scientific number in send buffer
  bool write(const float v) {
    if (binary_mode) {
      if (!check_write(4)) return fail(SEND_OVERFLOW);
      for (uint8_t i = 0; i < 4; i++) put(*(reinterpret_cast<const uint8_t*>(&v) + i));
      return true;
    }
    uint8_t exp = (*(reinterpret_cast<const uint16_t*>(&v) + 1) >> 7) & 0xff;
    //exp is "biased" so that the actual binary exponent is exp - 127, and exp = 0 and exp = 255 are special cases
    //go scientific if exp is 0 (sub-normal) or 255 (nan or inf) or if it's less than -4 or more than 20 (2^20 ~= 10^6)
    bool scientific = !(exp == 0 || exp == 255) && (exp < (127 - 4) || exp > (127 + 20));
    if (scientific) {
      char buf[1 + 1 + 1 + 6 + 1 + 1 + 2 + 1]; //sign + d + . + dddddd + E + sign + dd + \0
      for (size_t i = 0; i < sizeof(buf); i++) buf[i] = '\0';
      dtostre(v, buf, 6, DTOSTR_UPPERCASE);
      return write(buf, false, -1);
    } else {
      char buf[1 + 8 + 1 + 9 + 1]; //sign + dddddddd + . + ddddddddd + \0
      for (size_t i = 0; i < sizeof(buf); i++) buf[i] = '\0';
      dtostrf(v, 0, 9, buf);
      for (size_t i = sizeof(buf) - 1; i > 0 && (buf[i - 1] != '.') && (buf[i] == '\0' || buf[i] == '0'); i--) {
        buf[i] = '\0'; //trim trailing zeros
      }
      return write(buf , false, -1);
    }
  }

  //binary mode: append char to send buffer
  //text mode: if cook quote and escape iff necessary, then append char to send buffer
  bool write(const char c, const bool cook = false) {
    if (binary_mode) {
      if (!check_write(1)) return fail(SEND_OVERFLOW);
      put(c);
    } else {
      const char esc = cook ? escape(c) : 0;
      const bool quote = cook && (isspace(c) || esc);
      const uint8_t *write_start = send_write_ptr;
      if (!check_write(1 + (esc ? 1 : 0) + (quote ? 2 : 0))) return fail(SEND_OVERFLOW);
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
  bool write(const uint8_t *v, const bool progmem, const bool cook = false, const int16_t len = -1) {

    if (len == 0) return true;

    bool quote = false;
    uint16_t n = 0, n_esc = 0;
    while (len < 0 && v[n]) {
      if (n == send_buf_sz) return fail(SEND_OVERFLOW);
      if (!binary_mode && cook && escape(v[n])) { ++n_esc; quote = true; }
      else if (!binary_mode && cook && isspace(v[n])) quote = true;
      ++n;
    }

    if (!check_write((len > 0) ? len : (n + n_esc + (quote ? 2 : 0)))) return fail(SEND_OVERFLOW);

    const uint8_t *write_start = send_write_ptr;

    if (quote) put('"');

    for (uint16_t i = 0; len < 0 || i < len; i++) {
      const uint8_t c = progmem ? pgm_read_byte(v + i) : v[i];
      const char esc = (!binary_mode && cook && len < 0) ? escape(c) : 0;
      if (esc) { put('\\'); put(esc); } else put(c);
      if (len < 0 && c == '\0') break; //wrote terminating null
    }

    if (quote) put('"');

    if (!binary_mode && !send_read_ptr) send_read_ptr = write_start; //enable sending

    return true;
  }

  //return escape char if c needs to be backslash escaped, else return 0
  const char escape(const char c) {
    switch (c) {
      case '\'': return '\'';
      case '"': return '"';
      case '\\': return '\\';
      case '\a': return 'a';
      case '\b': return 'b';
      case '\f': return 'f';
      case '\n': return 'n';
      case '\r': return 'r';
      case '\t': return 't';
      case '\v': return 'v';
      default: return 0;
    }
  }

  //check if there are at least n free bytes available in send_buf
  bool check_write(const uint16_t n) {
    if (!send_write_ptr) return false;
    if (binary_mode) { if (send_write_ptr + n >= send_buf + send_buf_sz) return false; } //reserve byte for checksum
    else if (send_read_ptr > 0) { if (send_write_ptr + n > send_read_ptr) return false; }
    else if (send_write_ptr + n > send_buf + send_buf_sz) return false;
    else return true;
  }

  //append a byte to send_buf
  //advances send_write_ptr; send_read_ptr is updated in the write(...) functions which call this
  //assumes check_write() already returned true
  void put(const uint8_t c) {
    *send_write_ptr++ = c;
    if (!binary_mode && send_write_ptr == send_buf + send_buf_sz) send_write_ptr = send_buf; //send_buf circular in txt
  }
};

#endif
