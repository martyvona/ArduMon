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
  snprintf(out, 100, fmt, val);
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
  snprintf(out, 100, fmt, val);
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
    BAD_PACKET,     //invalid received checksum in binary mode
    BAD_CALL        //invalid argument in API call
  };

  Error get_err() { return err; }
  void clear_err() { err = Error::NONE; }

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
    err = Error::NONE;
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

    if (receiving && recv_timeout_ms > 0 && millis() > recv_deadline) { ok = false; err = Error::RECV_TIMEOUT; }

    while (ok && !handling && stream->available()) { //pump receive buffer

      if (recv_ptr - recv_buf >= recv_buf_sz) { err = Error::RECV_OVERFLOW; ok = false; }
      else {

        *recv_ptr = static_cast<uint8_t>(stream->read());

        if (recv_ptr == recv_buf) { //received first command byte
          receiving = true;
          recv_deadline = millis() + recv_timeout_ms;
        }

        if (binary_mode) {

          if (recv_ptr == recv_buf) { //received length
            if (*recv_ptr < 3) { err = Error::BAD_CMD; receiving = ok = false; }
          } else if (recv_ptr - recv_buf + 1 == recv_buf[0]) { //received full packet
            receiving = false; handling = true;
            ok = handle_bin_command();
            if (!ok && err == Error::NONE) err = Error::BAD_HANDLER;
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
          if (!ok && err == Error::NONE) err = Error::BAD_HANDLER;
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
    if (len > 254 || len >= send_buf_sz) { err = Error::SEND_OVERFLOW; return false; } //need 1 byte for checksum
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
  bool recv() { return next_token(1) != 0 || fail(Error::RECV_UNDERFLOW); }

  //receive a character
  bool recv(char *v) {
    const uint8_t *ptr = next_token(1);
    if (ptr) { *v = static_cast<char>(*ptr); return true; }
    else return fail(Error::RECV_UNDERFLOW);
  }

  //receive a string
  bool recv(const char* *v) {
    const char *ptr = next_token(0);
    if (ptr) { *v = ptr; return true; }
    else return fail(Error::RECV_UNDERFLOW);
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
  bool recv(uint64_t *v, const bool hex = false) { return parse<int64_t, uint64_t>(next_token(8), v, false, 8, hex); }
  bool recv( int64_t *v, const bool hex = false) { return parse<int64_t, uint64_t>(next_token(8), v, true,  8, hex); }

  //binary mode: receive float or double
  //text mode: receive a decimal or scientific float or double
  //on AVR double is synonymous with float by default, both are 4 bytes; otherwise double may be 8 bytes
  bool recv(float *v) { return parse(next_token(4), v); }
  bool recv(double *v) { return parse(next_token(sizeof(double)), v); }

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
  bool send_raw(const uint64_t v, const bool hex = false) { return write<uint64_t>(&v, false, 8, hex); }
  bool send_raw(const  int64_t v, const bool hex = false) { return write<uint64_t>(&v, true,  8, hex); }

  //binary mode: send float or double
  //text mode: send space separator if necessary, then send float or double as decimal or scientific
  //on AVR both double and float are 4 bytes; on other platforms double may be 8 bytes
  bool send(const float v) { return send_txt_sep() && send_raw(v); }
  bool send(const double v) { return send_txt_sep() && send_raw(v); }

  //binary mode: send float or double bytes
  //text mode: send float or double as decimal or scientific string
  //on AVR both double and float are 4 bytes; on other platforms double may be 8 bytes
  bool send_raw(const float v) { return write(v); }
  bool send_raw(const double v) { return write(v); }

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

  Error err = Error::NONE; //most recent error

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
    if (n_cmds == max_num_cmds) return fail(Error::CMD_OVERFLOW);
    for (uint8_t i = 0; i < n_cmds; i++) {
      if ((progmem && cmds[i].is_P(name)) || (!progmem && cmds[i].is(name))) return fail(Error::CMD_OVERFLOW);
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
  template <typename big_int = int32_t, //supports int32_t, int64_t
            typename big_uint = uint32_t> //supports uint32_t, uint64_t
  bool parse(const uint8_t *v, uint8_t *dest, const bool sgnd, const uint8_t num_bytes, bool hex) {

    if (num_bytes > sizeof(big_uint)) return fail(Error::BAD_CALL);

    if (!v) return fail(Error::RECV_UNDERFLOW);

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
      const char *dig = reinterpret_cast<const char*>(v);
      for (uint8_t i = 0; *dig; i++, dig++) if (i >= 2 * num_bytes) return fail(Error::RECV_OVERFLOW);
      for (uint8_t i = 0; dig > reinterpret_cast<const char*>(v); i++) {
        const char d = *--dig; uint8_t p;
        if (d >= '0' && d <= '9') p = d - '0';
        else if (d >= 'A' && d <= 'F') p = 10 + d - 'A';
        else if (d >= 'a' && d <= 'f') p = 10 + d - 'a';
        else return fail(Error::BAD_ARG);
        if ((i&1) == 0) dest[i/2] = p;
        else dest[i/2] |= p << 4;
      }
    } else {

      //strtol() and strtoul() are available but long int is only 32 bits on AVR
      //TODO use strtol()/strtoul() iff num_bytes <= sizeof(long)

      const int8_t sign = v[0] == '-' ? -1 : +1;
      v++;

      const bool neg = sign < 0;
      if (neg && !sgnd) return fail(Error::BAD_ARG);

      while (*v == '0') v++; //skip leading zeros

      uint8_t max_digits, last_chunk;
      switch (num_bytes) {
        case 1: { max_digits = 3; last_chunk = 0; break; }
        case 2: { max_digits = 5; last_chunk = 1; break; }
        case 4: { max_digits = 10; last_chunk = 2; break; }
        case 8: { max_digits = sgnd ? 19 : 20; last_chunk = 4; break; }
      }

      const char *dig = reinterpret_cast<const char*>(v);
      for (uint8_t i = 0; *dig; i++, dig++) if (i > max_digits) return fail(Error::RECV_OVERFLOW);

      //read digits from least to most significant in chunks of 4 at a time
      constexpr uint8_t num_chunks = sizeof(big_uint) > 4 ? 5 : 3;
      int16_t chunk[num_chunks];
      for (uint8_t i = 0; i < num_chunks; i++) chunk[i] = 0;
      for (uint8_t c = 0; c <= last_chunk && dig > reinterpret_cast<const char*>(v); c++) {
        for (uint16_t place = 1; place <= 1000; place *= 10) {
          if (--dig < reinterpret_cast<const char*>(v)) break; //no more digits to read
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

      uint8_t *bret = reinterpret_cast<uint8_t*>(&ret);
      for (uint8_t i = 0; i < num_bytes; i++) dest[i] = bret[i];
    }
  }

  //binary mode: copy a 4 byte float from v to dest
  //text mode: parse a null terminated decimal or scientific number from v to a 4 byte float at dest
  template <typename T> bool parse(const uint8_t *v, T *dest) {
    if (!v) return fail(Error::RECV_UNDERFLOW);
    if (binary_mode) {
      for (uint8_t i = 0; i < 4; i++) reinterpret_cast<uint8_t*>(dest)[i] = v[i];
      return true;
    }
    const char *s = reinterpret_cast<const char*>(v);
    char *e;
    double d = strtod(s, &e);
    if (s == e) return fail(Error::BAD_ARG);
    *dest = static_cast<T>(d);
    return true;
  }

  //binary mode: append num_bytes int to send buffer
  //text mode: append num_bytes int starting at v as decimal or hexadecimal string in send buffer
  template <typename big_uint = uint32_t> //supports uint32_t, uint64_t
  bool write(const uint8_t *v, const bool sgnd, const uint8_t num_bytes, const bool hex) {

    if (num_bytes > sizeof(big_uint)) return fail(Error::BAD_CALL);

    if (binary_mode) {
      if (!check_write(num_bytes)) return fail(Error::SEND_OVERFLOW);
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
    }

    //in most cases we flip negative v to positive num here
    //by inverting the bytes of v as we copy them to num and then adding one
    //(if num_bytes < sizeof(big_uint) don't need to sign extend the high bytes because the flip of 0xff is 0)
    //the one exception is if v is 0x80 00 .. 00 = -(2^(N-1)) as intN_t = 2^(N-1) as uintN_t
    big_uint num = 0;
    bool flip = neg;
    if (flip && num_bytes == sizeof(big_uint)) {
      if (v[sizeof(big_uint) - 1] == 0x80) {
        bool rest_zero = true;
        for (uint8_t i = 0; rest_zero && i < sizeof(big_uint) - 1; i++) rest_zero = rest_zero && v[i] == 0;
        if (rest_zero) flip = false;
      }
    }
    for (uint8_t i = 0; i < num_bytes; i++) *(reinterpret_cast<uint8_t*>(&num) + i) = flip ? ~v[i] : v[i];
    if (flip) ++num;

    constexpr uint8_t buff_sz = sizeof(big_uint) == 4 ? (1 + 10 + 1) : (1 + 20 + 1);
    char buf[buff_sz];
    uint8_t i = len - 1;
    buf[i] = '\0';

    if (num == 0) buf[--i] = '0';
    else if (hex) {
      for (uint8_t i = 0; i < num_bytes; i++) {
        buf[--i] = to_hex(v[i]);
        buf[--i] = to_hex(v[i] >> 4);
      }
    } else {
      while (num > 10000) {
        big_uint q = num / 10000;
        uint16_t r = num - q * 10000;
        num = q;
        for (uint8_t j = 0; j < 5; j++) {
          uint16_t qq = r / 10;
          buf[--i] = '0' + (r - qq * 10);
          r = qq;
        }
      }
      //if uint16_t is changed to big_uint then the next loop would be sufficient on its own
      //but the loop above reduces the amount of 32 or 64 bit math overall
      while (num) {
        uint16_t q = num / 10;
        buf[--i] = '0' + (num - q * 10);
        num = q;
      }
    }

    if (neg) buf[--i] = '-';

    if (!check_write(len - i)) return fail(Error::SEND_OVERFLOW);
    const uint8_t *write_start = send_write_ptr;
    while (i < len) put(buf[i++]);
    if (!send_read_ptr) send_read_ptr = write_start; //enable sending
  }

  //binary mode: append float or double v to send buffer
  //text mode: append float or double as decimal or scientific number in send buffer
  //on AVR double is synonymous with float by default, both are 4 bytes; otherwise double may be 8 bytes
  template <typename T> //supports float and double
  bool write(const T v) {
    constexpr uint8_t nb = sizeof(T); //4 or 8
    if (binary_mode) {
      if (!check_write(nb)) return fail(Error::SEND_OVERFLOW);
      for (uint8_t i = 0; i < nb; i++) put(*(reinterpret_cast<const uint8_t*>(&v) + i));
      return true;
    }
    constexpr uint8_t sig_dig = nb == 4 ? 8 : 16;
    constexpr uint8_t exp_dig = nb == 4 ? 3 : 4;
    constexpr uint8_t exp_bits = nb == 4 ? 8 : 11;
    constexpr uint16_t exp_bias = nb == 4 ? 127 : 1023;
    constexpr uint16_t exp_mask = (1 << exp_bits) - 1;
    uint16_t exp = (*(reinterpret_cast<const uint16_t*>(&v) + (nb / 2 - 1)) >> (16 - (exp_bits + 1))) & exp_mask;
    //go scientific if exp is 0 (sub-normal) or exp_mask (nan or inf) or if it's < -4 or > 20 (2^20 ~= 10^6)
    bool scientific = !(exp == 0 || exp == exp_mask) && (exp < (exp_bias - 4) || exp > (exp_bias + 20));
    if (scientific) {
      char buf[1 + 1 + 1 + (sig_dig-1) + 1 + 1 + exp_dig + 1]; //sign d . d{sig_dig-1} E sign d{exp_dig} \0
      for (uint8_t i = 0; i < sizeof(buf); i++) buf[i] = 0;
      dtostre(v, buf, sig_dig - 1, DTOSTR_UPPERCASE);
      uint8_t j = strchr(buf, 'E') - buf;
      uint8_t k = trim_trailing(buf, j - 1);
      while (buf[j]) buf[++k] = buf[j++];
      buf[++k] = 0;
      return write(buf, false, -1);
    } else {
      char buf[1 + 1 + 1 + 3 + sig_dig + 1]; //sign d{7} . d{sig_dig - 7} \0 | sign 0 . 000 d{sig_dig} \0
      for (uint8_t i = 0; i < sizeof(buf); i++) buf[i] = 0;
      dtostrf(v, -(sizeof(buf) - 1), sig_dig - 1, buf); //negative width = left align
      trim_trailing(buf, sizeof(buf) - 1);
      return write(buf, false, -1);
    }
  }

  //trim trailing whitespace and zeros backwards from start index; returns next un-trimmed index
  uint8_t trim_trailing(char *s, const uint8_t start) {
    uint8_t i = start;
    while (i >= 0 && (s[i - 1] != '.') && (s[i] == 0 || s[i] == '0' || s[i] == ' ')) s[i--] = '\0';
    return i;
  }

  //binary mode: append char to send buffer
  //text mode: if cook quote and escape iff necessary, then append char to send buffer
  bool write(const char c, const bool cook = false) {
    if (binary_mode) {
      if (!check_write(1)) return fail(Error::SEND_OVERFLOW);
      put(c);
    } else {
      const char esc = cook ? escape(c) : 0;
      const bool quote = cook && (isspace(c) || esc);
      const uint8_t *write_start = send_write_ptr;
      if (!check_write(1 + (esc ? 1 : 0) + (quote ? 2 : 0))) return fail(Error::SEND_OVERFLOW);
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
      if (n == send_buf_sz) return fail(Error::SEND_OVERFLOW);
      if (!binary_mode && cook && escape(v[n])) { ++n_esc; quote = true; }
      else if (!binary_mode && cook && isspace(v[n])) quote = true;
      ++n;
    }

    if (!check_write((len > 0) ? len : (n + n_esc + (quote ? 2 : 0)))) return fail(Error::SEND_OVERFLOW);

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
