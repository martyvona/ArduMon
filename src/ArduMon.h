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
class ArduMonStream { //shim for Arduino Stream in native host build
public:
  virtual ~ArduMonStream() {}
  virtual int16_t available() = 0;
  virtual int16_t read() = 0; //-1 if no data available
  virtual int16_t peek() = 0; //-1 if no data available
  virtual int16_t availableForWrite() = 0;
  virtual uint16_t write(uint8_t byte) = 0; //returns 1
};
#endif

//ArduMon: yet another Arduino serial command library
//
//see https://github.com/martyvona/ArduMon/blob/main/README.md
//
//max_num_cmds is the maximum number of commands that can be registered
//
//recv_buf_sz is the recieve buffer size in bytes
//in text mode the receive buffer must be large enough to hold the largest command line
//command history in text mode requires a receive buffer large enough to hold both the current and the previous cmd
//in binary mode the receive buffer must be large enough to hold the largest incoming packet (limited to 256 bytes)
//recv_buf_sz can be set to 0 for an application that only sends
//(an array cannot have 0 length but we handle that internally by substituting 1 for 0 when we create the buffer)
//
//send_buf_sz is the send buffer size in bytes
//send buffer is not used in text mode; send_buf_sz can be set to 0 if binary mode will not be used
//send_buf_sz can also be set to 0 for an application that only receives
//in binary mode the send buffer must be large enough to hold the largest outgoing packet (limited to 256 bytes)
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

  //handler_t is a pointer to a function taking a reference to ArduMon object and returning void
  //if the return is false then the handler failed, endHandler() will be automatically called if necessary
  //if the return is true then the handler succeded so far; it may or may not have called endHandler()
  //the command is still being handled until endHandler() is called
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

  static const FSH *errMsg(const Error e) {
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
    setBinaryModeImpl(binary, true, false);
    setUniversalHandler(0);
    setFallbackHandler(0);
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
  Stream *getStream() { return stream; }

  //get the number of registered commands
  //this will also be the binary code of the next command that will be added by addCmd() without an explicit code
  uint8_t getNumCmds() { return n_cmds; }

  //get the maximum number of commands that can be registered
  uint8_t getMaxNumCmds() { return max_num_cmds; }

  //get the send buffer size in bytes
  uint16_t getSendBufSize() { return send_buf_sz; }

  //binary mode: get packet size if currently sending a packet, else get number of bytes used so far in send buffer
  //text mode: return 0
  uint16_t getSendBufUsed() { return sendBufUsed(); }

  //sugar for getSendBufSize() - getSendBufUsed()
  uint16_t getSendBufFree() { return send_buf_sz - sendBufUsed(); }

  //get the receive buffer size in bytes
  uint16_t getRecvBufSize() { return recv_buf_sz; }

  //if not currently receiving or handling a command then return 0
  //if curently receiving a command then get number of bytes received so far
  //if handling in binary mode return received packet size
  //if handling in text mode return length of received command string
  uint16_t getRecvBufUsed() { return recvBufUsed(); }

  //sugar for getRecvBufSize() - getRecvBufUsed()
  uint16_t getRecvBufFree() { return recv_buf_sz - recvBufUsed(); }

  bool hasErr() const { return err != Error::NONE; }

  //bool conversion operator; returns !hasErr()
  operator bool() const { return !hasErr(); }

  Error getErr() { return err; }

  //return and clear any current error
  Error clearErr() { Error was = err; err = Error::NONE; return was; }

  //if hasErr() and there is an error handler, run it
  ArduMon& handleErr() { return handleErrImpl(); }

  //returns a default error handler that
  //* sends the error message back to the user in text mode
  //* sends the error message to the default Serial port on Arduino in binary mode (unless that Serial port is also the
  //  stream used for binary communication, in which case the error message is swallowed)
  //* prints the error message to the console otherwise (native binary mode)
  handler_t getDefaultErrorHandler() { return getDefaultErrorHandlerImpl(); }

  //install the default error handler returned by getDefaultErrorHandler()
  ArduMon& setDefaultErrorHandler() { return setErrorHandler(getDefaultErrorHandlerImpl()); }

  //set an error handler that will be called automatically during endHandler() if hasErr()
  //if the error handler returns true then clearErr() will be automatically called
  ArduMon&  setErrorHandler(const handler_t h)  { return setHandler (error_handler,  h, F_ERROR_RUNNABLE); }
  handler_t getErrorHandler()                   { return getHandler (error_handler,     F_ERROR_RUNNABLE); }
  ArduMon&  setErrorRunnable(Runnable* const r) { return setRunnable(error_runnable, r, F_ERROR_RUNNABLE); }
  Runnable* getErrorRunnable()                  { return getRunnable(error_runnable,    F_ERROR_RUNNABLE); }

  //set a universal command handler that will override any other handlers added with addCmd()
  //this can be useful e.g. in binary mode to handle received packets where byte two is not necessarily a command code
  //set handler to 0 to remove any existing universal handler (and thus re-enable handlers added with addCmd())
  ArduMon&  setUniversalHandler(const handler_t h)  { return setHandler (universal_handler,  h, F_UNIV_RUNNABLE); }
  handler_t getUniversalHandler()                   { return getHandler (universal_handler,     F_UNIV_RUNNABLE); }
  ArduMon&  setUniversalRunnable(Runnable* const r) { return setRunnable(universal_runnable, r, F_UNIV_RUNNABLE); }
  Runnable* getUniversalRunnable()                  { return getRunnable(universal_runnable,    F_UNIV_RUNNABLE); }

  //set a fallback command handler that will handle received commands that did not have a command name
  //(command code in binary mode) matching any handler added with addCmd()
  //set handler to 0 to remove any existing fallback handler
  ArduMon&  setFallbackHandler(const handler_t h)  { return setHandler (fallback_handler,  h, F_FALLBACK_RUNNABLE); }
  handler_t getFallbackHandler()                   { return getHandler (fallback_handler,     F_FALLBACK_RUNNABLE); }
  ArduMon&  setFallbackRunnable(Runnable* const r) { return setRunnable(fallback_runnable, r, F_FALLBACK_RUNNABLE); }
  Runnable* getFallbackRunnable()                  { return getRunnable(fallback_runnable,    F_FALLBACK_RUNNABLE); }

  //add a command: name may be null, but if not, it must be unique relative to already added commands
  //code must be unique relative to already added commands
  //CMD_OVERFLOW if command with the given name (if any) or code already exists, or if max_num_cmds already added
  ArduMon& addCmd(const handler_t handler, const char *name, const uint8_t code, const char *description = 0) {
    return addCmdImpl(handler, name, code, description, false);
  }

  //add a command using the next available command code
  ArduMon& addCmd(const handler_t handler, const char *name, const char *description = 0) {
    return addCmdImpl(handler, name, n_cmds, description, false);
  }

  //add a command with null name, for binary mode use only
  ArduMon& addCmd(const handler_t handler, const uint8_t code, const char *description = 0) {
    return addCmdImpl(handler, 0, code, description, false);
  }

  //add a command with a Runnable instead of a handler_t
  ArduMon& addCmd(Runnable* const runnable, const char *name, const uint8_t code, const char *description = 0) {
    return addCmdImpl(runnable, name, code, description, false);
  }

  //add a command using the next available command code
  ArduMon& addCmd(Runnable* const runnable, const char *name, const char *description = 0) {
    return addCmdImpl(runnable, name, n_cmds, description, false);
  }

  //add a command with null name, for binary mode use only
  ArduMon& addCmd(Runnable* const runnable, const uint8_t code, const char *description = 0) {
    return addCmdImpl(runnable, 0, code, description, false);
  }

  //remove command registered with given code
  ArduMon& removeCmd(const uint8_t code) { return removeCmdImpl(code); }

  //remove command registered with given name
  ArduMon& removeCmd(const char *name) { return removeCmdImpl(name); }

  //remove command registered with given handler
  ArduMon& removeCmd(const handler_t handler) { return removeCmdImpl(handler); }

  //remove command registered with given runnable
  ArduMon& removeCmd(Runnable* const runnable) { return removeCmdImpl(runnable); }

#ifdef ARDUINO
  //add a command with strings from program memory
  ArduMon& addCmd(const handler_t handler, const FSH *name, const uint8_t code, const FSH *description = 0) {
    return addCmdImpl(handler, CCS(name), code, CCS(description), true);
  }

  //add a command using the next available command code with strings from program memory
  ArduMon& addCmd(const handler_t handler, const FSH *name, const FSH *description = 0) {
    return addCmdImpl(handler, CCS(name), n_cmds, CCS(description), true);
  }

  //add a command with strings from program memory
  ArduMon& addCmd(Runnable* const runnable, const FSH *name, const uint8_t code, const FSH *description = 0) {
    return addCmdImpl(runnable, CCS(name), code, CCS(description), true);
  }

  //add a command using the next available command code with strings from program memory
  ArduMon& addCmd(Runnable* const runnable, const FSH *name, const FSH *description = 0) {
    return addCmdImpl(runnable, CCS(name), n_cmds, CCS(description), true);
  }

  //remove command registered with given name and return true if it was found, false if it was not found
  ArduMon& removeCmd(const FSH *name) { return removeCmdImpl(name); }
#endif

  //get the command code for a command name; returns -1 if not found
  int16_t getCmdCode(const char *name) { return getCmdCodeImpl<char>(name); }
#ifdef ARDUINO
  int16_t getCmdCode(const FSH *name) { return getCmdCodeImpl<FSH>(name); }
#endif

  //get the commad name for a command code; returns null if not found
  //the returned pointer will be in program memory on AVR if and only if the command was originally registered that way 
  const char * getCmdName(const uint8_t code) {
    for (uint8_t i = 0; i < n_cmds; i++) if (cmds[i].code == code) return cmds[i].name;
    return 0;
  }

  //noop in binary mode
  //in text mode send one line per command: cmd_name cmd_code_hex cmd_description
  ArduMon& sendCmds() { return sendCmdsImpl(); }

  //does nothing if already in the requested mode: binary mode if binary=true, else text mode
  //otherwise the command interpreter and send and receive buffers are reset
  //if the new mode is text and there is a prompt it is sent
  //Error::UNSUPPORTED if binary but !with_binary or !binary but !with_text
  ArduMon& setBinaryMode(const bool binary) { return setBinaryModeImpl(binary); }

  //reset the command interpreter and send and receive buffers
  //in text mode send prompt, if any
  ArduMon& reset() { return setBinaryModeImpl(binary_mode, true, true); }

  bool isBinaryMode() { return binary_mode; }
  bool isTextMode() { return !binary_mode; }

  //enable or disable received character echo in text mode
  ArduMon& setTextEcho(const bool echo) {
    if (echo) flags |= F_TXT_ECHO; else flags &= ~F_TXT_ECHO;
    return *this;
  }

  //set prompt to NULL to disable it
  //otherwise the new prompt is sent immediately in text mode iff a handler is not currently running
  ArduMon& setTextPrompt(const char *prompt) {
    txt_prompt = prompt;
    flags &= ~F_TXT_PROMPT_PROGMEM;
    return sendTextPrompt();
  }

#ifdef ARDUINO
  //set text prompt from a program memory string
  ArduMon& setTextPrompt(const FSH *prompt) {
    txt_prompt = CCS(prompt);
    flags |= F_TXT_PROMPT_PROGMEM;
    return sendTextPrompt();
  }
#endif

  static const millis_t ALWAYS_WAIT = -1; //-1 in base 2 is all 1s as unsigned

  //set receive timeout
  //if a command starts being received but is not finished by this timeout the command interpreter will reset
  //this probably makes more sense for automation than for interactive use
  //if a command is currently being received the new timeout will not apply until the next command
  //set to 0 or ALWAYS_WAIT to disable the timeout (it's disabled by default)
  ArduMon& setRecvTimeoutMS(const millis_t ms) { recv_timeout_ms = ms == ALWAYS_WAIT ? 0 : ms; return *this; }
  millis_t getRecvTimeoutMS() { return recv_timeout_ms; }

  //block for up to this long in send_packet() in binary mode, default 0, use ALWAYS_WAIT to block indefinitely
  //text mode sends always block until space is available in the Arduino serial send buffer
  ArduMon& setSendWaitMS(const millis_t ms) { send_wait_ms = ms; return *this; }
  millis_t getSendWaitMS() { return send_wait_ms; }

  //this must be called from the Arduino loop() method
  //receive available input bytes from serial stream
  //if the end of a command is received then dispatch and handle it
  //in binary mode then try to send remaining response packet bytes without blocking
  ArduMon& update() { return updateImpl(); }

  //reset the command interpreter and the receive buffer
  //if hasErr() and there is an error handler (or Runnable) then run it
  //in text mode then send the prompt, if any
  //in binary mode send_packet()
  ArduMon& endHandler() { return endHandlerImpl(); }

  //noop in text mode
  //in binary mode if the send buffer is empty then noop
  //otherwise compute packet checksum and length, disable writing send_buf, enable reading it, and start sending it
  //blocks for up to send_wait_ms
  ArduMon& sendPacket() { return sendPacketImpl(); }

  //check if a packet is still being sent in binary mode
  //do not write additional data to the send buffer while this is the case
  bool isSendingPacket() { return binary_mode && send_write_ptr == 0; }

  //check if a command handler is currently running
  bool isHandling() { return flags&F_HANDLING; }

  //check if the first byte of a command has been received but not yet the full command
  bool isReceiving() { return flags&F_RECEIVING; }

  //text mode: return number of command arguments, including the command name itself
  //binary mode: return number of command packet payload bytes including the command code, if any
  //in either case the return is only valid while handling a command
  uint8_t argc() { return arg_count; }

  //skip the next n tokens in text mode; skip the next n bytes in binary mode
  ArduMon& skip(const uint8_t n = 1) { nextTok(n); return *this; }

  //receive a character (recvChar() instead of recv(char &v) to disambiguate from recv(int8_t &v))
  ArduMon& recvChar(char &v) {
    const char *ptr = CCS(nextTok(1));
    if (!ptr || (!binary_mode && *(ptr + 1) != 0)) return fail(Error::BAD_ARG);
    v = *ptr;
    return *this;
  }

  //receive a string
  ArduMon& recv(const char* &v) {
    const char *ptr = CCS(nextTok(0));
    if (!ptr) return fail(Error::BAD_ARG);
    v = ptr;
    return *this;
  }

  //binary mode: receive a byte with value 0 (false) or nonzero (true)
  //text mode: receive "true", "false", "t", "f", "0", "1", "yes", "no", "y", "n" or uppercase equivalents
  ArduMon& recv(bool &v) { return parseBool(nextTok(1), &v); }

  //binary mode: receive an integer of the indicated size
  //text mode: receive a decimal or hexadecimal integer
  //if hex == true then always interpret as hex in text mode, else interpret as hex iff prefixed with 0x or 0X
  //Error::UNSUPPORTED if recv([u]int64_t) but !with_int64
  ArduMon& recv( uint8_t &v, const bool hex = false) { return parseInt(nextTok(1), BP(&v), false, 1, hex); }
  ArduMon& recv(  int8_t &v, const bool hex = false) { return parseInt(nextTok(1), BP(&v), true,  1, hex); }
  ArduMon& recv(uint16_t &v, const bool hex = false) { return parseInt(nextTok(2), BP(&v), false, 2, hex); }
  ArduMon& recv( int16_t &v, const bool hex = false) { return parseInt(nextTok(2), BP(&v), true,  2, hex); }
  ArduMon& recv(uint32_t &v, const bool hex = false) { return parseInt(nextTok(4), BP(&v), false, 4, hex); }
  ArduMon& recv( int32_t &v, const bool hex = false) { return parseInt(nextTok(4), BP(&v), true,  4, hex); }
  ArduMon& recv(uint64_t &v, const bool hex = false) { return parseInt(nextTok(8), BP(&v), false, 8, hex); }
  ArduMon& recv( int64_t &v, const bool hex = false) { return parseInt(nextTok(8), BP(&v), true,  8, hex); }

  //binary mode: receive float or double
  //text mode: receive a decimal or scientific float or double
  //on AVR double is synonymous with float by default, both are 4 bytes; otherwise double may be 8 bytes
  //Error::UNSUPPORTED if sizeof(float) != sizeof(double) and recv(double) but !with_double
  ArduMon& recv(float &v) { return parseFloat(nextTok(4), &v); }
  ArduMon& recv(double &v) { return parseFloat(nextTok(sizeof(double)), &v); }

  //noop in binary mode
  //in text mode send carriage return and line feed instead of pending space separator
  //if force=true then always send CRLF; otherwise only send if a space separator is pending
  ArduMon& sendCRLF(bool force = false) {
    if (binary_mode || !with_text) return *this;
    if (force || (flags&F_SPACE_PENDING)) writeChar('\r').writeChar('\n');
    flags &= ~F_SPACE_PENDING;
    return *this;
  }

  //binary mode: send a single character (8 bit clean)
  //text mode: send space separator if necessary, then send character with quote and escape iff nessary
  //(sendChar() instead of send(char) to disambiguate from send(int8_t v))
  ArduMon& sendChar(const char v) { return sendTextSep().writeChar(v, true); }

  //binary and text mode: send a single character (8 bit clean)
  ArduMon& sendRaw(const char v) { return writeChar(v); }

  //binary mode: append null terminated string to send buffer, including terminating null
  //text mode: send space sep if necessary, then send string with quote and escape iff necessary, w/o terminating null
  ArduMon& send(const char* v) { return sendTextSep().writeStr(v, false, true); }
#ifdef ARDUINO
  ArduMon& send(const FSH* v) { return sendTextSep().writeStr(CCS(v), true, true); }
#endif

  //binary mode: append null terminated string to send buffer, including terminating null
  //text mode: append string to send buffer, not including terminating null
  //if len >= 0 then send len bytes instead of checking for null terminator in either mode
  //8 bit clean if len >= 0, otherwise mostly 8 bit clean except for value 0 which is interpreted as a terminating null
  ArduMon& sendRaw(const char* v, const int16_t len = -1) { return writeStr(v, false, false, len); }
#ifdef ARDUINO
  ArduMon& sendRaw(const FSH* v, const int16_t len = -1) { return writeStr(CCS(v), true, false, len); }
#endif

  enum class BoolStyle : uint8_t { TRUE_FALSE, TF, ZERO_ONE, YES_NO, YN };

  //binary mode: send one byte with value 0 (false) or 1 (true)
  //text mode: send space separator if necessary, then send boolean value in indicated style
  ArduMon& send(const bool v, const BoolStyle style = BoolStyle::TRUE_FALSE, const bool upper_case = false) {
    return sendTextSep().sendRaw(v, style, upper_case);
  }

  //binary mode: send one byte with value 0 (false) or 1 (true)
  //text mode: send boolean value in indicated style
  ArduMon& sendRaw(const bool v, const BoolStyle style = BoolStyle::TRUE_FALSE, const bool upper_case = false) {
    return writeBool(v, style, upper_case);
  }

  static const uint8_t FMT_HEX = 0x80, FMT_PAD_ZERO = 0x40, FMT_PAD_RIGHT = 0x20;

  //binary mode: send a little-endian integer of the indicated size
  //text mode: send space separator if necessary, then send decimal or hexadecimal integer
  //fmt ignored in binary; in text mode it is a bitmask of FMT_* flags with low 5 bits specifying minimum field width
  //with FMT_HEX width can be at most 31
  //otherwise width will be clamped to 21 for [u]int64_t and 11 for the other int types
  ArduMon& send(const  uint8_t v, const uint8_t fmt = 0) { return sendTextSep().sendRaw(v, fmt); }
  ArduMon& send(const   int8_t v, const uint8_t fmt = 0) { return sendTextSep().sendRaw(v, fmt); }
  ArduMon& send(const uint16_t v, const uint8_t fmt = 0) { return sendTextSep().sendRaw(v, fmt); }
  ArduMon& send(const  int16_t v, const uint8_t fmt = 0) { return sendTextSep().sendRaw(v, fmt); }
  ArduMon& send(const uint32_t v, const uint8_t fmt = 0) { return sendTextSep().sendRaw(v, fmt); }
  ArduMon& send(const  int32_t v, const uint8_t fmt = 0) { return sendTextSep().sendRaw(v, fmt); }
  ArduMon& send(const uint64_t v, const uint8_t fmt = 0) { return sendTextSep().sendRaw(v, fmt); }
  ArduMon& send(const  int64_t v, const uint8_t fmt = 0) { return sendTextSep().sendRaw(v, fmt); }

  //binary mode: send an integer of the indicated size
  //text mode: send decimal or hexadecimal integer
  //fmt ignored in binary; in text mode it is a bitmask of FMT_* flags with low 5 bits specifying minimum field width
  //field width will be clamped to 21 for [u]int64_t and 11 for the other int types
  ArduMon& sendRaw(const  uint8_t v, const uint8_t fmt = 0) { return writeInt(BP(&v), false, 1, fmt); }
  ArduMon& sendRaw(const   int8_t v, const uint8_t fmt = 0) { return writeInt(BP(&v), true,  1, fmt); }
  ArduMon& sendRaw(const uint16_t v, const uint8_t fmt = 0) { return writeInt(BP(&v), false, 2, fmt); }
  ArduMon& sendRaw(const  int16_t v, const uint8_t fmt = 0) { return writeInt(BP(&v), true,  2, fmt); }
  ArduMon& sendRaw(const uint32_t v, const uint8_t fmt = 0) { return writeInt(BP(&v), false, 4, fmt); }
  ArduMon& sendRaw(const  int32_t v, const uint8_t fmt = 0) { return writeInt(BP(&v), true,  4, fmt); }
  ArduMon& sendRaw(const uint64_t v, const uint8_t fmt = 0) { return writeInt(BP(&v), false, 8, fmt); }
  ArduMon& sendRaw(const  int64_t v, const uint8_t fmt = 0) { return writeInt(BP(&v), true,  8, fmt); }

  //binary mode: send little-endian float or double bytes
  //text mode: send space separator if necessary, then send float or double as decimal or scientific
  //on AVR both double and float are 4 bytes by default; on other platforms double may be 8 bytes
  //precision is the minimum number of fraction digits (i.e. after the decimal point), right padded with 0s
  //if precision is negative then automatically use the minimum number of fraction digits
  //width only applies to non-scientific; if positive then left-pad the result with spaces to the specified minimum
  //width is limited to 10 for 4 byte float, 18 for 8 byte double
  ArduMon& send(const float v, bool scientific = false, int8_t precision = -1, int8_t width = -1) {
    return sendTextSep().sendRaw(v, scientific, precision, width);
  }
  ArduMon& send(const double v, bool scientific = false, int8_t precision = -1, int8_t width = -1) {
    return sendTextSep().sendRaw(v, scientific, precision, width);
  }

  //binary mode: send little-endian float or double bytes
  //text mode: send float or double as decimal or scientific string
  //see further comments for send(float)
  ArduMon& sendRaw(const float v, bool scientific = false, int8_t precision = -1, int8_t width = -1) {
    return writeFloat(v, scientific, precision, width);
  }
  ArduMon& sendRaw(const double v, bool scientific = false, int8_t precision = -1, int8_t width = -1) {
    return writeFloat(v, scientific, precision, width);
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
  char getKey() { return getKeyImpl(); }

  //below are conveniences for a partial set of ANSI/VT100 control codes; for more details see
  //https://vt100.net
  //https://github.com/martyvona/ArduMon/blob/main/extras/vtansi.htm
  //https://github.com/martyvona/ArduMon/blob/main/extras/VT100_Escape_Codes.html

  ArduMon& vt100ClearRight()    { return writeStr(F("\x1B[0K")); }
  ArduMon& vt100ClearLine()     { return writeStr(F("\r\x1B[2K")); }
  ArduMon& vt100CursorVisible() { return writeStr(F("\x1B[?25h")); }
  ArduMon& vt100CursorHidden()  { return writeStr(F("\x1B[?25l")); }

  static const char VT100_UP = 'A', VT100_DOWN = 'B', VT100_RIGHT = 'C', VT100_LEFT = 'D';

  //binary mode: noop
  //text mode: move VT100 cursor n places in dir
  ArduMon& vt100MoveRel(const uint16_t n, const char dir) {
    if (binary_mode || !with_text) return *this;
    if (!writeChar('\x1B').writeChar('[')) return *this;
    if (n < 10 && !sendRaw(static_cast<char>('0' + n))) return *this;
    if (n >= 10 && !sendRaw(n)) return *this;
    return writeChar(dir);
  }

  //binary mode: noop
  //text mode: move VT100 cursor to (row, col)
  ArduMon& vt100MoveAbs(const uint16_t row, const uint16_t col) {
    if (binary_mode || !with_text) return *this;
    return writeChar('\x1B').writeChar('[').sendRaw(row).writeChar(';').sendRaw(col).writeChar('H');
  }

  static const char VT100_ATTR_RESET = '0', VT100_ATTR_BRIGHT = '1', VT100_ATTR_UNDERSCORE = '4';
  static const char VT100_ATTR_BLINK = '5', VT100_ATTR_REVERSE = '7';

  //binary mode: noop
  //text mode: set one of the VT100_ATTR_* attributes
  ArduMon& vt100SetAttr(const char attr) {
    if (binary_mode || !with_text) return *this;
    return writeChar('\x1B').writeChar('[').writeChar(attr).writeChar('m');
  }

  static const char VT100_FOREGROUND = '3', VT100_BACKGROUND = '4';
  static const char VT100_BLACK = '0', VT100_RED = '1', VT100_GREEN = '2', VT100_YELLOW = '3', VT100_BLUE = '4';
  static const char VT100_MAGENTA = '5', VT100_CYAN = '6', VT100_WHITE = '7';

  //binary mode: noop
  //text mode: set one of the VT100_* colors
  ArduMon& vt100SetColor(const char fg_bg, const char color) {
    if (binary_mode || !with_text) return *this;
    return writeChar('\x1B').writeChar('[').writeChar(fg_bg).writeChar(color).writeChar('m');
  }

  //below are static utility methods 

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
  static char toHex(const uint8_t i) { return (i&0x0f) < 10 ? ('0' + (i&0x0f)) : ('A' + ((i&0x0f) - 10)); }

  //no ato[u]ll() or strto[u]ll() on AVR, and Arduino Stream::parseInt() doesn't handle 64 bits
  static bool parseInt64(const char *s, int64_t &v) {
    return parseDec(s, BP(&v), true, 8, static_cast<int64_t>(0), static_cast<uint64_t>(0));
  }

  static bool parseUInt64(const char *s, uint64_t &v) {
    return parseDec(s, BP(&v), false, 8, static_cast<int64_t>(0), static_cast<uint64_t>(0));
  }

  //standard Arduino Print::print() APIs don't handle 64 bits
  //fmt is a bitmask of FMT_* flags with low 5 bits specifying minimum field width
  static bool printInt64(Stream *stream, const int64_t &v, const uint8_t fmt = 0) {
    return writeDec(BP(&v), true, 8, fmt, static_cast<uint64_t>(0), [&](const char *buf){
      while (*buf) stream->write(*buf++);
    });
  }

  static bool printUInt64(Stream *stream, const uint64_t &v, const uint8_t fmt = 0) {
    return writeDec(BP(&v), false, 8, fmt, static_cast<uint64_t>(0), [&](const char *buf){
      while (*buf) stream->write(*buf++);
    });
  }

private:

  Stream *stream; //underlying serial stream

  Error err = Error::NONE; //most recent error

  bool binary_mode; //binary mode if true, text mode otherwise

  enum {
    F_TXT_ECHO           = 1 << 0, //echo received characters in text mode, typically for interactive terminal use
    F_TXT_PROMPT_PROGMEM = 1 << 1, //text mode prompt string is in program memory on AVR
    F_RECEIVING          = 1 << 2, //received the first but not yet last byte of a command
    F_HANDLING           = 1 << 3, //a command handler is currently running
    F_SPACE_PENDING      = 1 << 4, //a space should be sent before the next returned value in text mode
    F_ERROR_RUNNABLE     = 1 << 5, //error_handler is a runnable
    F_UNIV_RUNNABLE      = 1 << 6, //universal_handler is a runnable
    F_FALLBACK_RUNNABLE  = 1 << 7  //fallback_handler is a runnable
  };
  uint8_t flags = 0;

  const char *txt_prompt = 0; //prompt string in text mode, 0 if none

  millis_t recv_deadline = 0, recv_timeout_ms = 0; //receive timeout, disabled by default

  uint8_t arg_count = 0;

  //unfortunately zero length arrays are technically not allowed
  //though many compilers won't complain unless in pedantic mode
  //send_buf is not used in text mode, and receive-only applications are possible
  //recv_buf is required to receive commands in both text and binary mode, but send-only applications are possible
  char recv_buf[recv_buf_sz > 0 ? recv_buf_sz : 1], send_buf[send_buf_sz > 0 ? send_buf_sz : 1];

  //next available position in recv_buf while receiving a command
  //last received character when beginning to handle a command
  //start of next read while handling command
  char *recv_ptr = recv_buf;

  //send_buf is only used in binary mode
  //send_read_ptr is the next unsent byte; sending is disabled iff send_read_ptr is 0
  //send_write_ptr is the next free spot; writing to send_buf is disabled if send_write_ptr is 0
  //SEND_OVERFLOW iff send when send_write_ptr == send_buf + send_buf_sz - 1 (reserved for checksum)
  char *send_read_ptr = 0, *send_write_ptr = send_buf;

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

  handler_t getDefaultErrorHandlerImpl() {
    return [](ArduMon &am){
      const Error e = am.clearErr();
      if (am.isBinaryMode()) {
#if ARDUINO
        if (am.stream != &Serial) Serial.println(am.errMsg(e));
#else
        printf("%s\n", am.errMsg(e));
#endif
      } else am.sendCRLF().sendRaw(am.errMsg(e)).sendCRLF(true); 
      return true;
    };
  }

  ArduMon& setHandler(handler_t &which, const handler_t handler, const uint8_t runnable_flag)  {
    which = handler;
    flags &= ~runnable_flag;
    return *this;
  }

  handler_t getHandler(const handler_t handler, const uint8_t runnable_flag) {
    return (flags&runnable_flag) ? 0 : handler;
  }

  ArduMon& setRunnable(Runnable* &which, Runnable* const runnable, const uint8_t runnable_flag)  {
    which = runnable;
    flags |= runnable_flag;
    return *this;
  }

  Runnable* getRunnable(Runnable* const runnable, const uint8_t runnable_flag) {
    return (flags&runnable_flag) ? runnable : 0;
  }

  template <typename T>
  ArduMon& addCmdImpl(const T func, const char *name, const uint8_t code, const char *desc, const bool progmem) {
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
    setFunc(cmds[n_cmds], func);
    ++n_cmds;
    return *this;
  }

  template <typename T> ArduMon& removeCmdImpl(T &key) {
    for (uint8_t i = 0; i < n_cmds; i++) {
      if (cmds[i].is(key)) {
        for (i++; i < n_cmds; i++) cmds[i - 1] = cmds[i];
        --n_cmds;
        break;
      }
    }
    return *this;
  }

  template <typename T> int16_t getCmdCodeImpl(const T *name) {
    for (uint8_t i = 0; i < n_cmds; i++) if (cmds[i].is(name)) return cmds[i].code;
    return -1;
  }

  void setFunc(Cmd& cmd, const handler_t func) { cmd.handler = func; cmd.flags &= ~Cmd::F_RUNNABLE; }
  void setFunc(Cmd& cmd, Runnable* const func) { cmd.runnable = func; cmd.flags |= Cmd::F_RUNNABLE; }

  //does nothing in binary mode, if txt_prompt is null, or if currently handling
  //otherwise sends optional CRLF followed by txt_prompt and a space
  //this cannot cause an ArduMon error
  ArduMon& sendTextPrompt(const bool with_crlf = false) {
    if (!with_text || binary_mode || !txt_prompt || flags&F_HANDLING) return *this;
    if (with_crlf) writeChar('\r').writeChar('\n');  //writeChar() and writeStr() cannot error in text mode
    writeStr(txt_prompt, flags&F_TXT_PROMPT_PROGMEM).writeChar(' ');
    return *this;
  }

  //noop in binary mode or if hasErr()
  //in text mode send a space iff F_SPACE_PENDING
  //in text mode F_SPACE_PENDING is always set before successful return
  ArduMon& sendTextSep() {
    if (binary_mode || !with_text || hasErr()) return *this;
    if (flags&F_SPACE_PENDING) writeChar(' ');
    flags |= F_SPACE_PENDING;
    return *this;
  }

  //can't use std::function on AVR
  template <typename T> bool dispatch(const T& pred) {

    bool retval;
    if (invoke(universal_handler, universal_runnable, flags, F_UNIV_RUNNABLE, retval)) return retval;

    for (uint8_t i = 0; i < n_cmds; i++) {
      if (pred(cmds[i])) {
        if (invoke(cmds[i].handler, cmds[i].runnable, cmds[i].flags, Cmd::F_RUNNABLE, retval)) return retval;
        else return false;
      }
    }

    if (invoke(fallback_handler, fallback_runnable, flags, F_FALLBACK_RUNNABLE, retval)) return retval;

    return fail(Error::BAD_CMD).endHandlerImpl();
  }

  bool invoke(const handler_t handler, Runnable * const runnable, const uint8_t flags, const uint8_t runnable_flag,
              bool &retval) {
    if      ((flags&runnable_flag) && runnable) { retval = runnable->run(*this); return true; }
    else if (!(flags&runnable_flag) && handler) { retval = handler(*this); return true; }
    return false;
  }

  //upon call, recv_ptr is the last received byte, which should be the checksum
  //if the checksum is invalid then BAD_PACKET
  //otherwise set recv_ptr = recv_buf + 1 and dispatch()
  bool handleBinCommand() {
    const uint8_t len = static_cast<uint8_t>(recv_buf[0]);
    uint8_t sum = 0; for (uint8_t i = 0; i < len; i++) sum += static_cast<uint8_t>(recv_buf[i]);
    if (sum != 0) return fail(Error::BAD_PACKET);
    recv_ptr = recv_buf + 1; //skip over length
    arg_count = len - 2; //don't include length or checksum bytes, but include command code byte in arg count
    return dispatch([&](Cmd& cmd){ return arg_count && cmd.code == recv_buf[1]; });
  }

  //upon call, recv_ptr is the last received character, which will be either '\r' or '\n'
  //tokenize recv_buf, parsing quoted characters and strings with escapes, and discarding any line end comment
  //ignore empty commands
  //otherwise set recv_ptr = recv_buf and dispatch()
  bool handleTextCommand() {

    const uint16_t len = recv_ptr - recv_buf + 1;

    if (len <= 1) return endHandlerImpl(); //ignore empty command, e.g. if received just '\r' or '\n'

    const bool save_cmd = (len + 1) <= recv_buf_sz/2; //save cmd to upper half of recv_buf if possible for history
    if (save_cmd) recv_buf[recv_buf_sz/2] = '\n'; //saved command is signaled by recv_buf[recv_buf_sz/2] = '\n'

    bool in_str = false, in_chr = false;
    uint16_t j = 0; //write index
    for (uint16_t i = 0; i < len; i++, j++) {

      char c = recv_buf[i];

      const bool comment_start = !in_str && !in_chr && c == '#';

      //copy original input to upper half of recv_buf as saved command
      if (save_cmd) recv_buf[recv_buf_sz/2 + 1 + i] = (comment_start || c == '\n' || c == '\r')  ? 0 : c;

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

    //null terminate final command token and zero out rest of recv_buf
    //we will always write at least one 0 here because the command ended with '\r' or '\n'
    while (j < recv_buf_sz/2) recv_buf[j++] = 0;
    if (!save_cmd) while (j < recv_buf_sz) recv_buf[j++] = 0;

    char * const end = recv_buf + (save_cmd ? recv_buf_sz/2 : recv_buf_sz);

    recv_ptr = recv_buf;

    while (*recv_ptr == 0) { //skip leading spaces, which are now 0s
      if (++recv_ptr == end) return endHandlerImpl(); //ignore empty command
    }

    char * const tmp = recv_ptr;
    arg_count = 0;
    while (++recv_ptr <= end) { if ((recv_ptr == end || !(*recv_ptr)) && *(recv_ptr - 1)) ++arg_count; }
    recv_ptr = tmp;

    //DEBUG for (uint16_t k = 0; k < recv_buf_sz; k++) std::cerr << "recv_buf[" << k << "]=" << +recv_buf[k] << "\n";

    const char *cmd_name = CCS(nextTok(0));
    recv_ptr = tmp; //first token returned to command handler should be the command token itself

    return dispatch([&](Cmd& cmd){ return cmd.is(cmd_name); });
  }

  //advance recv_ptr to the start of the next input token in text mode and return the current token
  //return 0 if there are no more input tokens or already hasErr()
  //advance recv_ptr by binary_bytes in binary mode and return its previous value
  //unless there are not that many bytes remaining in the received packet, in which case return 0
  //if binary_bytes is 0 in binary mode then advance to the end of null terminated string
  const char * nextTok(const uint8_t binary_bytes) {

    if (hasErr()) return 0;

#define FAIL { fail(Error::RECV_UNDERFLOW); return 0; }

    //caution UB dragonnes https://pvs-studio.com/en/blog/posts/cpp/1199/
    //pointer arithmetic must result in an address within the array or "one past the end"

    if (recv_ptr - recv_buf >= recv_buf_sz) FAIL; //nothing left to receive

    const char *ret = recv_ptr;

    if (binary_mode && binary_bytes > 0) {

      //recv_buf[0] is the received packet length; can only receive up to one less than that
      //because the last packet buyte is the checksum which can't itself be received
      //this test also ensures that the requested binary_bytes are available
      if ((recv_ptr - recv_buf) + binary_bytes >= recv_buf[0]) FAIL;

      recv_ptr += binary_bytes; //advance recv_ptr for next receive

    } else { //!binary_mode || !binary_bytes

      if (!binary_mode && *ret == '\n') FAIL; //can't receive start of saved command

      //skip non-null characters of current token, but there needs to be at least one null after it
      while (*recv_ptr) if (++recv_ptr - recv_buf == recv_buf_sz) FAIL;

      //skip nulls to advance recv_ptr to beginning of next token, if any, for next receive
      while (*recv_ptr == 0) if (++recv_ptr - recv_buf == recv_buf_sz) break;
    }

#undef FAIL

    return ret;
  }

  //binary mode: receive a byte with value 0 or nonzero
  //text mode: receive "true", "false", "t", "f", "0", "1", "yes", "no", "y", "n" or uppercase equivalents
  ArduMon& parseBool(const char *v, bool *dst) {
    if (hasErr()) return *this;
    if (!v) return fail(Error::RECV_UNDERFLOW);
    if (binary_mode || !with_text) { *dst = *v != 0; return *this; }
    switch (*v++) {
      case '0': if (*v == 0) { *dst = false; return *this; } else return fail(Error::BAD_ARG);
      case '1': if (*v == 0) { *dst = true; return *this; } else return fail(Error::BAD_ARG);
      case 't': case 'T': if (*v == 0) { *dst = true; return *this; } else return chkSfx(F("RUE"), true, v, dst);
      case 'f': case 'F': if (*v == 0) { *dst = false; return *this; } else return chkSfx(F("ALSE"), false, v, dst);
      case 'y': case 'Y': if (*v == 0) { *dst = true; return *this; } else return chkSfx(F("ES"), true, v, dst);
      case 'n': case 'N': if (*v == 0) { *dst = false; return *this; } else return chkSfx(F("O"), false, v, dst);
      default: return fail(Error::BAD_ARG);
    }
  }

  ArduMon& chkSfx(const FSH *sfx, const bool ret, const char *v, bool *dest) {
    const char *s_sfx = CCS(sfx);
    while (true) {
      const char s = pgm_read_byte(s_sfx++), c = *CCS(v++); //non-AVR Arduino pgm_read_byte() is passthrough
      if (s == 0) { if (c == 0) { *dest = ret; return *this; } else return fail(Error::BAD_ARG); }
      if (s != c && !((s + 32) == c)) return fail(Error::BAD_ARG); //'A' + 32 == 'a'
    }
  }

  //binary mode: copy num_bytes int from v to dest
  //text mode: parse a null terminated decimal or hexadecimal int from v to num_bytes at dest
  //if hex == true then always interpret as hex, else interpret as hex iff prefixed with 0x or 0X
  ArduMon& parseInt(const char *v, char *dest, const bool sgnd, const uint8_t num_bytes, bool hex) {

    if (hasErr()) return *this;

    if (!v) return fail(Error::RECV_UNDERFLOW);

    if (binary_mode || !with_text) { memcpy(dest, v, num_bytes); return *this; }

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
        switch (num_bytes) {
          case 1: if (tmp < INT8_MIN || tmp > INT8_MAX) return fail(Error::BAD_ARG); else break;
          case 2: if (tmp < INT16_MIN || tmp > INT16_MAX) return fail(Error::BAD_ARG); else break;
          case 4: if (tmp < INT32_MIN || tmp > INT32_MAX) return fail(Error::BAD_ARG); else break;
          //otherwise rely on errno == ERANGE
        }
        //caution strict aliasing UB dragonnes https://gist.github.com/shafik/848ae25ee209f698763cffee272a58f8
        //it should be OK to cast unsigned to signed
        *(reinterpret_cast<long int*>(&ret)) = tmp;
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
      memcpy(dest, &ret, num_bytes);
      return *this;
    }

    //sizeof(long) is typically 4 on both AVR and ESP32, so typically only get here if num_bytes == 8
    //but just in case, handle the case that sizeof(long) < 4 as well, though compiler should usually optimize it out
    if (num_bytes <= 4 && sizeof(long) < 4) return parseDec<int32_t, uint32_t>(s, dest, sgnd, num_bytes);

    if (with_int64) return parseDec<int64_t, uint64_t>(s, dest, sgnd, num_bytes);

    return fail(Error::UNSUPPORTED);
  }

  //parse a null terminated decimal int from s to num_bytes at dest
  template <typename big_int, typename big_uint> //supports int32_t/uint32_t, int64_t/uint64_t
  ArduMon& parseDec(const char *s, char *dest, const bool sgnd, const uint8_t num_bytes) {
    if (binary_mode || !with_text) return fail(Error::UNSUPPORTED);
    if (!parseDec(s, dest, sgnd, num_bytes, static_cast<big_int>(0), static_cast<big_uint>(0)))
      return fail(Error::BAD_ARG);
    return *this;
  }

  template <typename big_int, typename big_uint> //supports uint32_t, uint64_t
  static bool parseDec(const char *s, char *dest, const bool sgnd, const uint8_t num_bytes,
                       const big_int &tag, const big_uint &utag) {
    
    const int8_t sign = *s == '-' ? -1 : +1;

    const bool neg = sign < 0;
    if (neg && !sgnd) return false;
    
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
    for (uint8_t i = 0; *dig; i++, dig++) if (i > max_digits) return false;

    //read digits from least to most significant in 4 digit chunks
    constexpr uint8_t num_chunks = sizeof(big_uint) > 4 ? 5 : 3;
    int16_t chunk[num_chunks];
    for (uint8_t i = 0; i < num_chunks; i++) chunk[i] = 0;
    for (uint8_t c = 0; c <= last_chunk && dig > s; c++) {
      for (uint16_t place = 1; place <= 1000; place *= 10) {
        if (--dig < s) break; //no more digits to read
        const char d = *dig;
        if (d < '0' || d > '9') return false;
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
      if (chunk[c] > max_chunk) return false;
      if (chunk[c] < max_chunk) break;
    }
    
    big_uint ret = 0;
    if (sgnd) {
      //caution strict aliasing UB dragonnes https://gist.github.com/shafik/848ae25ee209f698763cffee272a58f8
      //it should be OK to cast unsigned to signed
      big_int place = sign, *sret = reinterpret_cast<big_int*>(&ret);
      for (uint8_t c = 0; c <= last_chunk; c++, place *= 10000) *sret += chunk[c] * place;
    } else {
      big_uint place = 1;
      for (uint8_t c = 0; c <= last_chunk; c++, place *= 10000) ret += chunk[c] * place;
    }

    memcpy(dest, &ret, num_bytes);

    return true;
  }

  //binary mode: copy a 4 byte float or 8 byte double from v to dest
  //text mode: parse a null terminated decimal or scientific number from v to a 4 byte float or 8 byte double at dest
  template <typename T> ArduMon& parseFloat(const char *v, T *dest) {
    if (!with_float) return fail(Error::UNSUPPORTED);
    if (hasErr()) return *this;
    if (!v) return fail(Error::RECV_UNDERFLOW);
    if (binary_mode || !with_text) { memcpy(dest, v, sizeof(T)); return *this; }
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
  ArduMon& writeBool(const bool v, const BoolStyle style, const bool upper_case) {
    if (hasErr()) return *this;
    if (binary_mode || !with_text) return writeChar(v ? 1 : 0);
    switch (style) {
      case BoolStyle::TRUE_FALSE: return writeCaseStr(v ? F("TRUE") : F("FALSE"), !upper_case);
      case BoolStyle::TF: return writeCaseStr(v ? F("T") : F("F"), !upper_case);
      case BoolStyle::ZERO_ONE: return writeChar(v ? '1' : '0');
      case BoolStyle::YES_NO: return writeCaseStr(v ? F("YES") : F("NO"), !upper_case);
      case BoolStyle::YN: return writeCaseStr(v ? F("Y") : F("N"), !upper_case);
			default: return fail(Error::UNSUPPORTED);
    }
  }

  //binary mode: Error::UNSUPPORTED
  //text mode: append string to send buffer, not including terminating null
  //assume string is supplied in program memory in uppercase, convert to lowercase iff to_lower is true
  ArduMon& writeCaseStr(const FSH *uppercase, const bool to_lower) {
    if (binary_mode || !with_text) return fail(Error::UNSUPPORTED);
    uint8_t len = 0; while (CCS(uppercase)[len] != 0) ++len;
    char * const write_start = send_write_ptr;
    for (uint8_t i = 0; i < len + 1; i++) {
      const char c = pgm_read_byte(CCS(uppercase) + i);
      if (c) put(to_lower ? c + 32 : c); //'A' + 32 = 'a'
    }
    if (!send_read_ptr) send_read_ptr = write_start; //enable sending
    return *this;
  }

  //binary mode: append num_bytes int to send buffer
  //text mode: append num_bytes int starting at v as decimal or hexadecimal string in send buffer
  ArduMon& writeInt(const char *v, const bool sgnd, const uint8_t num_bytes, const uint8_t fmt) {

    if (hasErr()) return *this;

    if (binary_mode || !with_text) {
      if (!checkWrite(num_bytes)) return fail(Error::SEND_OVERFLOW);
      for (uint8_t i = 0; i < num_bytes; i++) put(v[i]);
      return *this;
    }

    if (fmt&FMT_HEX) {
      char * const write_start = send_write_ptr;
      const uint8_t width = fmt&(~(FMT_HEX|FMT_PAD_ZERO|FMT_PAD_RIGHT));
      const char c = fmt&FMT_PAD_ZERO ? '0' : ' ';
      const uint8_t pad = width > 2*num_bytes ? width - 2*num_bytes : 0;
      if (pad && !(fmt&FMT_PAD_RIGHT)) for (uint8_t i = 0; i < pad; i++) put(c);
      for (uint8_t i = num_bytes; i > 0; i--) { put(toHex(v[i-1] >> 4)); put(toHex(v[i-1] & 0x0f)); }
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
        memcpy(&i, v, num_bytes);
#ifdef ARDUINO
        ltoa(i, buf, 10);
#else
        snprintf(buf, sizeof(buf), "%ld", i);
#endif
      } else {
        unsigned long i = 0;
        memcpy(&i, v, num_bytes);
#ifdef ARDUINO
        ultoa(i, buf, 10);
#else
        snprintf(buf, sizeof(buf), "%lu", i);
#endif
      }
      pad(buf, buf_sz, fmt);
      return writeStr(buf);
    }

    //sizeof(long) is typically 4 on both AVR and ESP32, so typically only get here if num_bytes == 8
    //but just in case handle sizeof(long) < 4 as well, though compiler should typically optimize it out
    if (num_bytes <= 4 && sizeof(long) < 4) return writeDec<uint32_t>(v, sgnd, num_bytes, fmt);

    if (with_int64) return writeDec<uint64_t>(v, sgnd, num_bytes, fmt);

    return fail(Error::UNSUPPORTED);
  }

  template <typename big_uint> //supports uint32_t, uint64_t
  ArduMon& writeDec(const char *v, const bool sgnd, const uint8_t num_bytes, const uint8_t fmt) {
    if (binary_mode || !with_text ||
        !writeDec(v, sgnd, num_bytes, fmt, static_cast<big_uint>(0), [&](const char *buf){ writeStr(buf); }))
      return fail(Error::UNSUPPORTED);
    return *this;
  }

  //can't use std::function on AVR; OutFunc takes a const char * and returns void
  template <typename big_uint, typename OutFunc> //supports uint32_t, uint64_t
  static bool writeDec(const char *v, const bool sgnd, const uint8_t num_bytes, const uint8_t fmt,
                       const big_uint &tag, const OutFunc &out) {

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
          if (i == 0) return false; //shouldn't happen
          buf[--i] = '0' + (r - qq * 10);
          r = qq;
        }
      }
      //if uint16_t is changed to big_uint below then the next loop would be sufficient on its own
      //but the loop above reduces the amount of big_uint math
      while (num) {
        uint16_t q = num / 10;
        if (i == 0) return false; //shouldn't happen
        buf[--i] = '0' + (num - q * 10);
        num = q;
      }
    }
  
    if (neg) {
      if (i == 0) return false; //shouldn't happen
      buf[--i] = '-';
    }

    if (i > 0) for (uint8_t j = 0; i < len; i++, j++) buf[j] = buf[i];

    pad(buf, buf_sz, fmt);

    out(buf);

    return true;
  }

  //binary mode: append float or double v to send buffer
  //text mode: append float or double as decimal or scientific number in send buffer
  //on AVR double is synonymous with float by default, both are 4 bytes; otherwise double may be 8 bytes
  template <typename T> //supports float and double
  ArduMon& writeFloat(const T v, const bool scientific, const int8_t precision, const int8_t width) {

    if (!with_float) return fail(Error::UNSUPPORTED);

    if (hasErr()) return *this;

    constexpr uint8_t nb = with_double ? sizeof(T) : sizeof(float); //4 or 8
    if (nb < sizeof(T)) return fail(Error::UNSUPPORTED); //T = double, sizeof(double) > sizeof(float), !with_double

    if (binary_mode || !with_text) {
      if (!checkWrite(nb)) return fail(Error::SEND_OVERFLOW);
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
        uint8_t k = trimBackwards(buf, j - 1);
        while (buf[j]) buf[++k] = buf[j++];
        buf[++k] = 0;
      }
      return writeStr(buf, false, false, -1);
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
      if (precision < 0) trimBackwards(buf, buf_sz - 1);
      return writeStr(buf);
    }
  }

  //trim trailing whitespace and zeros backwards from start index; returns next un-trimmed index
  //stops trimming at character after '.' or first char
  //e.g. "1.23400 " -> "1.234", "1.000" -> "1.0", ".0" -> ".0", "000" -> "0"
  static uint8_t trimBackwards(char *s, const uint8_t start) {
    uint8_t i = start;
    while (i > 0 && (s[i - 1] != '.') && (s[i] == 0 || s[i] == '0' || s[i] == ' ')) s[i--] = 0;
    return i;
  }


  //binary mode: append char to send buffer
  //text mode: if cook quote and escape iff necessary, then append char to send buffer
  ArduMon& writeChar(const char c, const bool cook = false) {
    if (hasErr()) return *this;
    if (binary_mode || !with_text) {
      if (!checkWrite(1)) return fail(Error::SEND_OVERFLOW);
      put(c);
    } else {
      const char esc = cook ? escape(c, '\'') : 0;
      const bool quote = cook && (isspace(c) || esc);
      char * const write_start = send_write_ptr;
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
  ArduMon& writeStr(const char *v, const bool progmem = false, const bool cook = false, const int16_t len = -1) {

    if (!v || hasErr() || len == 0) return *this;

    bool quote = false;
    uint16_t n = 0, n_esc = 0;
    while (len < 0 && v[n]) {
      if (!binary_mode && cook && escape(v[n], '"')) { ++n_esc; quote = true; }
      else if (!binary_mode && cook && isspace(v[n])) quote = true;
      ++n;
    }

    if (binary_mode && !checkWrite((len > 0) ? len : (n + n_esc + (quote ? 2 : 0) + 1))) {
      return fail(Error::SEND_OVERFLOW);
    }

    char * const write_start = send_write_ptr;

    if (quote) put('"');

    for (uint16_t i = 0; len < 0 || i < len; i++) {
      const char c = progmem ? pgm_read_byte(v + i) : v[i];
      const char esc = (c && !binary_mode && cook && len < 0) ? escape(c, '"') : 0;
      if (esc) { put('\\'); put(esc); } else if (c || binary_mode) put(c);
      if (len < 0 && !c) break; //wrote terminating null
    }

    if (quote) put('"');

    if (!binary_mode && !send_read_ptr) send_read_ptr = write_start; //enable sending

    return *this;
  }

#ifdef ARDUINO
  ArduMon& writeStr(const FSH *v, const bool cook = false, const int16_t len = -1) {
    return writeStr(CCS(v), true, cook, len);
  }
#endif

  //text mode: noop
  //binary mode: check if there are at least n free bytes available in send_buf
  bool checkWrite(const uint16_t n) {
    if (!binary_mode || !with_binary) return true;
    if (!send_write_ptr) return false;
    if (send_write_ptr + n >= send_buf + send_buf_sz) return false; //reserve byte for checksum
    return true;
  }

  //text mode: send byte to output serial stream, blocking if necessary
  //binary mode: append a byte to send_buf and advance send_write_ptr
  //(send_read_ptr is updated in the write(...) functions which call this)
  //assumes checkWrite() already returned true
  void put(const char c) {
    if (binary_mode || !with_text) *send_write_ptr++ = c;
    else stream->write(c);
  }

  //see getSendBufUsed()
  uint16_t sendBufUsed() {
    if (!binary_mode || !with_binary) return 0;
    return send_read_ptr ? send_buf[0] : (send_write_ptr - send_buf) + 1; //+1 for checksum
  }

  //see getRecvBufUsed()
  uint16_t recvBufUsed() {
    if (!(flags&F_RECEIVING) && !(flags&F_HANDLING)) return 0;
    if (flags&F_RECEIVING) return recv_ptr - recv_buf;
    if (binary_mode || !with_text) return recv_buf[0];
    char *last_non_null = recv_buf + recv_buf_sz - 1;
    while (last_non_null >= recv_buf && *last_non_null == 0) --last_non_null;
    return (last_non_null - recv_buf) + 1;
  }

  //see setBinaryMode(), reset()
  ArduMon& setBinaryModeImpl(const bool binary, const bool force = false, const bool with_crlf = false) {
    if ((binary && !with_binary) || (!binary && !with_text)) return fail(Error::UNSUPPORTED);
    if (!force && binary_mode == binary) return *this;
    binary_mode = binary;
    flags &= ~(F_SPACE_PENDING | F_HANDLING | F_RECEIVING);
    recv_ptr = recv_buf;
    send_read_ptr = 0;
    arg_count = 0;
    err = Error::NONE;
    if (binary_mode || !with_text) send_write_ptr = send_buf + 1; //enable writing send buf, first byte for length
    else { send_write_ptr = send_buf; sendTextPrompt(with_crlf); }
    return *this;
  }

  //see update()
  ArduMon& updateImpl() {

    if ((flags&F_RECEIVING) && recv_timeout_ms > 0 && millis() > recv_deadline) fail(Error::RECV_TIMEOUT);

    while (!hasErr() && !(flags&F_HANDLING) && stream->available()) { //pump receive buffer

      if (recv_ptr - recv_buf >= recv_buf_sz) { fail(Error::RECV_OVERFLOW); break; }
      
      *recv_ptr = static_cast<char>(stream->read());
      
      if (recv_ptr == recv_buf) { //received first command byte
        flags |= F_RECEIVING;
        recv_deadline = millis() + recv_timeout_ms;
      }

      if (binary_mode || !with_text) {
        
        if (recv_ptr == recv_buf) { //received length
          if (*recv_ptr < 2) fail(Error::BAD_PACKET);
          else ++recv_ptr;
        } else if ((recv_ptr - recv_buf) + 1 == recv_buf[0]) { //received full packet
          flags &= ~F_RECEIVING; flags |= F_HANDLING;
          if (!handleBinCommand()) fail(Error::BAD_HANDLER).endHandlerImpl();
          break; //handle at most one command per update()
        } else ++recv_ptr;

        continue;
      }

      //text mode

      if (*recv_ptr == '\r' || *recv_ptr == '\n') { //text mode end of command
        //interactive terminal programs like minicom will send '\r'
        //but if we only echo that, then the cursor will not advance to the next line
        if (flags&F_TXT_ECHO) { writeChar('\r'); writeChar('\n'); } //ignore echo errors
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
        if (!handleTextCommand()) fail(Error::BAD_HANDLER).endHandlerImpl();
        break; //handle at most one command per update() 
      }

      if (*recv_ptr == '\b' || *recv_ptr == 0x7F) { //text mode backspace or DEL
        if (recv_ptr > recv_buf) {
          if (flags&F_TXT_ECHO) { vt100MoveRel(1, VT100_LEFT); vt100ClearRight(); }
          --recv_ptr;
        }
        continue;
      }

      //debug; this is how to see e.g. that hitting backspace in arduino-cli monitor on OS X actually sends DEL...
      //writeChar('\r'); writeChar('\n');
      //writeChar(toHex((*recv_ptr)>>4)); writeChar(toHex(*recv_ptr));
      //writeChar('\r'); writeChar('\n');

      //text mode command character
      
      //catch and ignore VT100 movement codes: up <ESC>[A, down <ESC>[B, right <ESC>[C, left <ESC>[D
      //these are sent by minicom when the user hits the arrow keys on the keyboard
      //we unfortunately don't support command line editing with these
      //except for possibly 1 line of command history, if there was room in the upper half of recv_buf
      bool esc_seq_end = (*recv_ptr >= 'A' && *recv_ptr <= 'D') &&
        recv_ptr > (recv_buf+1) && *(recv_ptr-1) == '[' && *(recv_ptr-2) == 27;
      
      bool esc_seq_pending = *recv_ptr == 27 || (*recv_ptr == '[' && recv_ptr > recv_buf && *(recv_ptr-1) == 27);
      
      if ((flags&F_TXT_ECHO) && !(esc_seq_end || esc_seq_pending)) writeChar(*recv_ptr);
      
      if (!esc_seq_end) ++recv_ptr; //common case
      else if (*recv_ptr == 'A' && (recv_ptr - recv_buf) < recv_buf_sz/2 && recv_buf[recv_buf_sz/2] == '\n') {
        //user hit up arrow and we have a saved previous command: switch to it
        vt100ClearLine();
        sendTextPrompt();
        recv_ptr = recv_buf;
        for (uint16_t i = recv_buf_sz/2 + 1; i < recv_buf_sz && recv_buf[i]; i++) {
          writeChar(recv_buf[i]);
          *recv_ptr++ = recv_buf[i];
        }
      } else recv_ptr -= 2;  //esc_seq_end but wasn't up arrow or we didn't have saved command: ignore escape sequence

    } //pump receive buffer

    //RECV_OVERFLOW, RECV_TIMEOUT, BAD_CMD, BAD_PACKET, PARSE_ERR, UNSUPPORTED
    if (!isHandling() && hasErr() && handleErrImpl()) endHandlerImpl().sendTextPrompt();

    if (binary_mode) pumpSendBuf(0);

    return *this;
  }

  void pumpSendBuf(const millis_t wait_ms) {
    if (!binary_mode || !with_binary) return;
    const millis_t deadline = millis() + wait_ms;
    do {
      while (send_read_ptr != 0 && stream->availableForWrite()) {
        stream->write(*send_read_ptr++);
        if (send_read_ptr - send_buf == send_buf[0]) { //sent entire packet
          send_read_ptr = 0; //disable reading from send buf
          send_write_ptr = send_buf + 1; //enable writing to send buf, reserve first byte for length
        }
      }
      //reduce repetitive calls to millis() which temporarily disables interrupts
      if (send_read_ptr != 0 && wait_ms > 0) delayMicroseconds(10);
    } while (send_read_ptr != 0 && wait_ms > 0 && (wait_ms == ALWAYS_WAIT || millis() < deadline));
  }

  void pumpSendBuf() { pumpSendBuf(send_wait_ms); }

  ArduMon& handleErrImpl() {
    if (err != Error::NONE &&
        ((flags&F_ERROR_RUNNABLE && error_runnable && error_runnable->run(*this)) ||
         (!(flags&F_ERROR_RUNNABLE) && error_handler && error_handler(*this)))) {
      err = Error::NONE;
    }
    return *this;
  }

  //see endHandler()
  ArduMon& endHandlerImpl() {

    const bool was_handling = flags&F_HANDLING; //tolerate being called when not actually handling

    //BAD_HANDLER, RECV_UNDERFLOW, BAD_ARG, SEND_OVERFLOW, UNSUPPORTED
    handleErrImpl();

    if (!binary_mode) sendCRLF();

    flags &= ~(F_SPACE_PENDING | F_HANDLING);
    recv_ptr = recv_buf;
    arg_count = 0;

    if (!was_handling) return *this;

    else if (!binary_mode || !with_binary) return sendTextPrompt();
    else return sendPacketImpl();
  }

  ArduMon& sendPacketImpl() {

    if (!binary_mode || !with_binary) return *this;

    const uint16_t len = send_write_ptr - send_buf;

    //checkWrite() already ensured that len < send_buf_sz, so the next line is redundant
    //also it's better to have send_packet_impl() not be able to generate an error
    //since it's called after handle_err_impl() in endHandlerImpl()
    //if (len >= send_buf_sz) return fail(Error::SEND_OVERFLOW); //need 1 byte for checksum

    if (len > 1) { //ignore empty packet, but first byte of send_buf is reserved for length
      send_buf[0] = static_cast<uint8_t>(len + 1); //set packet length including checksum
      uint8_t sum = 0; for (uint8_t i = 0; i < len; i++) sum += static_cast<uint8_t>(send_buf[i]);
      send_buf[len] = static_cast<uint8_t>(-sum); //set packet checksum
      send_write_ptr = 0; //disable writing to send buf
      send_read_ptr = send_buf; //enable reading from send buf
      pumpSendBuf();
    } //else send_write_ptr must still be send_buf + 1 and send_read_ptr = 0

    return *this;
  }

  ArduMon& sendCmdsImpl() {
    if (binary_mode || !with_text) return *this;
    for (uint8_t i = 0; i < n_cmds; i++) { //sending cannot error in text mode
      writeChar(toHex(cmds[i].code >> 4)).writeChar(toHex(cmds[i].code)).writeChar(' ');
      if (cmds[i].name) writeStr(cmds[i].name, cmds[i].flags&Cmd::F_PROGMEM);
      if (cmds[i].description) writeChar(' ').writeStr(cmds[i].description, cmds[i].flags&Cmd::F_PROGMEM);
      sendCRLF(true);
    }
    return *this;
  }

  char getKeyImpl() {
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

  static void pad(char *buf, const uint8_t buf_sz, const uint8_t fmt) {
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

  //caution strict aliasing UB dragonnes https://gist.github.com/shafik/848ae25ee209f698763cffee272a58f8
  //it should be OK to cast to [const] char*

  static const char * CCS(const void *s) { return reinterpret_cast<const char*>(s); }

  static char * BP(void *p) { return reinterpret_cast<char*>(p); }

  static const char * BP(const void *p) { return reinterpret_cast<const char*>(p); }
};

#endif
