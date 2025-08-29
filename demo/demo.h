/**
 * ArduMon: Yet another Arduino serial command library.
 *
 * See https://github.com/martyvona/ArduMon/blob/main/README.md
 *
 * This is the main code of the demo.  It can compile for Arduino when included in demo.ino, and it can also compile for
 * the native host when included in native/demo.cpp.  By default it implements a text mode ArduMon server supporting a
 * small catalog of demonstration commands, including commands to echo values of various types, as well as a countdown
 * timer (see AM_Timer.h).  It can also compile as a binary (not text) server with BINARY=true (see
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

#include "AM_Timer.h"

//builds text server demo by default
//#define BASELINE_MEM //to check memory usage of boilerplate
//#define BINARY true //to build binary server
//#define BINARY_CLIENT //to build binary client

/* configure ArduMon **************************************************************************************************/

#ifdef BINARY_CLIENT
#define BINARY true
#endif

#ifndef BINARY
#define BINARY false
#endif

//comment these out to compile the binary demo (client or server) to use default serial port for binary communication
//this will disable debug prints, but can be useful e.g. to run the binary server on an Arduino
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
using AM = ArduMon<MAX_CMDS, RECV_BUF_SZ, SEND_BUF_SZ, WITH_INT64, WITH_FLOAT, WITH_DOUBLE, WITH_BINARY, WITH_TEXT>;

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

#else //text mode demo, or binary mode but BIN_RX_PIN not defined

//connect an Arduino by USB and run minicom or screen on USB serial port as described in README.md
#define AM_STREAM Serial

#endif //BINARY && defined(BIN_RX_PIN)

#endif //ARDUINO

//AM_STREAM is defined externally for native build

/* create the ArduMon instance am and other global state **************************************************************/

#ifndef BASELINE_MEM

AM am(&AM_STREAM, BINARY); //the ArduMon instance

//noop if ArduMon is not currently in error, otherwise clear the error
//then on native print the error to stdout; on Arduino print the error to the default Serial port
//except in binary mode when BIN_RX_PIN is not defined, in which case the error is swallowed
//this is used by the demo code for reporting ArduMon errors outside the context of a command handler
void print_error() { if (am.has_err()) println(AM::err_msg(am.clear_err())); }

bool done = false; //terminate the demo when this flag is set

//this error handler wraps the default one and adds a total count
uint16_t num_errors = 0;
bool count_errors(AM &am) { if (num_errors < 65535) ++num_errors; return am.get_default_error_handler()(am); }

#ifdef BINARY_CLIENT
#include "binary_client.h" //most of the demo binary client here
#else
#include "server_commands.h" //most of the demo server (both text and binary mode) here
#endif

#endif //BASELINE_MEM

/* Arduino setup() and loop() *****************************************************************************************/

void setup() {
#ifdef ARDUINO
  pinMode(LED_BUILTIN, OUTPUT);
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
  am.set_error_handler(count_errors);
#ifdef BINARY_CLIENT
  am.set_send_wait_ms(AM::ALWAYS_WAIT);
#else
  add_cmds(); //text or binary server
#endif
#endif //BASELINE_MEM
}

void loop() {
#ifndef BASELINE_MEM
#ifdef ARDUINO
  if (done) {
    digitalWrite(LED_BUILTIN, !num_errors || !((millis()/1000)%2)); //blink LED if there were errors, solid otherwise
    return;
  }
#endif
  am.update();
#ifndef BINARY_CLIENT
  timer.tick(am); //text or binary server: tick the timer
#else //binary client: crank the state machine
  BCStage *next; if (current_bc_stage && (next = current_bc_stage->update())) current_bc_stage = next;
#endif //BINARY_CLIENT
#endif //BASELINE_MEM
}

