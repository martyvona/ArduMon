/**
 * ArduMon: Yet another Arduino serial command library.
 *
 * See https://github.com/martyvona/ArduMon/blob/main/README.md
 *
 * This "native" demo driver compiles and runs on an OS X or Linux host, including under WSL on Windows.  WSL2 does not
 * support UNIX sockets, which are used in some modes, so in those cases use WSL1 on Windows.
 *
 * The build-native.sh script builds this file in two different ways:
 * * with DEMO_CLIENT not defined, resulting in executable server file "ardumon_server"
 * * with DEMO_CLIENT defined, resulting in executable file "ardumon_client"
 *
 * ardumon_server always creates a UNIX socket file; it runs in text mode by default.  Connect to it either with a
 * serial terminal like minicom that can handle UNIX sockets, or with ardumon_client to run a prepared set of text
 * commands.  Adding the --binary command line option to ardumon_server switches it to binary mode.  Run ardumon_client
 * --binary_demo to connect to it and run a hardcoded set of demo commands.
 *
 * ardumon_client can also connect to a serial port file corresponding to an actual Arduino.  If the Arduino implements
 * any ArduMon text mode CLI it can be exercised with an ArduMon script, see ardumon_script.txt for the syntax and an
 * example.  If the Arduino is running the ArduMon binary demo server then it can be exercised with the --binary_demo
 * option.
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

#include <cstddef>
#include <cstdint>
#include <string>
#include <cstring>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <chrono>
#include <thread>
#include <stdexcept>
#include <exception>
#include <vector>
#include <utility>
#include <fstream>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <signal.h>

#include "arduino_shims.h"
#include "CircBuf.h"

#include <ArduMon.h>

template <size_t in_cap, size_t out_cap> class BufStream : public ArduMonStream {
public :

  CircBuf<in_cap> in;
  CircBuf<out_cap> out;

  BufStream() : in("serial receive"), out("serial send") {}

  int16_t available() { return in.size(); }
  int16_t read() { return in.get(); }
  int16_t peek() { return in.size() > 0 ? static_cast<int16_t>(in.peek()) : -1; }

  int16_t availableForWrite() { return out.free(); }
  uint16_t write(uint8_t val) { out.put(val); return 1; }
};

//emulate the 64 byte Arduino serial input buffer
#define SERIAL_IN_BUF_SZ 64

#ifndef DEMO_CLIENT
//since this demo is single threaded
//and doesn't implement the equivalent of the Arduino serial send interrupt
//use a large output buffer so that command handlers which send a lot, like "help", do not hang
#define SERIAL_OUT_BUF_SZ 2048
#else
#define SERIAL_OUT_BUF_SZ 64
#endif

BufStream<SERIAL_IN_BUF_SZ, SERIAL_OUT_BUF_SZ> demo_stream;  

#define AM_STREAM demo_stream
#include "../demo.h"

#ifdef DEMO_CLIENT
struct termios orig_attribs;
bool read_orig_attribs = false;
#else
int listen_fileno = -1;
#endif
int com_fileno = -1;
std::string com_path;

void usage() {
#ifdef DEMO_CLIENT
  std::string role = "_client";
  std::string args = "[--binary_demo]|[--auto_wait[=ms]] [unix#]";
  std::string sfx = " [< ardumon_script.txt]";
#else
  std::string role = "_server";
  std::string args = "[-b|--binary] ";
  std::string sfx = "";
#endif
  std::cerr << "USAGE: ardumon" << role << " [-v|--verbose] " << args <<  "com_file_or_path" << sfx << "\n"; exit(1);
}

void status() {
  std::cout << demo_stream.in.status() << "\n";
  std::cout << demo_stream.out.status() << "\n";
  std::cout << "ArduMon receive buffer: " << am.get_recv_buf_used() << "/" << am.get_recv_buf_size() << " used\n";
  std::cout << "ArduMon response buffer: " << am.get_send_buf_used() << "/" << am.get_send_buf_size() << " used\n";
  std::string state = "idle";
  if (am.is_receiving()) state = "receiving";
  else if (am.is_handling()) state = "handling";
  std::cout << "ArduMon " << state << (am.is_binary_mode() ? " (binary)" : " (text)") << "\n";
  AM::millis_t rtm = am.get_recv_timeout_ms();
  if (rtm > 0) std::cout << "receive timeout " << rtm << "ms\n";
  std::cout << "com file: " << com_path << "\n" << std::flush;
}

bool exists(const std::string path) { struct stat s; return (stat(path.c_str(), &s) == 0); }

bool is_empty(const std::string path) { struct stat s; return (stat(path.c_str(), &s) == 0) && s.st_size == 0; }

void sleep_ms(const uint32_t ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

void cleanup() {
  if (com_fileno >= 0) {
#ifdef DEMO_CLIENT
    if (read_orig_attribs) {
      std::cout << "restoring attributes on " << com_path << "\n";
      if (tcsetattr(com_fileno, TCSANOW, &orig_attribs) != 0) perror(("error setting attribs on " + com_path).c_str());
    }
#endif
    std::cout << "closing " << com_path << "\n";
    close(com_fileno);
    com_fileno = -1;
  }
#ifndef DEMO_CLIENT
  if (listen_fileno >= 0) { close(listen_fileno); listen_fileno = -1; }
  if (exists(com_path) && is_empty(com_path)) unlink(com_path.c_str());
#endif
}

void terminated() {
  if (std::current_exception()) {
    try { std::rethrow_exception(std::current_exception()); }
    catch (const std::exception& e) { std::cerr << "unhandled exception: " << e.what() << "\n"; }
  }
  cleanup();
  exit(1);
}

void terminate_handler(int s) { terminated(); }

using Script = std::vector<std::pair<std::string, std::string>>;

Script read_script(const uint32_t def_wait_ms, const bool auto_wait) {
  Script ret; std::string ln, def_wait = std::to_string(def_wait_ms);
  while (std::getline(std::cin, ln)) {
    if (ln.empty() || ln[0] == '#') continue; //ignore empty line or comment
    while (ln.back() == '\n' || ln.back() == '\r') ln.pop_back(); //strip \n and \r at line end
    if (ln[0] == '>') ret.emplace_back("recv", ln.substr(1));
    else if (ln[0] == '?') ret.emplace_back("wait", ln.length() > 1 ? ln.substr(1) : def_wait);
    else {
      if (auto_wait && !ret.empty() && ret.back().first == "send") ret.emplace_back("wait", def_wait);
      //if command line starts with a space, remove it
      //this allows e.g. " >foo" to issue command ">foo" that starts with >, i.e. the leading space escapes the >
      //by just stripping a single space we also allow commands that start with whitespace, e.g. "  foo" -> " foo"
      //the ArduMon command interpreter should in turn ignore leading whitespace, but the point may be to test that
      if (ln[0] == ' ') ln.erase(0, 1);
      ret.emplace_back("send", ln);
    }
  }
  return ret;
}

int main(int argc, const char **argv) {

  std::set_terminate(terminated);
  atexit(cleanup);

  struct sigaction sig_handler;
  sig_handler.sa_handler = terminate_handler;
  sigemptyset(&sig_handler.sa_mask);
  sig_handler.sa_flags = 0;
  sigaction(SIGINT, &sig_handler, NULL);

  char buf[2048];

  const char *com_file_or_path = 0;
  bool verbose = false, binary = false, auto_wait = false;
  uint32_t def_wait_ms = 100;
  Script script;

  for (int i = 1; i < argc; i++) {
    if (argv[i][0] == '-') {
      if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0) verbose = true;
#ifdef DEMO_CLIENT
      else if (strcmp(argv[i], "--binary_demo") == 0) binary = true;
      else if (strncmp(argv[i], "--auto_wait", 11) == 0) {
        auto_wait = true;
        if (strlen(argv[i]) > 11 && argv[i][11] == '=') {
          try { def_wait_ms = static_cast<uint32_t>(std::stoul(argv[i] + 12)); }
          catch (const std::invalid_argument &e) { std::cerr << "bad --auto_wait " << e.what() << "\n"; exit(1); }
          catch (const std::out_of_range &e) { std::cerr << "bad --auto_wait " << e.what() << "\n"; exit(1); }
        }
      }
#else
      else if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--binary") == 0) binary = true;
#endif
      else usage();
    } else com_file_or_path = argv[i];
  }

  if (!com_file_or_path) usage();

#ifdef DEMO_CLIENT
  const bool client = true;
  std::string role = binary ? "binary demo client" : "text client";
#else
  const bool client = false;
  std::string role = binary ? "binary server" : "text server";
#endif

  std::cout << "ArduMon " << role << "\n";

  if (!client || binary) setup(); //call Arduino setup() method defined in demo.h

#ifndef DEMO_CLIENT
  std::cout << "registered " << static_cast<int>(am.get_num_cmds()) << "/" << static_cast<int>(am.get_max_num_cmds())
            << " command handlers\n";
  const bool is_socket = true;
  if (binary) {
    std::cout << "switching to binary mode\n";
    am.set_binary_mode(true);
    demo_stream.out.clear(); //the demo> text prompt was already sent; clear it
  } else std::cout << "proceeding in text mode\n";
#else
  const bool is_socket = strncmp("unix#", com_file_or_path, 5) == 0;
  if (is_socket) com_file_or_path += 5;
#endif //DEMO_CLIENT

  if (com_file_or_path[0] != '/') {
    if (!getcwd(buf, sizeof(buf))) { perror("error getting current working directory"); exit(1); }
    com_path += buf; com_path += "/"; com_path += com_file_or_path;
  } else com_path = com_file_or_path;

  if (verbose) status();

  struct sockaddr_un addr;
  if (is_socket) {
    memset(&addr, 0, sizeof(struct sockaddr_un));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, com_path.c_str(), sizeof(addr.sun_path) - 1);
  }

#ifndef DEMO_CLIENT 

  //text or binary server: create com_path as unix socket and listen() on it
  if (exists(com_path) && is_empty(com_path)) {
    std::cout << com_path << " exists and is empty, removing\n";
    unlink(com_path.c_str());
  }
  listen_fileno = socket(AF_UNIX, SOCK_STREAM, 0);
  if (listen_fileno < 0) { perror(("error opeining " + com_path).c_str()); exit(1); }
  if (bind(listen_fileno, (const struct sockaddr *) &addr, sizeof(struct sockaddr_un)) < 0) {
    perror(("error binding " + com_path).c_str()); exit(1);
  }
  if (listen(listen_fileno, 1) < 0) { perror(("error listening on " + com_path).c_str()); exit(1); }

  std::cout << role << ": waiting for connection on " << com_path << "...\n";
  std::cout << "example connection(s):\n";
  if (binary) std::cout << "ardumon_client --binary_demo unix#" << com_path << "\n";
  else {
    std::cout << "minicom -D unix#" << com_path << "\n";
    std::cout << "ardumon_client unix#" << com_path << " < ardumon_script.txt\n";
  }
  std::cout << std::flush;

  com_fileno = accept(listen_fileno, NULL, NULL);
  if (com_fileno < 0) { perror(("error accepting connection on " + com_path).c_str()); exit(1); }
  std::cout << "got connection on " << com_path << "\n";

#else //DEMO_CLIENT

  if (is_socket) { //com_path is a UNIX socket (com_file_or_path started with "unix#")

    com_fileno = socket(AF_UNIX, SOCK_STREAM, 0);
    if (connect(com_fileno, (struct sockaddr *)&addr, sizeof(addr)) == -1) {
      perror(("error connecting to " + com_path).c_str()); exit(1);
    }

  } else { //com_path is a serial port file, not a UNIX socket

    com_fileno = open(com_path.c_str(), O_RDWR | O_NOCTTY);
    if (com_fileno < 0) { perror(("error opening " + com_path).c_str()); exit(1); }

    struct termios t;
    if (tcgetattr(com_fileno, &t) != 0) { perror(("error getting attribs on " + com_path).c_str()); exit(1); }
    memcpy(&orig_attribs, &t, sizeof(struct termios)); read_orig_attribs = true;

    cfmakeraw(&t); //put the serial port in "raw" binary mode

    if (cfsetspeed(&t, B115200) != 0) { perror(("error setting 115200 baud on " + com_path).c_str()); exit(1); }

    //disabling HUPCL (hang up on close) like this should be equivalent to stty -hupcl
    //t.c_cflag &= ~HUPCL;
    //that should prevent the serial port from twiddling DTR on connect and consequently resetting the Arduino
    //however, it is now too late: we already opened the port and the twiddling already occurred!

    if (tcsetattr(com_fileno, TCSANOW, &t) != 0) { perror(("error setting attribs on " + com_path).c_str()); exit(1); }

    //if the Arduino was reset when we opened the serial port we now need to wait a bit
    std::cerr << "delaying 5s...\n";
    sleep_ms(5000);
  }

  if (!binary) {
    std::cout << "reading ArduMon script from stdin... ";
    script = read_script(def_wait_ms, auto_wait);
    std::cout << script.size() << " steps\n";
  }

#endif

  fcntl(com_fileno, F_SETFL, O_NONBLOCK);

  const auto log = [&](const char *what, const uint8_t b) {
    if (verbose) {
      const std::string pad = b < 10 ? "  " : b < 100 ? " " : "";
      std::cout << role << " " << what << " " << pad << +b << " 0x" << AM::to_hex(b >> 4) << AM::to_hex(b);
      if (b >= 32 && b <= 126) std::cout << " '" << b << "'";
      std::cout << "\n" << std::flush;
    }
  };

  auto script_step = script.begin();
  std::string script_response;
  uint64_t wait_start = 0; uint32_t wait_ms = 0;

  while (!demo_done || demo_stream.out.size()) {

    //move any incoming bytes waiting in com_fileno to demo_stream.in
    int nr = read(com_fileno, buf, std::min(sizeof(buf), demo_stream.in.free()));
    if (nr < 0) {
      if (errno == ECONNRESET || errno == ENOTCONN) break;
      else if (errno == EAGAIN) nr = 0; //nonblocking read failed due to nothing available to read
      else { perror(("error reading from " + com_path).c_str()); exit(1); }
    }
    for (size_t i = 0; i < nr; i++) { demo_stream.in.put(buf[i]); if (verbose) log("rcvd", buf[i]); }

    //move any outgoing bytes waiting in demo_stream.out to com_fileno
    size_t ns = std::min(demo_stream.out.size(), sizeof(buf)), nw = 0;
    if (ns > 0) {
      for (size_t i = 0; i < ns; i++) { buf[i] = demo_stream.out.get(); if (verbose) log("sent", buf[i]); }
      while (ns - nw > 0) {
        int ret = write(com_fileno, buf + nw, ns - nw);
        if (ret < 0) {
          if (errno == ECONNRESET || errno == ENOTCONN) break;
          else if (errno == EAGAIN) ret = 0; //nonblocking write failed
          else { perror(("error writing to " + com_path).c_str()); exit(1); }
        }
        nw += ret;
        if (ns - nw > 0) sleep_ms(1);
      }
    }

    if (verbose && (nr > 0 || nw > 0)) status();

    if (!client || binary) loop(); //call Arduino loop() method defined in demo.h
    else { //demo client text script mode
      if (script_step == script.end()) { demo_done = true; break; }
      const size_t step_num = script_step - script.begin();
      if (script_step->first == "send") {
        std::cout << "script step " << step_num << " SEND " << script_step->second << "\n";
        nr = script_step->second.length() + 1;
        for (size_t i = 0; i < nr; i++) demo_stream.out.put((i == nr - 1) ? '\n' : script_step->second[i]);
        ++script_step;
      } else if (script_step->first == "recv") {
        while (demo_stream.in.size()) script_response += demo_stream.in.get();
        auto start = script_response.begin(), end = script_response.end();
        while (start < end && (end = std::find(start, end, '\n')) != script_response.end()) {
          std::string response_line(start, ++end); //pop first response_line from beginning of script_response
          script_response.erase(start, end);
          start = script_response.begin(); end = script_response.end();
          while (response_line.back() == '\n' || response_line.back() == '\r') response_line.pop_back();
          if (script_step == script.end() || script_step->first != "recv") {
            std::cerr << "ERROR: received extra line at script step " << step_num << ":\n"
                      << "received: " << response_line << "\n";
            exit(1);
          }
          std::cout << "script step " << step_num << " RECV " << script_step->second << "\n";
          if (response_line != script_step->second) {
            std::cerr << "ERROR: script recv at step " << step_num << " mismatch:\n"
                      << "expected: " << script_step->second << "\n"
                      << "received: " << response_line << "\n";
            exit(1);
          }
          ++script_step;
        }
      } else if (script_step->first == "wait") {
        const uint64_t now = millis();
        if (!wait_start) {
          std::cout << "script step " << step_num << " WAIT " << script_step->second << "\n";
          wait_start = now;
          try { wait_ms = static_cast<uint32_t>(std::stoul(script_step->second)); }
          catch (const std::invalid_argument &e) { std::cerr << "bad wait_ms " << e.what() << "\n"; exit(1); }
          catch (const std::out_of_range &e) { std::cerr << "bad wait_ms " << e.what() << "\n"; exit(1); }
        } else if (now - wait_start > wait_ms) { demo_stream.in.clear(); wait_start = 0; ++script_step; }
      }
    }
    
    sleep_ms(1);
  }

  exit(0);
}
