/**
 * ArduMon: Yet another Arduino serial command library.
 *
 * See https://github.com/martyvona/ArduMon/blob/main/README.md
 *
 * This "native" demo driver compiles and runs on an OS X or Linux host.
 *
 * When compiled as a server (BINARY_CLIENT not defined) this code can run in binary or text mode, controlled by a
 * command line option, and a UNIX socket file is created to which clients can connect.  In text mode the client would
 * be a standard serial terminal program like minicom.  In binary mode the client would be another invocation of this
 * demo but compiled as a client (BINARY_CLIENT defined).  It is also possible to run this demo as a binary client and
 * connect to a serial port file corresponding to a physical Arduino running the binary demo server.
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

#ifndef BINARY_CLIENT
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

#ifdef BINARY_CLIENT
struct termios orig_attribs;
bool read_orig_attribs = false;
#else
int listen_fileno = -1;
#endif
int com_fileno = -1;
bool script_mode = false;
std::string com_path;

void usage() {
#ifdef BINARY_CLIENT
  std::string role = "_client";
  std::string role_args = "[unix#]";
#else
  std::string role = "";
  std::string role_args = "[-b|--binary]|[-s|--script] [-q|--quiet] ";
#endif
  std::cerr << "USAGE: demo" << role << "_native [-v|--verbose] " << role_args <<  "<com_file_or_path>\n"; exit(1);
}

void status() {
  std::cout << demo_stream.in.status() << "\n";
  std::cout << demo_stream.out.status() << "\n";
  std::cout << "command receive buffer: " << am.get_recv_buf_used() << "/" << am.get_recv_buf_size() << " used\n";
  std::cout << "command response buffer: " << am.get_send_buf_used() << "/" << am.get_send_buf_size() << " used\n";
  std::string state = "idle";
  if (am.is_receiving()) state = "receiving";
  else if (am.is_handling()) state = "handling";
  std::cout << "command interpreter " << state << (am.is_binary_mode() ? " (binary)" : " (text)") << "\n";
  AM::millis_t rtm = am.get_recv_timeout_ms();
  if (rtm > 0) std::cout << "command receive timeout " << rtm << "ms\n";
  std::cout << "com file: " << com_path << "\n" << std::flush;
}

bool exists(const std::string path) { struct stat s; return (stat(path.c_str(), &s) == 0); }

bool is_empty(const std::string path) { struct stat s; return (stat(path.c_str(), &s) == 0) && s.st_size == 0; }

void sleep_ms(const uint32_t ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

void cleanup() {
  if (com_fileno >= 0) {
#ifdef BINARY_CLIENT
    if (read_orig_attribs) {
      std::cout << "restoring attributes on " << com_path << "\n";
      if (tcsetattr(com_fileno, TCSANOW, &orig_attribs) != 0) perror(("error setting attribs on " + com_path).c_str());
    }
#endif
    std::cout << "closing " << com_path << "\n";
    close(com_fileno);
    com_fileno = -1;
  }
#ifndef BINARY_CLIENT
  if (listen_fileno >= 0) { close(listen_fileno); listen_fileno = -1; }
  if (!script_mode && exists(com_path) && is_empty(com_path)) unlink(com_path.c_str());
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

Script read_script(const std::string& file_path) {
  Script records; std::string ln;
  std::ifstream file(file_path);
  if (!file.is_open()) { std::cerr << "ERROR: unable to open script file: " << file_path << "\n"; exit(1); }
  while (std::getline(file, ln)) {
    std::string::iterator it;
    if (ln.empty() || std::all_of(ln.begin(), ln.end(), isspace)) continue; //ignore empty line
    if ((it = std::find_if_not(ln.begin(), ln.end(), isspace)) != ln.end() && *it == '#') continue; //ignore comment
    while (ln.back() == '\n' || ln.back() == '\r') ln.pop_back(); //remove \n and \r
    if (ln[0] != '>') records.emplace_back("send", ln);
    else records.emplace_back("recv", ln.substr(1));
  }
  file.close();
  return records;
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
  bool verbose = false, binary = false;
  Script script;

  for (int i = 1; i < argc; i++) {
    if (argv[i][0] == '-') {
#ifndef BINARY_CLIENT
      if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--binary") == 0) binary = true; else
      if (strcmp(argv[i], "-q") == 0 || strcmp(argv[i], "--quiet") == 0) demo_quiet = true; else
      if (strcmp(argv[i], "-s") == 0 || strcmp(argv[i], "--script") == 0) script_mode = demo_quiet = true; else
#endif
      if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0) verbose = true;
      else usage();
    } else com_file_or_path = argv[i];
  }

  if (!com_file_or_path) usage();

#ifdef BINARY_CLIENT
  binary = true;
  std::string role = "binary client";
#else
  std::string role = (binary ? "binary" : "text");
  role += " server";
#endif

  std::cout << "ArduMon " << role << "\n";

  setup(); //call Arduino setup() method defined in demo.h

#ifndef BINARY_CLIENT
  std::cout << "registered " << static_cast<int>(am.get_num_cmds()) << "/" << static_cast<int>(am.get_max_num_cmds())
            << " command handlers\n";
  bool is_socket = true;
  if (binary) {
    if (script_mode) { std::cerr << "ERROR: --binary and --script are mutually exclusive\n"; exit(1); }
    std::cout << "switching to binary mode\n";
    am.set_binary_mode(true);
    demo_stream.out.clear(); //the demo> text prompt was already sent; clear it
  } else {
    std::cout << "proceeding in text mode\n";
    if (script_mode) is_socket = false;
  }
#else
  const bool is_socket = strncmp("unix#", com_file_or_path, 5) == 0;
  if (is_socket) com_file_or_path += 5;
#endif //BINARY_CLIENT

  if (com_file_or_path[0] != '/') {
    if (!getcwd(buf, sizeof(buf))) { perror("error getting current working directory"); exit(1); }
    com_path += buf; com_path += "/"; com_path += com_file_or_path;
  } else com_path = com_file_or_path;

  status();

  struct sockaddr_un addr;
  if (is_socket) {
    memset(&addr, 0, sizeof(struct sockaddr_un));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, com_path.c_str(), sizeof(addr.sun_path) - 1);
  }

#ifndef BINARY_CLIENT 

  if (script_mode) {

    std::cout << "reading script from " << com_path << "...\n";
    script = read_script(com_path);
    std::cout << "read " << script.size() << " script steps\n";

  } else { //text or binary server: create com_path as unix socket and listen() on it
      
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

    std::cout << "demo server: waiting for connection on " << com_path << "...\n";
    std::cout << "example connection:\n";
    if (binary) std::cout << "demo_client_native unix#" << com_path << "\n";
    else std::cout << "minicom -D unix#" << com_path << "\n";
    std::cout << std::flush;

    com_fileno = accept(listen_fileno, NULL, NULL);
    if (com_fileno < 0) { perror(("error accepting connection on " + com_path).c_str()); exit(1); }
    std::cout << "got connection on " << com_path << "\n";
  }

#else //binary client

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

  while (!demo_done && (!script_mode || script_step != script.end())) {

    int nr = 0, nw = 0;
    const size_t step_num = script_step - script.begin();

    if (!script_mode) {
      nr = read(com_fileno, buf, std::min(sizeof(buf), demo_stream.in.free()));
      if (nr < 0) {
        if (errno == ECONNRESET || errno == ENOTCONN) break;
        else if (errno == EAGAIN) nr = 0; //nonblocking read failed due to nothing available to read
        else { perror(("error reading from " + com_path).c_str()); exit(1); }
      }
    } else if (script_step->first == "send") {
      nr = script_step->second.length() + 1;
      if (nr > sizeof(buf)) {
        std::cerr << "ERROR: script send at step " << step_num << " is " << nr << " > " << sizeof(buf) << " chars\n";
        exit(1);
      }
      for (size_t i = 0; i < nr; i++) buf[i] = (i == nr - 1) ? '\n' : script_step->second[i];
      std::cout << "script step " << step_num << " SEND " << script_step->second << "\n";
      ++script_step;
    }

    for (size_t i = 0; i < nr; i++) { demo_stream.in.put(buf[i]); if (verbose) log("rcvd", buf[i]); }
    
    loop(); //call Arduino loop() method defined in demo.h
    
    size_t ns = std::min(demo_stream.out.size(), sizeof(buf));
    if (ns > sizeof(buf)) {
      std::cerr << "ERROR: " << role << " sent " << ns << " > " << sizeof(buf) << " chars\n";
      exit(1);
    }

    if (ns > 0) {
      for (size_t i = 0; i < ns; i++) { buf[i] = demo_stream.out.get(); if (verbose) log("sent", buf[i]); }
      if (!script_mode) {
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
      } else if (script_step->first == "recv") {
        nw = ns;
        script_response.append(buf, ns);
        auto start = script_response.begin(), end = script_response.end();
        while (start < end && (end = std::find(start, end, '\n')) != script_response.end()) {
          std::string response_line(start, ++end); //pop first response_line from beginning of script_response
          script_response.erase(start, end);
          start = script_response.begin(); end = script_response.end();
          while (response_line.back() == '\n' || response_line.back() == '\r') response_line.pop_back();
          if (script_step->first != "recv") {
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
      }
    }

    if (verbose && (nr > 0 || nw > 0)) status();

    sleep_ms(1);
  }

  exit(0);
}
