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
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <signal.h>

#include "circ_buf.h"
#include "arduino_shims.h"

bool quit;
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
//demo.ino defines the global ArduMon isntance am

#ifndef BINARY_CLIENT
int listen_fileno = -1;
#endif
int com_fileno = -1;
std::string com_path;

bool quit_cmd(AM &am) { quit = true; return true; }

void usage() {
#ifdef BINARY_CLIENT
  std::string role = "_client";
  std::string role_args = "[unix#]";
#else
  std::string role = "";
  std::string role_args = "[-b|binary] ";
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

void sleep_ms(const uint32_t ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

void cleanup() {
  if (com_fileno >= 0) {
    std::cout << "closing " << com_path << "\n";
    close(com_fileno);
    com_fileno = -1;
  }
#ifndef BINARY_CLIENT
  if (listen_fileno >= 0) { close(listen_fileno); listen_fileno = -1; }
  if (exists(com_path)) unlink(com_path.c_str());
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
  for (int i = 1; i < argc; i++) {
    if (argv[i][0] == '-') {
#ifndef BINARY_CLIENT
      if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--binary") == 0) binary = true; else
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

  std::cout << "registering quit command\n";
  if (!am.add_cmd(quit_cmd, F("quit"), F("quit"))) show_error(am);

  std::cout << "registered " << static_cast<int>(am.get_num_cmds()) << "/" << static_cast<int>(am.get_max_num_cmds())
            << " command handlers\n";

#ifndef BINARY_CLIENT
  if (binary) {
    std::cout << "switching to binary mode\n";
    am.set_binary_mode(true);
    demo_stream.out.clear(); //the demo> text prompt was already sent; clear it
  } else std::cout << "proceeding in text mode\n";
#endif

#ifdef BINARY_CLIENT
  const bool is_socket = strncmp("unix#", com_file_or_path, 5) == 0;
  if (is_socket) com_file_or_path += 5;
#else
  const bool is_socket = true;
#endif

  if (com_file_or_path[0] != '/') {
    if (!getcwd(buf, sizeof(buf))) { perror("error getting current working directory"); exit(1); }
    com_path += buf;
    com_path += "/";
    com_path += com_file_or_path;
  }

  status();

  struct sockaddr_un addr;
  if (is_socket) {
    memset(&addr, 0, sizeof(struct sockaddr_un));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, com_path.c_str(), sizeof(addr.sun_path) - 1);
  }

#ifndef BINARY_CLIENT 

  //text or binary server: create com_path as unix socket and listen() on it

  if (exists(com_path)) {
    std::cout << com_path << " exists, removing\n";
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

#else //binary client

  if (is_socket) { //com_path is a UNIX socket (command line option started with "unix#")
    com_fileno = socket(AF_UNIX, SOCK_STREAM, 0);
    if (connect(com_fileno, (struct sockaddr *)&addr, sizeof(addr)) == -1) {
      perror(("error connecting to " + com_path).c_str()); exit(1);
    }
  } else { //com_path is a serial port file, not a UNIX socket
    com_fileno = open(com_path.c_str(), O_RDWR | O_NOCTTY);
    if (com_fileno < 0) { perror(("error opening " + com_path).c_str()); exit(1); }
    struct termios t;
    if (tcgetattr(com_fileno, &t) != 0) { perror(("error getting attribs on " + com_path).c_str()); exit(1); }
    t.c_iflag &= ~(IXON | IXOFF | IXANY); //disable software flow control (XON/XOFF)
    t.c_cflag &= ~CRTSCTS; // disable hardware flow control (RTS/CTS)
    if (tcsetattr(com_fileno, TCSANOW, &t) != 0) { perror(("error setting attribs on " + com_path).c_str()); exit(1); }
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

  quit = false;
  while (!quit) {
    
    int nr = read(com_fileno, buf, std::min(sizeof(buf), demo_stream.in.free()));
    if (nr < 0) {
      if (errno == ECONNRESET || errno == ENOTCONN) break;
      else if (errno == EAGAIN) nr = 0; //nonblocking read failed due to nothing available to read
      else { perror(("error reading from " + com_path).c_str()); exit(1); }
    }

    for (size_t i = 0; i < nr; i++) { demo_stream.in.put(buf[i]); if (verbose) log("rcvd", buf[i]); }
    
    loop(); //call Arduino loop() method defined in demo.h
    
    AM::Error e = am.get_err();
    if (e != AM::Error::NONE) { std::cerr << "ArduMon error: " << am.err_msg(e) << "\n"; am.clear_err(); }
    
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

    sleep_ms(1);
  }

#ifndef BINARY_CLIENT
  if (!quit) std::cout << "lost connection on " << com_path << "\n";
#endif

  std::cout << "closing " << com_path << "\n";
  close(com_fileno); com_fileno = -1;
#ifndef BINARY_CLIENT
  close(listen_fileno); listen_fileno = -1;
  unlink(com_path.c_str());
#endif

  exit(0);
}
