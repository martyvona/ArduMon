/**
 * ArduMon: Yet another Arduino serial command library.
 *
 * See https://github.com/martyvona/ArduMon/blob/main/README.md
 *
 * This "native" demo driver compiles and runs entirely on an OS X or Linux host, no Arduino involved.  It creates and
 * listens on a UNIX socket at a given path.  Connect to that socket with a capable serial terminal program, e.g.
 *
 * minicom -D unix#SOCKET_PATH
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
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <signal.h>

#include <ArduMon.h>

template <size_t capacity> class CircBuf {
public:

  CircBuf(const char *n) : name(n) {}

  void put(uint8_t val) {
    if (read_idx == write_idx) throw std::runtime_error(name + " overflow");
    if (read_idx == capacity) read_idx = write_idx = 0;
    buf[write_idx++] = val;
    if (write_idx == capacity) write_idx = 0;
  }

  uint8_t get() {
    if (read_idx == capacity) throw std::runtime_error(name + " underflow");
    uint8_t ret = buf[read_idx++];
    if (read_idx == capacity) read_idx = 0;
    if (read_idx == write_idx) { read_idx = capacity; write_idx = 0; }
    return ret;
  }

  size_t size() {
    if (read_idx == capacity) return 0;
    if (read_idx == write_idx) return capacity;
    if (write_idx > read_idx) return write_idx - read_idx;
    return capacity - (read_idx - write_idx);
  }

  size_t free() { return capacity - size(); }

  std::string status() { return name + ": " + std::to_string(size()) + "/" + std::to_string(capacity) + " used"; }

private:
  const std::string name;
  uint8_t buf[capacity];
  size_t write_idx = 0, read_idx = capacity;
};

template <size_t in_cap, size_t out_cap> class BufStream : public ArduMonStream {
public :

  CircBuf<in_cap> in;
  CircBuf<out_cap> out;

  BufStream() : in("serial receive"), out("serial send") {}

  int16_t available() { return in.size(); }
  int16_t read() { return in.get(); }

  int16_t availableForWrite() { return out.free(); }
  uint16_t write(uint8_t val) { out.put(val); return 1; }
};

//emulate the 64 byte Arduino serial input buffer
#define SERIAL_IN_BUF_SZ 64

//since this demo is single threaded
//and doesn't implement the equivalent of the Arduino serial send interrupt
//use a large output buffer so that command handlers which send a lot, like "help", do not hang
#define SERIAL_OUT_BUF_SZ 2048

BufStream<SERIAL_IN_BUF_SZ, SERIAL_OUT_BUF_SZ> demo_stream;  

#define STREAM demo_stream
#include "../demo.ino"
//demo.ino defines the global ArduMon isntance am

int listen_socket;
std::string socket_path;
bool quit;

bool quit_cmd(AM *am) { quit = true; return true; }

void usage() { std::cerr << "USAGE: demo [-b|--binary] <unix_socket_filename>\n"; exit(1); }

void status() {
  std::cout << demo_stream.in.status() << "\n";
  std::cout << demo_stream.out.status() << "\n";
  std::cout << "command receive buffer: " << am.get_recv_buf_used() << "/" << am.get_recv_buf_size() << " used\n";
  std::cout << "command response buffer: " << am.get_send_buf_used() << "/" << am.get_send_buf_size() << " used\n";
  std::string state = "idle";
  if (am.is_receiving()) state = "receiving";
  else if (am.is_handling()) state = "handling";
  if (am.is_binary_mode()) state += " (binary)";
  std::cout << "command interpreter " << state << "\n";
  AM::millis_t rtm = am.get_recv_timeout_ms();
  if (rtm > 0) std::cout << "command receive timeout " << rtm << "ms\n";
  std::cout << "UNIX socket: " << socket_path << "\n";
  std::cout << std::flush;
}

bool exists(const std::string path) { struct stat s; return (stat(path.c_str(), &s) == 0); }

void sleep_ms(const uint32_t ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

void terminated() {
  if (std::current_exception()) {
    try { std::rethrow_exception(std::current_exception()); }
    catch (const std::exception& e) { std::cerr << "unhandled exception: " << e.what() << "\n"; }
  }
  if (exists(socket_path)) {
    std::cout << "closing UNIX socket\n";
    close(listen_socket);
    unlink(socket_path.c_str());
  }
  exit(1);
}

void terminate_handler(int s) { terminated(); }

int main(int argc, const char **argv) {

  if (argc < 2) usage();

  std::set_terminate(terminated);

  struct sigaction sig_handler;
  sig_handler.sa_handler = terminate_handler;
  sigemptyset(&sig_handler.sa_mask);
  sig_handler.sa_flags = 0;
  sigaction(SIGINT, &sig_handler, NULL);

  const char *socket_filename;
  bool binary = false, verbose = false;
  for (int i = 1; i < argc; i++) {
    if (argv[i][0] == '-') {
      if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--binary") == 0) binary = true;
      else if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0) verbose = true;
      else usage();
    } else socket_filename = argv[i];
  }

  if (!socket_filename) usage();

  std::cout << "adding demo commands\n";
  setup();
  if (!am.add_cmd(quit_cmd, F("quit"), F("quit"))) show_error(&am);
  std::cout << "added "
            << static_cast<int>(am.get_num_cmds()) << "/" << static_cast<int>(am.get_max_num_cmds()) << " commands\n";

  if (binary) {
    std::cout << "switching to binary mode\n";
    am.set_binary_mode(true);
  } else std::cout << "proceeding in text mode\n";

  char buf[2048];
  if (!getcwd(buf, sizeof(buf))) { perror("error getting current working directory"); exit(1); }

  socket_path += buf;
  socket_path += "/";
  socket_path += socket_filename;

  status();

  if (exists(socket_path)) {
    std::cout << "UNIX socket path exists, removing\n";
    unlink(socket_path.c_str());
  }

  listen_socket = socket(AF_UNIX, SOCK_STREAM, 0);
  if (listen_socket < 0) { perror("error opeining UNIX socket"); exit(1); }

  struct sockaddr_un addr;
  memset(&addr, 0, sizeof(struct sockaddr_un));
  addr.sun_family = AF_UNIX;
  strncpy(addr.sun_path, socket_path.c_str(), sizeof(addr.sun_path) - 1);
  if (bind(listen_socket, (const struct sockaddr *) &addr, sizeof(struct sockaddr_un)) < 0) {
    perror("error binding UNIX socket"); exit(1);
  }

  if (listen(listen_socket, 1) < 0) { perror("error listening on UNIX socket"); exit(1); }

  std::cout << "demo server: waiting for connection on UNIX socket " << socket_path << "...\n";
  std::cout << "example connection:\n";
  std::cout << "minicom -D unix#" << socket_path << "\n";
  std::cout << std::flush;

  int data_socket = accept(listen_socket, NULL, NULL);
  if (data_socket < 0) { perror("error accepting connection on UNIX socket"); exit(1); }
  std::cout << "got connection on UNIX socket " << socket_path << "\n";

  fcntl(data_socket, F_SETFL, O_NONBLOCK);

  std::cout << "resetting command interpreter\n";
  am.reset();
  
  const uint32_t STATUS_MS = 2000;
  uint32_t ms_to_status = STATUS_MS;
  quit = false;
  while (!quit) {
    
    int ret = read(data_socket, buf, std::min(sizeof(buf), demo_stream.in.free()));
    if (ret < 0) {
      if (errno == ECONNRESET || errno == ENOTCONN) break;
      else if (errno == EAGAIN) ret = 0; //nonblocking read failed due to nothing available to read
      else { perror("error reading from demo client"); exit(1); }
    }
    if (verbose && ret > 0) std::cout << "read " << ret << " bytes from demo client\n" << std::flush;

    for (size_t i = 0; i < ret; i++) demo_stream.in.put(buf[i]);
    
    loop();
    
    AM::Error e = am.get_err();
    if (e != AM::Error::NONE) {
      std::cerr << "command interpreter error: " << am.err_msg(e) << "\n";
      am.clear_err();
    }
    
    size_t ns = std::min(demo_stream.out.size(), sizeof(buf));
    if (ns > 0) {
      for (size_t i = 0; i < ns; i++) buf[i] = demo_stream.out.get();
      int nw = 0;
      while (ns - nw > 0) {
        ret = write(data_socket, buf + nw, ns - nw);
        if (ret < 0) {
          if (errno == ECONNRESET || errno == ENOTCONN) break;
          else if (errno == EAGAIN) ret = 0; //nonblocking write failed
          else { perror("error writing to demo client"); exit(1); }
        }
        if (verbose && ret > 0) std::cout << "wrote " << ret << " bytes to demo client\n" << std::flush;
        nw += ret;
        if (ns - nw > 0) {
          sleep_ms(1);
          if (verbose && --ms_to_status == 0) { status(); ms_to_status = STATUS_MS; }
        }
      }
    }
    
    sleep_ms(1);
    if (verbose && --ms_to_status == 0) { status(); ms_to_status = STATUS_MS; }
  }

  if (!quit) std::cout << "lost connection on UNIX socket\n";

  std::cout << "closing UNIX socket\n";
  close(listen_socket);
  unlink(socket_path.c_str());

  exit(0);
}
