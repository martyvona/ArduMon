//g++ -o test_CmdSlave test_CmdSlave.cpp --std=c++11

#include <iostream>

#include "CmdSlave.h"

#define MAX_CMDS 64
#define RECV_BUF_SZ 256
#define SEND_BUF_SZ 256

CmdSlave<MAX_CMDS,RECV_BUF_SZ,SEND_BUF_SZ> cmd(0, false);

int main(int argc, const char **argv) {

  cmd.update();

  cmd.recv();
  char c; cmd.recv(&c);

  const char *s; cmd.recv(&s);

  uint8_t   u8; cmd.recv(&u8);
  int8_t    s8; cmd.recv(&s8);
  uint16_t u16; cmd.recv(&u16);
  int16_t  s16; cmd.recv(&s16);
  uint32_t u32; cmd.recv(&u32);
  int32_t  s32; cmd.recv(&s32);
  uint64_t u64; cmd.recv(&u64);
  int64_t  s64; cmd.recv(&s64);
  float      f; cmd.recv(&f);
  double     d; cmd.recv(&d);

  cmd.send(c); cmd.send_raw(c);
  cmd.send(s); cmd.send_raw(s);

  cmd.send(u8);
  cmd.send(s8);
  cmd.send(u16);
  cmd.send(s16);
  cmd.send(u32);
  cmd.send(s32);
  cmd.send(u64);
  cmd.send(s64);

  cmd.send(f);
  cmd.send(d);
}
