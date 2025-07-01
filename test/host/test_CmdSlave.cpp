//g++ -o test_CmdSlave test_CmdSlave.cpp --std=c++11

#include <iostream>
#include <sys/time.h>
#include <stdio.h>

#include "CmdSlave.h"

extern "C" {

  uint32_t millis() {
    struct timeval tv;
    gettimeofday(&tv,NULL);
    uint64_t now_ms = tv.tv_sec * 1000 + tv.tv_usec / 1000;
    static uint64_t start_ms = now_ms;
    return now_ms - start_ms;
  }

  void delayMicroseconds(uint16_t us) {
    struct timeval tv;
    gettimeofday(&tv,NULL);
    uint64_t start_us = tv.tv_sec * 1000000 + tv.tv_usec, now_us;
    do {
      gettimeofday(&tv,NULL);
      now_us = tv.tv_sec * 1000000 + tv.tv_usec;
    } while (now_us - start_us < us);
  }
}

int main(int argc, const char **argv) {
  std::cout << millis() << "\n";
  for (int i = 0; i < 20; i++) delayMicroseconds(100);
  std::cout << millis() << "\n";
  char buf[100];
  std::cout << dtostrf(1.23456789e30, 0, 4, buf) << "\n";
  std::cout << dtostre(1.23456789e30, buf, 4, 0) << "\n";
}
