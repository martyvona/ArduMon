#include <stddef.h>
#include <stdint.h>

#include <ArduMonBase.h>

class Stream : public ArduMonBase::Stream {
public :
  int16_t available() { return 0; }
  int16_t read() { return 0; }
  int16_t availableForWrite() { return 0; }
  uint16_t write(uint8_t val) { return 0; }
};

Stream demo_stream;  

#include "../demo.ino"

int main(int argc, const char **argv) {
  add_cmds();
}
