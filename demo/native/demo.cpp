#include <stddef.h>
#include <stdint.h>
#include <stdexcept>
#include <string>

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

private:
  const std::string name;
  uint8_t buf[capacity];
  size_t write_idx = 0, read_idx = capacity;
};

#include <ArduMonBase.h>

template <size_t in_cap, size_t out_cap> class BufStream : public ArduMonBase::Stream {
public :
  BufStream() : in("serial receive"), out("serial send") {}
  int16_t available() { return in.size(); }
  int16_t read() { return in.get(); }
  int16_t availableForWrite() { return out.free(); }
  uint16_t write(uint8_t val) { out.put(val); return 1; }
private:
  CircBuf<in_cap> in;
  CircBuf<out_cap> out;
};

#define IN_CAP 64
#define OUT_CAP 64

BufStream<IN_CAP, OUT_CAP> demo_stream;  

#include "../demo.ino"

int main(int argc, const char **argv) {
  add_cmds();
}
