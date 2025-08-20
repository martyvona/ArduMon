#ifndef CIRC_BUF_H
#define CIRC_BUF_H

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

  uint8_t peek() {
    if (read_idx == capacity) throw std::runtime_error(name + " underflow");
    return buf[read_idx];
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

#endif //CIRC_BUF_H
