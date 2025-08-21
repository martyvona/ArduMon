#ifndef CIRC_BUF_H
#define CIRC_BUF_H

/**
 * ArduMon: Yet another Arduino serial command library.
 *
 * See https://github.com/martyvona/ArduMon/blob/main/README.md
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
