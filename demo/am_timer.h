#ifndef AM_TIMER_H
#define AM_TIMER_H

#ifndef ARDUINO
#include <time.h>
#endif

//ArduMon demonstration of a simple timer
template <typename AM> class Timer {
public:

  Timer(AM *_am) : am(_am) {}

  bool start() {

    running = false;
    if (!am->recv()) return false; //skip over command token or code

    uint8_t h, m, s;
    if (!am->recv(&h) || !am->recv(&m) || !am->recv(&s)) return false;
    total_ms = (h * 3600 + m * 60 + s) * 1000ul;

    accel = 1;
    if (am->argc() > 4 && !am->recv(&accel)) return false;

    if (am->is_txt_mode()) {
      am->send_raw(F("counting down from "));
      am->send_raw(h);
      am->send_raw(':');
      am->send_raw(m, 2|AM::FMT_PAD_ZERO);
      am->send_raw(':');
      am->send_raw(s, 2|AM::FMT_PAD_ZERO);
      am->send_raw(F(", accel="));
      am->send_raw(accel);
      am->send_raw(F(", hit any key to cancel..."));
      am->send_CRLF();
      am->send_raw(AM::VT100_CURSOR_HIDDEN);
      //am->vt100_set_attr(AM::VT100_ATTR_REVERSE);
      //am->vt100_set_attr(AM::VT100_ATTR_BLINK);
      //am->vt100_set_attr(AM::VT100_ATTR_BRIGHT);
      //am->vt100_set_attr(AM::VT100_ATTR_UNDERSCORE);
      am->vt100_set_color(AM::VT100_FOREGROUND, AM::VT100_CYAN);
    }

    start_ms = millis();
    running = true;
    return true;
  }

  bool is_running() { return running; };

  bool tick() {
    if (!running) return false;
    uint8_t h, m, s; uint16_t ms = 0;
    const unsigned long elapsed_ms = (millis() - start_ms) * accel, remaining_ms = total_ms - elapsed_ms;
    if (elapsed_ms >= total_ms || (am->is_txt_mode() && am->get_key() != 0)) {
      h = m = s = 0; ms = 0;
      running = false;
    } else {
      h = remaining_ms / (3600 * 1000ul);
      m = (remaining_ms - h * 3600 * 1000ul) / (60 * 1000ul);
      s = (remaining_ms - (h * 3600 + m * 60) * 1000ul) / 1000ul;
      ms = remaining_ms - (h * 3600 + m * 60 + s) * 1000ul;
    }
    if (am->is_binary_mode()) {
      //32 bits is sufficient for up to about 49 days, but we are limited to about 11 days since h,m,s <= 255
      am->send(static_cast<uint32_t>(elapsed_ms)); am->send(static_cast<uint32_t>(remaining_ms));
      am->send_packet();
    } else {
      am->send_raw(AM::VT100_CLEAR_LINE);
      am->send_raw(h, 3|AM::FMT_PAD_ZERO);
      am->send_raw(':');
      am->send_raw(m, 2|AM::FMT_PAD_ZERO);
      am->send_raw(':');
      am->send_raw(s, 2|AM::FMT_PAD_ZERO);
      am->send_raw('.');
      am->send_raw(ms, 3|AM::FMT_PAD_ZERO);
    }
    if (!running) {
      if (am->is_txt_mode()) {
        am->send_raw(AM::VT100_CURSOR_VISIBLE);
        am->vt100_set_attr(AM::VT100_ATTR_RESET);
      }
      am->end_cmd();
    }
    return running;
  }

private:

  AM * const am;
  bool running;
  float accel;
  unsigned long total_ms, start_ms;

#ifndef ARDUINO
  unsigned long millis() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint64_t now_ms = ts.tv_sec * 1000ul + ts.tv_nsec / 1000000ul;
    static uint64_t start_ms = now_ms;
    return now_ms - start_ms;
  }
#endif
};

#endif //AM_TIMER_H
