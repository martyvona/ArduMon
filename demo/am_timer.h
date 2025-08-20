#ifndef AM_TIMER_H
#define AM_TIMER_H

//A countdown timer which demonstrates several ways ArduMon commands can send data over time.
//
//In synchronous text mode the start_timer command does not end until the timer reaches zero, or the user stops it by
//hitting any key.  The Arduino loop() function is not blocked.  The remaining time is periodically reported to the user
//using VT100 control codes to create a rolling counter that overwrites itself in the terminal without scrolling.
//
//In asynchronous text mode the start_timer command ends quickly, but the timer keeps running.  The time can be
//requested later with the get_time command, and the timer can be stopped with stop_timer.
//
//Synchronous and asynchronous binary modes are similar to the corresponding text modes, except the time reports are
//sent as binary packets instead of text and VT100 control codes.  These packets can either start with a configurable
//command code or not.  This demonstrates that the receiver, which can be a separate instance of ArduMon running on
//another processor, can receive response packets either as incoming commands or as generic packets using the ArduMon
//set_universal_handler() facility.
template <typename AM> class Timer {
public:

  bool start(AM *am) {

    running = false;
    if (!am->recv()) return false; //skip over command token or code

    uint8_t h, m, s;
    if (!am->recv(&h) || !am->recv(&m) || !am->recv(&s)) return false;
    total_ms = remaining_ms = (h * 3600 + m * 60 + s) * 1000ul;

    accel = 1;
    if (am->argc() > 4 && !am->recv(&accel)) return false;

    async = false;
    if (am->argc() > 5 && !am->recv(&async)) return false;

    async_cmd_code = -1;
    if (async && am->argc() > 6 && !am->recv(&async_cmd_code)) return false;

    sync_throttle_ms = 100;
    if (!async && am->argc() > 6 && !am->recv(&sync_throttle_ms)) return false;

    if (am->is_txt_mode()) {
      am->send_raw(F("counting down from "));
      am->send_raw(h);
      am->send_raw(':');
      am->send_raw(m, 2|AM::FMT_PAD_ZERO);
      am->send_raw(':');
      am->send_raw(s, 2|AM::FMT_PAD_ZERO);
      am->send_raw(F(", accel="));
      am->send_raw(accel);
      am->send_raw(async ? F(", async=true") : F(", hit any key to cancel..."));
      am->send_CRLF();
      if (!async) {
        am->send_raw(AM::VT100_CURSOR_HIDDEN);
        //am->vt100_set_attr(AM::VT100_ATTR_REVERSE);
        //am->vt100_set_attr(AM::VT100_ATTR_BLINK);
        //am->vt100_set_attr(AM::VT100_ATTR_BRIGHT);
        //am->vt100_set_attr(AM::VT100_ATTR_UNDERSCORE);
        am->vt100_set_color(AM::VT100_FOREGROUND, AM::VT100_CYAN);
      }
    }

    start_ms = millis();
    elapsed_ms = 0;
    running = true;

    if (async) am->end_cmd(); else get_time(am, start_ms);

    return true;
  }

  bool stop(AM *am) { running = false; return true; }

  bool get_time(AM *am, const uint64_t now) {
    last_send_ms = now;
    if (am->is_txt_mode()) {
      const uint8_t h = remaining_ms / (3600 * 1000ul);
      const uint8_t m = (remaining_ms - h * 3600 * 1000ul) / (60 * 1000ul);
      const uint8_t s = (remaining_ms - (h * 3600 + m * 60) * 1000ul) / 1000ul;
      const uint16_t ms = remaining_ms - (h * 3600 + m * 60 + s) * 1000ul;
      return (async || am->send_raw(AM::VT100_CLEAR_LINE)) &&
        am->send_raw(h, 3|AM::FMT_PAD_ZERO) && am->send_raw(':') &&
        am->send_raw(m, 2|AM::FMT_PAD_ZERO) && am->send_raw(':') &&
        am->send_raw(s, 2|AM::FMT_PAD_ZERO) && am->send_raw('.') &&
        am->send_raw(ms, 3|AM::FMT_PAD_ZERO) &&
        ((!async && running) || am->send_CRLF()) &&
        (async || running || (am->send_raw(AM::VT100_CURSOR_VISIBLE) && am->vt100_set_attr(AM::VT100_ATTR_RESET)));
    } else {
      //32 bits is sufficient for up to about 49 days, but we are limited to about 11 days since h,m,s <= 255
      return (async_cmd_code < 0 || async_cmd_code > 255 || am->send(static_cast<uint8_t>(async_cmd_code))) &&
        am->send(static_cast<uint32_t>(start_ms)) && am->send(static_cast<uint32_t>(total_ms)) &&
        am->send(static_cast<uint32_t>(elapsed_ms)) && am->send(static_cast<uint32_t>(remaining_ms)) &&
        am->send_packet();
    }
  }

  bool get_time(AM *am) { return get_time(am, millis()); }

  bool is_running() { return running; };

  bool tick(AM *am) {
    if (!running) return false;
    const uint64_t now = millis();
    elapsed_ms = (millis() - start_ms) * accel;
    remaining_ms = elapsed_ms <= total_ms ? total_ms - elapsed_ms : 0;
    if (remaining_ms == 0 || (!async && am->is_txt_mode() && am->get_key() != 0)) running = false;
    if (!async) {
      if (now - last_send_ms >= sync_throttle_ms || !running) get_time(am, now);
      if (!running) am->end_cmd();
    }
    return running;
  }

private:
  bool running, async = true;
  float accel;
  uint16_t async_cmd_code;
  uint64_t start_ms, total_ms, elapsed_ms, remaining_ms, sync_throttle_ms, last_send_ms;
};

#endif //AM_TIMER_H
