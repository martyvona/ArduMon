#ifndef AM_TIMER_H
#define AM_TIMER_H

/**
 * ArduMon: Yet another Arduino serial command library.
 *
 * See https://github.com/martyvona/ArduMon/blob/main/README.md
 *
 * This is a countdown timer which demonstrates several ways ArduMon commands can send data over time.
 *
 * In synchronous text mode the ArduMonTimer::start() command does not end until the timer reaches zero, or the user
 * stops it by hitting any key.  The Arduino loop() function is not blocked.  The remaining time is periodically
 * reported to the user using VT100 control codes to create a rolling counter that overwrites itself in the terminal
 * without scrolling.
 * 
 * In asynchronous text mode the ArduMonTimer::start() command ends quickly, but the timer keeps running.  The time can
 * be requested later with ArduMonTimer::send(), and the timer can be stopped with ArduMonTimer::stop().
 * 
 * Synchronous and asynchronous binary modes are similar to the corresponding text modes, except the time reports are
 * sent as binary packets instead of text and VT100 control codes.  These packets can either start with a configurable
 * command code or not.  This demonstrates that the receiver, which can be a separate instance of ArduMon running on
 * another processor, can receive response packets either as incoming commands or as generic packets using the ArduMon
 * setUniversalHandler() facility.
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
template <typename AM> class ArduMonTimer {
public:

  struct Cmd : public AM::Runnable { ArduMonTimer& tm; Cmd(ArduMonTimer &_tm) : tm(_tm) {} };
  struct StartCmd : public Cmd { using Cmd::Cmd; bool run(AM &am) { return Cmd::tm.start(am); } };
  struct StopCmd  : public Cmd { using Cmd::Cmd; bool run(AM &am) { return Cmd::tm.stop(am); } };
  struct GetCmd   : public Cmd { using Cmd::Cmd; bool run(AM &am) { return Cmd::tm.send(am); } };

  StartCmd start_cmd; StopCmd stop_cmd; GetCmd get_cmd;

  ArduMonTimer() : start_cmd(*this), stop_cmd(*this), get_cmd(*this) {}

  bool start(AM &am) {

    running = false;
    if (!am.skip()) return false; //skip over command token or code

    uint8_t h, m, s;
    if (!am.recv(h).recv(m).recv(s)) return false;
    total_ms = remaining_ms = (h * 3600 + m * 60 + s) * 1000ul;

    accel = 1;
    if (am.argc() > 4 && !am.recv(accel)) return false;

    sync_throttle_ms = 100;
    if (am.argc() > 5 && !am.recv(sync_throttle_ms)) return false;

    bin_response_code = -1;
    if (am.argc() > 6 && !am.recv(bin_response_code)) return false;

    if (am.isTextMode()) {
      //send()s cannot error in text mode
      am.sendRaw(F("counting down from "))
        .sendRaw(h).sendRaw(':').sendRaw(m, 2|AM::FMT_PAD_ZERO).sendRaw(':').sendRaw(s, 2|AM::FMT_PAD_ZERO)
        .sendRaw(F(", accel=")).sendRaw(accel)
        .sendRaw(!isSynchronous() ? F(", async=true") : F(", hit any key to cancel..."))
        .sendCRLF(true);
      if (isSynchronous())
        am.vt100CursorHidden()
          //.vt100SetAttr(AM::VT100_ATTR_REVERSE)
          //.vt100SetAttr(AM::VT100_ATTR_BLINK)
          //.vt100SetAttr(AM::VT100_ATTR_BRIGHT)
          //.vt100SetAttr(AM::VT100_ATTR_UNDERSCORE)
          .vt100SetColor(AM::VT100_FOREGROUND, AM::VT100_CYAN);
    }

    start_ms = last_send_ms = millis();
    elapsed_ms = 0;
    running = true;

    if (!isSynchronous()) return am.endHandler();
    else return send(am, start_ms);
  }

  bool stop(AM &am) { running = false; return am.endHandler(); }

  bool send(AM &am, const uint64_t now) {
    last_send_ms = now;
    if (am.isTextMode()) {
      const uint8_t h = remaining_ms / (3600 * 1000ul);
      const uint8_t m = (remaining_ms - h * 3600 * 1000ul) / (60 * 1000ul);
      const uint8_t s = (remaining_ms - (h * 3600 + m * 60) * 1000ul) / 1000ul;
      const uint16_t ms = remaining_ms - (h * 3600 + m * 60 + s) * 1000ul;
      if (isSynchronous()) am.vt100ClearLine();
      am.sendRaw(h, 3|AM::FMT_PAD_ZERO).sendRaw(':')
        .sendRaw(m, 2|AM::FMT_PAD_ZERO).sendRaw(':')
        .sendRaw(s, 2|AM::FMT_PAD_ZERO).sendRaw('.').sendRaw(ms, 3|AM::FMT_PAD_ZERO);
      if (isSynchronous() && !running) am.vt100CursorVisible().vt100SetAttr(AM::VT100_ATTR_RESET);
      if (!isSynchronous() || !running) am.sendCRLF(true);
      return true; //send()s cannot error in text mode
    } else { //binary response; 2^32 msec ~= 49 days, but we are limited to about 11 days since h,m,s <= 255
      return (bin_response_code < 0 || bin_response_code > 255 || am.send(static_cast<uint8_t>(bin_response_code))) &&
        am
        .send(static_cast<uint32_t>(total_ms))
        .send(static_cast<uint32_t>(elapsed_ms))
        .send(static_cast<uint32_t>(remaining_ms))
        .sendPacket();
    }
  }

  bool send(AM &am) { return send(am, millis()) && am.endHandler(); }

  bool isRunning() { return running; };

  bool isSynchronous() { return sync_throttle_ms >= 0; }

  bool tick(AM &am) {
    if (!running) return false;
    const uint64_t now = millis();
    elapsed_ms = static_cast<uint32_t>(millis() - start_ms) * accel; //elapsed_ms > total_ms if not tick()ed enough
    remaining_ms = elapsed_ms <= total_ms ? total_ms - elapsed_ms : 0;
    if (remaining_ms == 0 || (isSynchronous() && am.isTextMode() && am.getKey() != 0)) running = false;
    if (isSynchronous()) {
      if (now - last_send_ms >= sync_throttle_ms || !running) send(am, now);
      if (!running) am.endHandler();
    }
    return running;
  }

private:
  bool running;
  float accel;
  int16_t bin_response_code, sync_throttle_ms;
  uint64_t start_ms, last_send_ms;
  uint32_t total_ms, elapsed_ms, remaining_ms;
};

#endif //AM_TIMER_H
