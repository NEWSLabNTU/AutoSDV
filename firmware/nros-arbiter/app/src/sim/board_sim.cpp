// The board, simulated: board_io, capture and vesc_uart for a target with
// none of the car's hardware (QEMU mps2/an521). The inputs follow a fixed
// 12 s script that walks the arbiter through every ordinary mode:
//
//     0 s  switch OFF, radio and Orin pulses valid, heartbeat toggling  MANUAL
//     3 s  switch ON, sticks centred                                     ARMING
//    ~4 s  held 1 s with the heartbeat alive                             AUTO
//     8 s  heartbeat stops toggling                                      ORIN_LOST
//    10 s  switch OFF                                                    MANUAL
//
// and then repeats, so a run of any length keeps publishing transitions.
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "board_io.h"
#include "capture.h"
#include "config.h"
#include "vesc_uart.h"

LOG_MODULE_REGISTER(board_sim, LOG_LEVEL_INF);

namespace {

constexpr uint32_t kCycleMs = 12000;
constexpr uint32_t kSwitchOnMs = 3000;
constexpr uint32_t kHeartbeatStopMs = 8000;
constexpr uint32_t kSwitchOffMs = 10000;
constexpr uint32_t kFrameMs = 20;  // 50 Hz, the receiver's and the PCA9685's rate

uint32_t phase(uint32_t now_ms) { return now_ms % kCycleMs; }

}  // namespace

namespace board_io {

bool begin() {
  LOG_INF("simulated board: inputs follow a %u ms script", kCycleMs);
  return capture::begin() && vesc_uart::begin();
}

bool switch_on(uint32_t now_ms) {
  uint32_t t = phase(now_ms);
  return t >= kSwitchOnMs && t < kSwitchOffMs;
}

void set_auto_line(bool) {}

void set_servo_us(uint16_t us) { LOG_INF("servo %u us", us); }

// No watchdog: a reset in QEMU would only restart the script.
bool watchdog_start(uint32_t) { return true; }
void watchdog_feed() {}

}  // namespace board_io

namespace capture {

bool begin() { return true; }

arbiter::Pulse read(Channel ch, uint32_t now_ms) {
  arbiter::Pulse p;
  uint32_t frame = now_ms / kFrameMs;
  p.seq = frame + 1;
  p.age_ms = now_ms % kFrameMs;
  switch (ch) {
    case RxSteer:
    case RxThrottle:
    case OrinSteer:
      p.width_us = arbiter::cfg::kRxCentreUs;
      break;
    case OrinThrottle:
      p.width_us = arbiter::cfg::kOrinThrottle.centre_us;
      break;
    case Heartbeat: {
      // Alternates every frame while alive; frozen high once it "dies".
      bool alive = phase(now_ms) < kHeartbeatStopMs;
      bool high = alive ? (frame & 1u) : true;
      p.width_us = high ? 2000 : 1000;
      break;
    }
    default:
      break;
  }
  return p;
}

}  // namespace capture

namespace vesc_uart {

bool begin() { return true; }

void send(const arbiter::VescCommand&) {}

}  // namespace vesc_uart
