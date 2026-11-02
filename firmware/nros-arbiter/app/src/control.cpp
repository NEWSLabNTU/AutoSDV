#include "control.h"

#include <atomic>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "board_io.h"
#include "capture.h"
#include "vesc_uart.h"

LOG_MODULE_REGISTER(arbiter, LOG_LEVEL_INF);

namespace control {

namespace {

constexpr uint32_t kVescPeriodMs = 10;  // 100 Hz, well inside the 50 ms VESC timeout
constexpr uint32_t kWatchdogMs = 100;

std::atomic<uint8_t> current_mode{static_cast<uint8_t>(arbiter::Mode::Manual)};
std::atomic<uint32_t> changes{0};

arbiter::Arbiter arb;

void run(void*, void*, void*) {
  if (!board_io::begin()) {
    // Without its inputs or outputs the board cannot arbitrate. Driving
    // nothing leaves the VESC to brake on its own UART timeout.
    LOG_ERR("hardware init failed; arbiter not running");
    return;
  }
  // A hung loop stops the VESC commands, and the VESC brakes on its own
  // timeout; the watchdog then restarts the board in MANUAL.
  if (!board_io::watchdog_start(kWatchdogMs)) LOG_ERR("watchdog setup failed");

  k_timer tick;
  k_timer_init(&tick, nullptr, nullptr);
  k_timer_start(&tick, K_MSEC(1), K_MSEC(1));

  uint32_t last_vesc_ms = 0;
  uint16_t last_servo_us = 0;
  arbiter::Mode last_mode = arbiter::Mode::Manual;

  for (;;) {
    k_timer_status_sync(&tick);
    uint32_t now = k_uptime_get_32();

    arbiter::Inputs in;
    in.switch_on = board_io::switch_on(now);
    in.rx_steer = capture::read(capture::RxSteer, now);
    in.rx_throttle = capture::read(capture::RxThrottle, now);
    in.orin_steer = capture::read(capture::OrinSteer, now);
    in.orin_throttle = capture::read(capture::OrinThrottle, now);
    in.heartbeat = capture::read(capture::Heartbeat, now);

    arbiter::Outputs out = arb.step(in, now);

    // No servo pulse until a valid steering source has been seen.
    if (out.servo_us != 0 && out.servo_us != last_servo_us) {
      board_io::set_servo_us(out.servo_us);
      last_servo_us = out.servo_us;
    }
    board_io::set_auto_line(out.auto_line);

    if (now - last_vesc_ms >= kVescPeriodMs) {
      last_vesc_ms = now;
      vesc_uart::send(out.vesc);
    }

    if (out.mode != last_mode) {
      LOG_INF("%u ms: %s -> %s", now, arbiter::mode_name(last_mode),
              arbiter::mode_name(out.mode));
      last_mode = out.mode;
      current_mode.store(static_cast<uint8_t>(out.mode), std::memory_order_relaxed);
      changes.fetch_add(1, std::memory_order_relaxed);
    }

    board_io::watchdog_feed();
  }
}

// Cooperative and above every nano-ros thread (main runs at priority 5), so
// zenoh traffic cannot delay a step.
K_THREAD_DEFINE(control_thread, 2048, run, nullptr, nullptr, nullptr, K_PRIO_COOP(2), 0, 0);

}  // namespace

arbiter::Mode mode() {
  return static_cast<arbiter::Mode>(current_mode.load(std::memory_order_relaxed));
}

uint32_t mode_changes() { return changes.load(std::memory_order_relaxed); }

}  // namespace control
