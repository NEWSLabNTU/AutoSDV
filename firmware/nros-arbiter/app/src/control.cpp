#include "control.h"

#include <atomic>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "capture.h"
#include "vesc_uart.h"

LOG_MODULE_REGISTER(arbiter, LOG_LEVEL_INF);

namespace control {

namespace {

constexpr uint32_t kVescPeriodMs = 10;  // 100 Hz, well inside the 50 ms VESC timeout
constexpr uint32_t kWatchdogMs = 100;

#define USER_NODE DT_PATH(zephyr_user)

const gpio_dt_spec kSwitch = GPIO_DT_SPEC_GET(USER_NODE, switch_gpios);
const gpio_dt_spec kAutoLine = GPIO_DT_SPEC_GET(USER_NODE, auto_line_gpios);
const pwm_dt_spec kServo = PWM_DT_SPEC_GET(USER_NODE);
const device* const kWatchdog = DEVICE_DT_GET(DT_ALIAS(watchdog0));

std::atomic<uint8_t> current_mode{static_cast<uint8_t>(arbiter::Mode::Manual)};
std::atomic<uint32_t> changes{0};

arbiter::Arbiter arb;

int setup_watchdog() {
  wdt_timeout_cfg cfg = {};
  cfg.window.min = 0;
  cfg.window.max = kWatchdogMs;
  cfg.flags = WDT_FLAG_RESET_SOC;
  int channel = wdt_install_timeout(kWatchdog, &cfg);
  if (channel < 0) return channel;
  int err = wdt_setup(kWatchdog, WDT_OPT_PAUSE_HALTED_BY_DBG);
  return err < 0 ? err : channel;
}

void run(void*, void*, void*) {
  bool ok = gpio_is_ready_dt(&kSwitch) && gpio_pin_configure_dt(&kSwitch, GPIO_INPUT) == 0 &&
            gpio_is_ready_dt(&kAutoLine) &&
            gpio_pin_configure_dt(&kAutoLine, GPIO_OUTPUT_INACTIVE) == 0 &&
            pwm_is_ready_dt(&kServo) && capture::begin() && vesc_uart::begin();
  if (!ok) {
    // Without its inputs or outputs the board cannot arbitrate. Driving
    // nothing leaves the VESC to brake on its own UART timeout.
    LOG_ERR("hardware init failed; arbiter not running");
    return;
  }
  // A hung loop stops the VESC commands, and the VESC brakes on its own
  // timeout; the watchdog then restarts the board in MANUAL.
  int wdt_channel = setup_watchdog();
  if (wdt_channel < 0) LOG_ERR("watchdog setup failed (%d)", wdt_channel);

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
    in.switch_on = gpio_pin_get_dt(&kSwitch) > 0;
    in.rx_steer = capture::read(capture::RxSteer, now);
    in.rx_throttle = capture::read(capture::RxThrottle, now);
    in.orin_steer = capture::read(capture::OrinSteer, now);
    in.orin_throttle = capture::read(capture::OrinThrottle, now);
    in.heartbeat = capture::read(capture::Heartbeat, now);

    arbiter::Outputs out = arb.step(in, now);

    // No servo pulse until a valid steering source has been seen.
    if (out.servo_us != 0 && out.servo_us != last_servo_us) {
      pwm_set_pulse_dt(&kServo, PWM_USEC(out.servo_us));
      last_servo_us = out.servo_us;
    }
    gpio_pin_set_dt(&kAutoLine, out.auto_line ? 1 : 0);

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

    if (wdt_channel >= 0) wdt_feed(kWatchdog, wdt_channel);
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
