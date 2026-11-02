// board_io on the XIAO RP2350: the pins of app.overlay's zephyr,user node.
#include "board_io.h"

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/watchdog.h>

#include "capture.h"
#include "vesc_uart.h"

namespace board_io {

namespace {

#define USER_NODE DT_PATH(zephyr_user)

const gpio_dt_spec kSwitch = GPIO_DT_SPEC_GET(USER_NODE, switch_gpios);
const gpio_dt_spec kAutoLine = GPIO_DT_SPEC_GET(USER_NODE, auto_line_gpios);
const pwm_dt_spec kServo = PWM_DT_SPEC_GET(USER_NODE);
const device* const kWatchdog = DEVICE_DT_GET(DT_ALIAS(watchdog0));

int wdt_channel = -1;

}  // namespace

bool begin() {
  return gpio_is_ready_dt(&kSwitch) && gpio_pin_configure_dt(&kSwitch, GPIO_INPUT) == 0 &&
         gpio_is_ready_dt(&kAutoLine) &&
         gpio_pin_configure_dt(&kAutoLine, GPIO_OUTPUT_INACTIVE) == 0 &&
         pwm_is_ready_dt(&kServo) && capture::begin() && vesc_uart::begin();
}

bool switch_on(uint32_t) { return gpio_pin_get_dt(&kSwitch) > 0; }

void set_auto_line(bool on) { gpio_pin_set_dt(&kAutoLine, on ? 1 : 0); }

void set_servo_us(uint16_t us) { pwm_set_pulse_dt(&kServo, PWM_USEC(us)); }

bool watchdog_start(uint32_t timeout_ms) {
  wdt_timeout_cfg cfg = {};
  cfg.window.min = 0;
  cfg.window.max = timeout_ms;
  cfg.flags = WDT_FLAG_RESET_SOC;
  int channel = wdt_install_timeout(kWatchdog, &cfg);
  if (channel < 0) return false;
  if (wdt_setup(kWatchdog, WDT_OPT_PAUSE_HALTED_BY_DBG) < 0) return false;
  wdt_channel = channel;
  return true;
}

void watchdog_feed() {
  if (wdt_channel >= 0) wdt_feed(kWatchdog, wdt_channel);
}

}  // namespace board_io
