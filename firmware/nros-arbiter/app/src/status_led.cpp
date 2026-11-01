// The onboard RGB LED shows the mode, using the pattern table the host tests
// cover (led_pattern.h). A low-priority thread of its own: the LED is for the
// person at the car, so it must not depend on zenoh, and it must not cost the
// control loop the PIO write.
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/kernel.h>

#include "control.h"
#include "led_pattern.h"

namespace {

constexpr uint32_t kPeriodMs = 20;
constexpr uint8_t kBrightness = 64;  // of 255, as the PlatformIO build

const gpio_dt_spec kLedPower = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), led_power_gpios);
const device* const kStrip = DEVICE_DT_GET(DT_ALIAS(led_strip));

uint8_t scale(uint8_t v) { return static_cast<uint8_t>((v * kBrightness) / 255); }

void run(void*, void*, void*) {
  if (!gpio_is_ready_dt(&kLedPower) || !device_is_ready(kStrip)) return;
  gpio_pin_configure_dt(&kLedPower, GPIO_OUTPUT_ACTIVE);
  for (;;) {
    arbiter::Rgb c = arbiter::led_pattern(control::mode(), k_uptime_get_32());
    led_rgb pixel = {};
    pixel.r = scale(c.r);
    pixel.g = scale(c.g);
    pixel.b = scale(c.b);
    led_strip_update_rgb(kStrip, &pixel, 1);
    k_msleep(kPeriodMs);
  }
}

K_THREAD_DEFINE(status_led_thread, 1024, run, nullptr, nullptr, nullptr, 10, 0, 0);

}  // namespace
