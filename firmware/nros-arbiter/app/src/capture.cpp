#include "capture.h"

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

namespace capture {

namespace {

#define USER_NODE DT_PATH(zephyr_user)

const gpio_dt_spec kPins[kCount] = {
    GPIO_DT_SPEC_GET(USER_NODE, rx_steer_gpios),
    GPIO_DT_SPEC_GET(USER_NODE, rx_throttle_gpios),
    GPIO_DT_SPEC_GET(USER_NODE, orin_throttle_gpios),
    GPIO_DT_SPEC_GET(USER_NODE, orin_steer_gpios),
    GPIO_DT_SPEC_GET(USER_NODE, heartbeat_gpios),
};

struct State {
  bool high;
  uint32_t rise_us;
  uint16_t width_us;
  uint32_t fall_ms;
  uint32_t seq;
};

State channels[kCount];
gpio_callback callbacks[kCount];

uint32_t now_us() { return k_cyc_to_us_floor32(k_cycle_get_32()); }

void on_edge(const device* port, gpio_callback* cb, gpio_port_pins_t pins) {
  uint32_t t = now_us();
  int idx = cb - callbacks;
  State& s = channels[idx];
  if (gpio_pin_get_dt(&kPins[idx]) > 0) {
    s.high = true;
    s.rise_us = t;
  } else if (s.high) {
    // Only a falling edge after a seen rising edge is a pulse; the first fall
    // after boot has no start.
    s.high = false;
    uint32_t w = t - s.rise_us;
    s.width_us = w > 0xffff ? 0xffff : static_cast<uint16_t>(w);
    s.fall_ms = k_uptime_get_32();
    s.seq = s.seq + 1;
  }
  ARG_UNUSED(port);
  ARG_UNUSED(pins);
}

}  // namespace

bool begin() {
  for (int i = 0; i < kCount; ++i) {
    const gpio_dt_spec& p = kPins[i];
    if (!gpio_is_ready_dt(&p)) return false;
    if (gpio_pin_configure_dt(&p, GPIO_INPUT) != 0) return false;
    // One callback per pin, so the handler knows its channel from the
    // callback's address.
    gpio_init_callback(&callbacks[i], on_edge, BIT(p.pin));
    if (gpio_add_callback_dt(&p, &callbacks[i]) != 0) return false;
    if (gpio_pin_interrupt_configure_dt(&p, GPIO_INT_EDGE_BOTH) != 0) return false;
  }
  return true;
}

arbiter::Pulse read(Channel ch, uint32_t now_ms) {
  arbiter::Pulse p;
  unsigned int key = irq_lock();
  const State& s = channels[ch];
  p.width_us = s.width_us;
  uint32_t fall_ms = s.fall_ms;
  p.seq = s.seq;
  irq_unlock(key);
  p.age_ms = p.seq == 0 ? UINT32_MAX : now_ms - fall_ms;
  return p;
}

}  // namespace capture
