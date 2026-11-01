#include "vesc_uart.h"

#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>

#include "vesc_packet.h"

namespace vesc_uart {

namespace {

const device* const kUart = DEVICE_DT_GET(DT_NODELABEL(uart0));

RING_BUF_DECLARE(tx_ring, 64);

void on_irq(const device* dev, void*) {
  if (!uart_irq_update(dev)) return;
  if (uart_irq_tx_ready(dev)) {
    uint8_t* data;
    uint32_t len = ring_buf_get_claim(&tx_ring, &data, 16);
    if (len == 0) {
      uart_irq_tx_disable(dev);
      return;
    }
    int sent = uart_fifo_fill(dev, data, len);
    ring_buf_get_finish(&tx_ring, sent > 0 ? sent : 0);
  }
}

}  // namespace

bool begin() {
  if (!device_is_ready(kUart)) return false;
  return uart_irq_callback_user_data_set(kUart, on_irq, nullptr) == 0;
}

void send(const arbiter::VescCommand& c) {
  uint8_t buf[vesc::kMaxPacket];
  size_t n = c.kind == arbiter::VescCommand::Kind::Current ? vesc::set_current(c.amps, buf)
                                                           : vesc::set_brake_current(c.amps, buf);
  unsigned int key = irq_lock();
  bool fits = ring_buf_space_get(&tx_ring) >= n;
  if (fits) ring_buf_put(&tx_ring, buf, n);
  irq_unlock(key);
  if (fits) uart_irq_tx_enable(kUart);
}

}  // namespace vesc_uart
