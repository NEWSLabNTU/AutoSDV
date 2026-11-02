// What the control loop reads from and writes to the board, apart from the
// pulse inputs (capture.h) and the VESC (vesc_uart.h). board_io_xiao.cpp is
// the car's XIAO RP2350; sim/board_io_sim.cpp stands in on a board with none
// of this hardware (QEMU mps2/an521), so the same loop can run there.
#pragma once

#include <stdint.h>

namespace board_io {

// Configures every input and output, capture and the VESC UART included;
// false if any of them is not ready.
bool begin();

// The auto-mode switch, debounced by the arbiter, not here.
bool switch_on(uint32_t now_ms);

void set_auto_line(bool on);

// One steering pulse width; called only when it changes.
void set_servo_us(uint16_t us);

// Starts a watchdog that resets the board after timeout_ms without a feed.
// Returns false if there is none or it cannot be set up.
bool watchdog_start(uint32_t timeout_ms);
void watchdog_feed();

}  // namespace board_io
