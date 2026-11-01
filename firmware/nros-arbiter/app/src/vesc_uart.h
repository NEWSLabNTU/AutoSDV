// VESC commands over UART0 (D6/D7), sent from an interrupt-driven TX buffer
// so the 1 kHz control loop never waits on the 115200 baud line.
#pragma once

#include "arbiter.h"

namespace vesc_uart {

bool begin();

// Queues one command; a packet that does not fit the buffer is dropped (the
// next one follows 10 ms later, inside the VESC's 50 ms timeout).
void send(const arbiter::VescCommand& c);

}  // namespace vesc_uart
