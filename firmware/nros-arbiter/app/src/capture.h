// Pulse capture on GPIO interrupts, the Zephyr counterpart of
// firmware/rc-arbiter/src/capture.h: each edge is timestamped from the
// hardware cycle counter, each falling edge completes a pulse.
#pragma once

#include <stdint.h>

#include "pulse.h"

namespace capture {

enum Channel : uint8_t { RxSteer, RxThrottle, OrinThrottle, OrinSteer, Heartbeat, kCount };

// Configures the five inputs and their interrupts; false if a device is not ready.
bool begin();

// A consistent copy of one channel, with its age relative to now_ms.
arbiter::Pulse read(Channel ch, uint32_t now_ms);

}  // namespace capture
