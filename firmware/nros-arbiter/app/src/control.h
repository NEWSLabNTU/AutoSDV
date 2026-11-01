// The arbiter's control loop: a cooperative thread at 1 kHz that reads the
// inputs, runs arbiter::Arbiter::step() and writes the servo, the VESC, the
// AUTO line and the watchdog. It starts at boot and never depends on zenoh:
// the board arbitrates whether or not the Orin's ROS link is up.
#pragma once

#include <stdint.h>

#include "arbiter.h"

namespace control {

// The current mode, readable from any thread.
arbiter::Mode mode();

// Mode changes since boot.
uint32_t mode_changes();

}  // namespace control
