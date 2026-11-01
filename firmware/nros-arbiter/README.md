# RC override arbiter on Zephyr + nano-ros

The same arbiter as [`../rc-arbiter`](../rc-arbiter), built on Zephyr 4.4 with
[nano-ros](https://github.com/NEWSLabNTU/nano-ros), so the board can also talk
ROS 2 to the Orin. What the arbiter does and why is in
[`docs/design/rc-override-arbiter.md`](../../docs/design/rc-override-arbiter.md).

Status: builds (551,764 B of 1 MB flash, 266,032 B of 520 KB RAM, nano-ros
`41891e9`). It has not run on a board or in an emulator yet.

## What is shared and what is not

The decisions live in `../rc-arbiter/lib` (the state machine, pulse handling,
the heartbeat, the LED table, the VESC packets). This build compiles those
files in place; it copies nothing, so `just arbiter test` covers both builds.
`app/src` is only the Zephyr hardware layer and the ROS node.

| | `rc-arbiter` (PlatformIO) | `nros-arbiter` (this) |
|---|---|---|
| Framework | arduino-pico | Zephyr 4.4 |
| Control loop | `loop()` at 1 kHz | cooperative thread at 1 kHz (`control.cpp`) |
| Pulse capture | GPIO interrupts, `micros()` | GPIO interrupts, cycle counter |
| VESC UART | `Serial1.write` | interrupt-driven TX buffer |
| Log | USB serial | second USB CDC-ACM port |
| ROS 2 | none | `/vehicle/arbiter/mode` (`std_msgs/String`, 10 Hz) over USB CDC-ACM |

The ROS link is report-only: nothing received over it can change who drives.
The control loop and the LED run whether or not the link is up.

## Layout

```
manifest/west.yml          Zephyr 4.4 + hal_rpi_pico + nano-ros, pinned
boards/nros-board-xiao-rp2350/   the nano-ros board package: one descriptor,
                           one Kconfig fragment (found via NROS_EXTRA_BOARD_PATH)
app/
  system.toml              the nano-ros image: board, RMW, serial locator, and
                           the node's entities, which size its RAM pools
  prj.conf, prj-link.conf  Kconfig: hardware, then the zenoh-over-USB link
  app.overlay              pin map (same as ../rc-arbiter/src/pins.h)
  src/                     control loop, capture, VESC UART, LED, ROS node
ws.sh                      workspace setup, build, flash
```

## Developer steps

```bash
just arbiter nros-setup    # once: ~1 GB into firmware/nros-arbiter/.ws
just arbiter nros-build    # prints flash and RAM use
just arbiter nros-flash    # hold BOOT while plugging the board in first
```

Prerequisites: `west`, `uv` (for Zephyr 4.4's Python 3.12 venv), a Rust
toolchain (to build the nros CLI), and an Arm GNU toolchain, found in
`GNUARMEMB_TOOLCHAIN_PATH` or nano-ros's store (`~/.nros/sdk/arm-none-eabi-gcc`).
Set `NROS_ARBITER_WS` to keep the workspace elsewhere.

## The Orin end

The board is a zenoh client on its first CDC-ACM port. The Orin needs a zenoh
router listening on that serial device, and a bridge into the car's ROS 2
setup, which runs Cyclone DDS rather than rmw_zenoh. Not set up yet.

## Pins

USB carries both CDC-ACM ports, so UART0 (D6/D7) is free for the VESC. The
board's default console (uart0) moves to the second CDC-ACM port, and uart1 is
disabled because its TX pin is D12, the optional external LED. That LED is
not driven by this build yet; the onboard RGB LED is.
