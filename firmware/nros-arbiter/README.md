# RC override arbiter on Zephyr + nano-ros

The same arbiter as [`../rc-arbiter`](../rc-arbiter), built on Zephyr 4.4 with
[nano-ros](https://github.com/NEWSLabNTU/nano-ros), so the board can also talk
ROS 2 to the Orin. What the arbiter does and why is in
[`docs/design/rc-override-arbiter.md`](../../docs/design/rc-override-arbiter.md).

Status: builds (553,008 B of 1 MB flash, 266,032 B of 520 KB RAM, nano-ros
`2c6b8c2`). Runs end to end in QEMU on a Cortex-M33 (below); not yet run on
a board.

## What is shared and what is not

The decisions live in `../rc-arbiter/lib` (the state machine, pulse handling,
the heartbeat, the LED table, the VESC packets). This build compiles those
files in place; it copies nothing, so `just arbiter test` covers both builds.
`app/src` is only the Zephyr hardware layer and the ROS node, and the
hardware layer sits behind `board_io.h` (plus `capture.h` and `vesc_uart.h`),
so the same control loop runs on the XIAO and, simulated, in QEMU.

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
boards/                    nano-ros board packages (found via NROS_EXTRA_BOARD_PATH)
  nros-board-xiao-rp2350/  the car's board
  nros-board-mps2-an521/   QEMU's Cortex-M33, the emulator stand-in
patches/                   upstream fixes the pinned Zephyr / zenoh-pico lack
app/
  system.toml              the images (arbiter, arbiter_qemu): board, RMW,
                           serial locator, and the node's entities, which size
                           its RAM pools
  prj.conf                 Kconfig every image shares
  prj-link.conf            the zenoh link over a UART
  prj-xiao.conf            the XIAO's hardware and USB CDC-ACM
  boards/*.overlay         per-board devicetree; the XIAO's is the pin map
                           (same as ../rc-arbiter/src/pins.h)
  src/                     control loop, ROS node, the XIAO hardware layer
  src/sim/board_sim.cpp    the hardware layer, simulated by a 12 s script
qemu/                      end-to-end run: QEMU <-> rmw_zenohd <-> ROS 2
wokwi/                     Wokwi project (see below; not usable for this image)
ws.sh                      workspace setup, build, flash, qemu, sim
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

## Patches

`ws.sh setup` applies `patches/` after nano-ros's own 4.4 patch set. Drop each
once the pinned version contains it.

`patches/zephyr-*.patch`, upstream Zephyr fixes that v4.4.0 lacks:

- `bd05a26a67` (hal_rpi_pico uninitialized data). The RP2350 entropy driver
  pulls in pico_rand, whose `NOLOAD` section landed in the middle of the
  initialized data region, so every section after it was copied from the
  wrong flash address at boot. On this image that is the USB device context:
  USB init fails with "Failed to initialize language descriptor (-1)" and
  then faults. Found in the simulator; it would hit the board the same way.

`patches/zenoh-pico-*.patch`, fixes to nano-ros's zenoh-pico fork, for when
the pin lacks one. nano-ros compiles zenoh-pico from a shared store unless its
own checkout's submodule carries edits, so `ws.sh setup` checks the submodule
out at nano-ros's pin inside `.ws` and applies them there, never to the store.
None is needed at the current pin: the last one (a UART with no runtime
configuration, which QEMU's CMSDK UART is) landed in the fork as `fed5d711`.

## Emulator (QEMU)

```bash
just arbiter nros-qemu [SECONDS]    # default 30
```

QEMU's `mps2-an521` is a Cortex-M33, the RP2350's core, with none of the car's
hardware: image `arbiter_qemu` swaps the hardware layer for
`src/sim/board_sim.cpp`, whose inputs follow a 12 s script (switch ON at 3 s,
AUTO about 1 s later, heartbeat lost at 8 s, switch OFF at 10 s). The zenoh
link runs on uart1 through a socat pty pair to a stock `rmw_zenohd` with a
serial listener, and a ROS 2 subscriber prints every mode change it receives.
The run fails when the subscriber receives nothing. A passing run:

```
--- board
[00:00:03.050,000] <inf> arbiter: 3050 ms: MANUAL -> ARMING
[00:00:04.050,000] <inf> arbiter: 4050 ms: ARMING -> AUTO
[00:00:08.131,000] <inf> arbiter: 8131 ms: AUTO -> DISENGAGED_ORIN_LOST
[00:00:10.050,000] <inf> arbiter: 10050 ms: DISENGAGED_ORIN_LOST -> MANUAL
--- ROS 2 subscriber
  0.07 s  MANUAL
  3.67 s  ARMING
  4.98 s  AUTO
 10.10 s  DISENGAGED_ORIN_LOST
 12.45 s  MANUAL
messages: 239
```

Needs ROS 2 Humble with `ros-humble-rmw-zenoh-cpp`, and `socat`. The router
listens on 127.0.0.1:7457, so a router already running on port 7447 is left
alone. QEMU's timing is approximate (`-icount shift=auto`), so the host's
times drift from the board's.

What it does not cover: the RP2350's peripherals (pulse capture, PWM, the VESC
UART, USB, the LED), and floating point in hardware (Zephyr's AN521 `cpu0`
declares no FPU, so floats are soft here).

The board keeps dialling until the router answers: both board tomls state
`[knobs.zenoh.limits] connect_forever = true`, because nano-ros's default is
one attempt and the car powers the arbiter up before the Orin. The late-router
case has its own run, with the router and subscriber started 20 s after the
board:

```bash
just arbiter nros-qemu 45 20
```

The board's session comes up when the router does, and the subscriber
receives from then on (175 messages in the remaining 25 s). The control loop
runs either way.

## Simulator (Wokwi)

```bash
just arbiter nros-sim --timeout 10000
```

Wokwi has no XIAO RP2350, but its Pi Pico 2 is the same RP2350 and the
firmware addresses pins by GPIO number. The simulator build differs in one
way (`wokwi/sim.overlay`): the log goes to uart1 (GPIO20/21), because Wokwi's
serial monitor cannot read the second USB CDC-ACM port. `wokwi/diagram.json`
wires the mode switch, the servo and the RGB LED. Needs `wokwi-cli` and a token
in `$WOKWI_CLI_TOKEN` or `~/.config/wokwi/token` (never in this repository).

Status: not usable for this image. It boots and USB comes up, then the first
zenoh-pico heap allocation returns to its caller with the callee's frame still
on the stack. The same image runs cleanly on QEMU's Cortex-M33 (above), so the
fault is Wokwi's emulation, not the firmware. Hello world runs fine there.

## The Orin end

The board is a zenoh client on its first CDC-ACM port. The Orin needs a zenoh
router listening on that serial device (a stock `rmw_zenohd` has the serial
link; `qemu/e2e.sh` shows the config), and a bridge into the car's ROS 2
setup, which runs Cyclone DDS rather than rmw_zenoh. Not set up yet.

## Pins

USB carries both CDC-ACM ports, so UART0 (D6/D7) is free for the VESC. The
board's default console (uart0) moves to the second CDC-ACM port, and uart1 is
disabled because its TX pin is D12, the optional external LED. That LED is
not driven by this build yet; the onboard RGB LED is.
