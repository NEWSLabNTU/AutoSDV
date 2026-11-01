# RC override arbiter firmware for the Seeed XIAO RP2350.
#
# Sources: firmware/rc-arbiter. Design: docs/design/rc-override-arbiter.md.
# Needs PlatformIO: ./setup.sh --run --only platformio

set working-directory := '..'

fw := 'firmware/rc-arbiter'

# `just <module>` runs the module's FIRST recipe, so that recipe has to be the
# harmless one. Private keeps it out of `just --list --list-submodules`.
[private]
default:
    @just --list arbiter

# Host unit tests of the arbiter and the VESC packets; no board needed
test:
    pio test -d {{fw}} -e native

# Cross-compile for the board; prints flash and RAM use
build:
    pio run -d {{fw}} -e xiao_rp2350

# Flash over USB. First time: hold BOOT while plugging the board in
flash:
    pio run -d {{fw}} -e xiao_rp2350 -t upload

# USB serial log: one line per mode change
monitor:
    pio device monitor -d {{fw}} -e xiao_rp2350

# Remove the build output (.pio/build); the downloaded toolchain stays
clean:
    pio run -d {{fw}} -e xiao_rp2350 -t clean
    rm -rf {{fw}}/.pio/build/native

# ---- Zephyr + nano-ros build (firmware/nros-arbiter) -----------------------
# The same arbiter logic (firmware/rc-arbiter/lib) on Zephyr 4.4, with a
# nano-ros node that publishes the mode to the Orin over USB serial.

nros := 'firmware/nros-arbiter'

# Fetch Zephyr 4.4 and nano-ros, apply nano-ros's patches, build the nros CLI (~1 GB, once)
nros-setup:
    {{nros}}/ws.sh setup

# Cross-compile the Zephyr + nano-ros image; prints flash and RAM use
nros-build *ARGS:
    {{nros}}/ws.sh build {{ARGS}}

# Copy the UF2 onto a board in BOOTSEL mode (hold BOOT while plugging in)
nros-flash:
    {{nros}}/ws.sh flash

# Remove the Zephyr build directory; the workspace stays
nros-clean:
    {{nros}}/ws.sh clean
