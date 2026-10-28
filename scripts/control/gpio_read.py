#!/usr/bin/env python3
"""Watch the hall sensor's GPIO pin: its live level and every edge (drives nothing).

    python3 scripts/control/gpio_read.py            # live level, refreshed in place
    python3 scripts/control/gpio_read.py --edges    # plus one line per edge

Opens the pin's kernel event line itself, for BOTH edges, instead of
Jetson.GPIO's add_event_detect: there, input() reads through the line handle
add_event_detect closed (Jetson.GPIO 2.1.x), so the level cannot be shown.
On its own event line the level is read at any moment, and each event says
whether it rose or fell and carries the kernel's timestamp, not the time a
Python callback happened to run.

The status line shows the level now, the rising and falling counts, and the
bursts: edges closer than BURST_GAP_MS are counted as one burst, so a clean
magnet pass is two bursts (it arrives, it leaves) and a chattering one is the
same two bursts with many edges each. velocity_report counts rising edges and
drops gaps under 1 ms, so it sees about one edge per burst that contains one.

Set the pin and the wheel below; keep the wheel equal to
params/velocity_report.yaml so the speeds compare. Stop velocity_report first:
both want the pin's event line.
"""
import ctypes
import fcntl
import math
import os
import select
import sys
import time

import Jetson.GPIO as GPIO
from Jetson.GPIO import gpio as jgpio
from Jetson.GPIO import gpio_cdev as cdev

# Configuration
PIN = 16                    # GPIO pin (BOARD numbering)
WHEEL_DIAMETER_CM = 10.5    # Wheel diameter in centimeters
MARKERS_PER_ROTATION = 12   # Number of markers per wheel rotation
BURST_GAP_MS = 1.0          # edges closer than this belong to one burst
REFRESH_S = 0.1             # status line refresh period

METRES_PER_MARKER = math.pi * WHEEL_DIAMETER_CM / 100.0 / MARKERS_PER_ROTATION
EVENT_SIZE = ctypes.sizeof(cdev.gpioevent_data)


def open_event_line(pin):
    """(chip fd, event line fd) for BOARD ``pin``, requesting both edges."""
    GPIO.setmode(GPIO.BOARD)
    info = jgpio._channel_to_info(pin, need_gpio=True)
    chip_fd = cdev.chip_open_by_label(info.gpio_chip)
    request = cdev.request_event(info.line_offset, cdev.GPIOEVENT_REQUEST_BOTH_EDGES, 'gpio_read')
    try:
        fcntl.ioctl(chip_fd, cdev.GPIO_GET_LINEEVENT_IOCTL, request)
    except OSError as e:
        os.close(chip_fd)
        sys.exit(f'pin {pin} ({info.gpio_chip} line {info.line_offset}): {e.strerror}; '
                 'is velocity_report (or another reader) holding it?')
    return chip_fd, request.fd, info


def main():
    """Refresh the status line until Ctrl+C."""
    show_edges = '--edges' in sys.argv[1:]
    chip_fd, fd, info = open_event_line(PIN)
    level = cdev.get_value(fd)
    print(f'pin {PIN} ({info.gpio_name}, {info.gpio_chip} line {info.line_offset}): start level '
          f'{level}. Both edges; a burst is edges under {BURST_GAP_MS:g} ms apart. Ctrl+C stops.',
          flush=True)

    rising = falling = bursts = 0
    last_ns = None          # kernel time of the previous edge
    burst_start_ns = None
    burst_edges = 0
    t0 = time.monotonic()
    next_refresh = 0.0
    try:
        while True:
            ready, _, _ = select.select([fd], [], [], REFRESH_S)
            if ready:
                data = os.read(fd, EVENT_SIZE * 64)
                for i in range(0, len(data) - EVENT_SIZE + 1, EVENT_SIZE):
                    ev = cdev.gpioevent_data.from_buffer_copy(data, i)
                    up = ev.id == cdev.GPIOEVENT_REQUEST_RISING_EDGE
                    rising += up
                    falling += not up
                    gap_ms = (ev.timestamp - last_ns) / 1e6 if last_ns is not None else math.inf
                    if gap_ms >= BURST_GAP_MS:
                        if burst_edges and show_edges:
                            print(f'\r  burst: {burst_edges} edge(s) over '
                                  f'{(last_ns - burst_start_ns) / 1e6:.2f} ms' + ' ' * 20)
                        bursts += 1
                        burst_start_ns, burst_edges = ev.timestamp, 0
                    burst_edges += 1
                    last_ns = ev.timestamp
                    if show_edges:
                        gap = f'{gap_ms:9.3f} ms' if gap_ms != math.inf else '        first'
                        print(f'\r{"RISE" if up else "fall"}  gap {gap}' + ' ' * 40, flush=True)
            now = time.monotonic()
            if now >= next_refresh:
                next_refresh = now + REFRESH_S
                level = cdev.get_value(fd)
                ago = (f'{(time.clock_gettime_ns(time.CLOCK_MONOTONIC) - last_ns) / 1e9:6.2f} s ago'
                       if last_ns is not None else '   never')
                # velocity_report: about one counted edge per burst; a magnet pass is 2 bursts
                print(f'\r[{now - t0:6.1f} s] level {level} {"HIGH" if level else "LOW "}  '
                      f'rise {rising:6d}  fall {falling:6d}  bursts {bursts:5d} '
                      f'(~{bursts / 2:.0f} magnets, {bursts / 2 * METRES_PER_MARKER:.2f} m)  '
                      f'last edge {ago}', end='', flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        os.close(fd)
        os.close(chip_fd)
    print(f'\nStopped. rising {rising}, falling {falling}, bursts {bursts}', flush=True)


if __name__ == '__main__':
    main()
