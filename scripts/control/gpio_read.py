#!/usr/bin/env python3
"""Watch hall-sensor edges the way velocity_report sees them (drives nothing).

    python3 scripts/control/gpio_read.py           # a line per edge, a summary per second
    python3 scripts/control/gpio_read.py --quiet   # the summaries only

Uses the same detection as velocity_report.py: GPIO.add_event_detect with a
callback. It does not poll GPIO.input(): add_event_detect closes the pin's line
handle and requests an event line instead (Jetson.GPIO 2.1.x), so input()
afterwards reads through a stale handle. The level is read once, before.

Each edge prints its gap to the previous one and the speed that gap means.
Set the pin, the edge and the wheel below; keep the wheel equal to
params/velocity_report.yaml so the speeds compare.
"""
import math
import sys
import threading
import time

import Jetson.GPIO as GPIO

# Configuration
PIN = 16                    # GPIO pin (BOARD numbering)
EDGE = GPIO.RISING          # velocity_report counts rising edges; GPIO.BOTH checks the wiring
WHEEL_DIAMETER_CM = 10.5    # Wheel diameter in centimeters
MARKERS_PER_ROTATION = 12   # Number of markers per wheel rotation

# With BOTH, two edges per marker pass
EDGES_PER_MARKER = 2 if EDGE == GPIO.BOTH else 1
METRES_PER_EDGE = math.pi * WHEEL_DIAMETER_CM / 100.0 / MARKERS_PER_ROTATION / EDGES_PER_MARKER
EDGE_NAME = {GPIO.RISING: 'rising', GPIO.FALLING: 'falling', GPIO.BOTH: 'both'}[EDGE]

# State, shared with the callback thread
lock = threading.Lock()
count = 0
window = 0
last_t = None
quiet = '--quiet' in sys.argv[1:]


def on_edge(channel):
    """Count an edge and print its gap."""
    global count, window, last_t
    t = time.monotonic()
    with lock:
        count += 1
        window += 1
        n, last = count, last_t
        last_t = t
    if quiet:
        return
    if last is None:
        print(f'edge #{n:6d} pin {channel}', flush=True)
    else:
        gap = t - last
        print(f'edge #{n:6d} pin {channel}  gap {gap * 1e3:8.2f} ms  '
              f'{METRES_PER_EDGE / gap:6.3f} m/s', flush=True)


def main():
    """Print a summary per second until Ctrl+C."""
    global window

    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(PIN, GPIO.IN)
    level = GPIO.input(PIN)
    GPIO.add_event_detect(PIN, EDGE, callback=on_edge)

    print(f'pin {PIN}: start level {level} ({"HIGH" if level else "LOW"}), '
          f'{EDGE_NAME} edges, {METRES_PER_EDGE * 100:.2f} cm per edge. Ctrl+C stops.', flush=True)

    t0 = time.monotonic()
    try:
        while True:
            time.sleep(1.0)
            with lock:
                k, window = window, 0
                total = count
            print(f'[{time.monotonic() - t0:6.1f} s] {k:4d} edges/s  '
                  f'{k * METRES_PER_EDGE:6.3f} m/s  total {total} '
                  f'({total * METRES_PER_EDGE:.2f} m)', flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
    print(f'\nStopped. {count} edges, {count * METRES_PER_EDGE:.2f} m', flush=True)


if __name__ == '__main__':
    main()
