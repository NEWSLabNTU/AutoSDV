#!/usr/bin/env python3
"""Wheel speed from the hall sensor, counted the way velocity_report counts it (drives nothing).

    python3 scripts/control/gpio_speed.py

One count per magnet: the sensor's edges come in bursts (it chatters at each
switch, and switches twice per magnet), and only the burst that settles at
ACTIVE_LEVEL is counted, timed at its first edge by the kernel. This is
velocity_report's own reader, autosdv_vehicle_interface.hall_sensor, so the two
cannot disagree on what a magnet is. Each line shows the speed over the last
interval and the speed from the last two magnets' gap.

Set the pin and the wheel below; keep them equal to params/velocity_report.yaml.
Stop velocity_report first: both want the pin's event line.
"""
import math
import os
import sys
import threading
import time

try:
    from autosdv_vehicle_interface.hall_sensor import GpioBurstReader
except ImportError:             # not sourced: the package's source tree
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..',
                                    'src', 'vehicle', 'autosdv_vehicle_launch',
                                    'autosdv_vehicle_interface'))
    from autosdv_vehicle_interface.hall_sensor import GpioBurstReader

# Configuration
PIN = 16                    # GPIO pin (BOARD numbering)
WHEEL_DIAMETER_CM = 10.5    # Wheel diameter in centimeters
MARKERS_PER_ROTATION = 12   # Number of markers per wheel rotation
UPDATE_RATE = 10.0          # Speed update frequency in Hz
BURST_GAP = 0.001           # s of quiet that ends a burst
ACTIVE_LEVEL = 0            # the level a burst settles at when a magnet arrives

# Calculate wheel circumference
WHEEL_CIRCUMFERENCE = (WHEEL_DIAMETER_CM / 100.0) * math.pi
METRES_PER_MARKER = WHEEL_CIRCUMFERENCE / MARKERS_PER_ROTATION

# State, shared with the reader thread
lock = threading.Lock()
count = 0
stamps = []                 # the last two magnets' times [ns]


def on_marker(stamp_ns):
    """Count one magnet."""
    global count
    with lock:
        count += 1
        stamps.append(stamp_ns)
        del stamps[:-2]


def main():
    """Print the speed at UPDATE_RATE until Ctrl+C."""
    global count
    reader = GpioBurstReader(PIN, on_marker, lambda: time.clock_gettime_ns(time.CLOCK_MONOTONIC),
                             burst_gap=BURST_GAP, active_level=ACTIVE_LEVEL, consumer='gpio_speed')
    print(f'Measuring speed on {reader.description}, one count per burst settling at '
          f'{ACTIVE_LEVEL}')
    print(f'Wheel: {WHEEL_DIAMETER_CM}cm diameter, {MARKERS_PER_ROTATION} markers/rotation '
          f'({METRES_PER_MARKER * 100:.2f} cm per marker)')
    print(f'Update rate: {UPDATE_RATE} Hz\n')

    interval = 1.0 / UPDATE_RATE
    prev_time = time.monotonic()
    total = 0
    try:
        while True:
            time.sleep(interval)

            # Take the count and reset it in one step, so no magnet is lost between
            with lock:
                markers, count = count, 0
                last_two = list(stamps)
            total += markers

            # Calculate speed
            curr_time = time.monotonic()
            elapsed = curr_time - prev_time
            prev_time = curr_time
            speed_mps = markers * METRES_PER_MARKER / elapsed
            gap = (last_two[1] - last_two[0]) / 1e9 if len(last_two) == 2 else math.inf
            gap_mps = METRES_PER_MARKER / gap

            # Display
            print(f'Speed: {speed_mps:6.3f} m/s (last gap {gap_mps:6.3f} m/s) | '
                  f'Markers: {markers:3d}  total {total:5d} ({total * METRES_PER_MARKER:6.2f} m, '
                  f'{total / MARKERS_PER_ROTATION:5.2f} rev) | bursts {reader.detector.bursts}',
                  flush=True)

    except KeyboardInterrupt:
        print("\nStopped")
    finally:
        reader.close()


if __name__ == "__main__":
    main()
