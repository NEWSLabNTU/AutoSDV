#!/usr/bin/env python3
"""Minimalist GPIO-based speed measurement for AutoSDV (drives nothing).

Counts rising edges with GPIO.add_event_detect, as velocity_report.py does,
and prints the speed over each update interval. Set the pin and the wheel
below; keep the wheel equal to params/velocity_report.yaml so the speeds
compare.
"""
import math
import threading
import time

import Jetson.GPIO as GPIO

# Configuration
PIN = 15                    # GPIO pin (BOARD numbering)
WHEEL_DIAMETER_CM = 10.5    # Wheel diameter in centimeters
MARKERS_PER_ROTATION = 12   # Number of markers per wheel rotation
UPDATE_RATE = 10.0          # Speed update frequency in Hz

# Calculate wheel circumference
WHEEL_CIRCUMFERENCE = (WHEEL_DIAMETER_CM / 100.0) * math.pi

# State, shared with the callback thread
lock = threading.Lock()
count = 0


def on_marker(channel):
    """Increment count on each wheel marker detection."""
    global count
    with lock:
        count += 1


def main():
    """Print the speed at UPDATE_RATE until Ctrl+C."""
    global count

    # Setup GPIO
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(PIN, GPIO.IN)
    GPIO.add_event_detect(PIN, GPIO.RISING, callback=on_marker)

    print(f"Measuring speed on pin {PIN}")
    print(f"Wheel: {WHEEL_DIAMETER_CM}cm diameter, {MARKERS_PER_ROTATION} markers/rotation")
    print(f"Update rate: {UPDATE_RATE} Hz\n")

    interval = 1.0 / UPDATE_RATE
    prev_time = time.monotonic()

    try:
        while True:
            time.sleep(interval)

            # Take the count and reset it in one step, so no edge is lost between
            with lock:
                markers, count = count, 0

            # Calculate speed
            curr_time = time.monotonic()
            elapsed = curr_time - prev_time
            prev_time = curr_time
            rotations_per_sec = markers / elapsed / MARKERS_PER_ROTATION
            speed_mps = rotations_per_sec * WHEEL_CIRCUMFERENCE
            speed_kmh = speed_mps * 3.6

            # Display
            print(f"Speed: {speed_mps:6.3f} m/s ({speed_kmh:6.3f} km/h) | "
                  f"Markers: {markers:4d} | RPM: {rotations_per_sec * 60:6.1f}", flush=True)

    except KeyboardInterrupt:
        print("\nStopped")
    finally:
        GPIO.cleanup()


if __name__ == "__main__":
    main()
