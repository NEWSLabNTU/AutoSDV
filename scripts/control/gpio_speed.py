#!/usr/bin/env python3
"""Minimalist GPIO-based speed measurement for AutoSDV."""
import math
import time
import Jetson.GPIO as GPIO

# Configuration
PIN = 15                    # GPIO pin (BOARD numbering)
WHEEL_DIAMETER_CM = 20.0    # Wheel diameter in centimeters
MARKERS_PER_ROTATION = 20   # Number of markers per wheel rotation
UPDATE_RATE = 10.0          # Speed update frequency in Hz

# Calculate wheel circumference
WHEEL_CIRCUMFERENCE = (WHEEL_DIAMETER_CM / 100.0) * math.pi

# State
count = 0

def on_marker(channel):
    """Increment count on each wheel marker detection."""
    global count
    count += 1

def main():
    global count

    # Setup GPIO
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(PIN, GPIO.IN)
    GPIO.add_event_detect(PIN, GPIO.RISING, callback=on_marker)

    print(f"Measuring speed on pin {PIN}")
    print(f"Wheel: {WHEEL_DIAMETER_CM}cm diameter, {MARKERS_PER_ROTATION} markers/rotation")
    print(f"Update rate: {UPDATE_RATE} Hz\n")

    interval = 1.0 / UPDATE_RATE
    prev_time = time.time()

    try:
        while True:
            time.sleep(interval)

            # Calculate speed
            curr_time = time.time()
            elapsed = curr_time - prev_time
            markers_per_sec = count / elapsed
            rotations_per_sec = markers_per_sec / MARKERS_PER_ROTATION
            speed_mps = rotations_per_sec * WHEEL_CIRCUMFERENCE
            speed_kmh = speed_mps * 3.6

            # Display
            print(f"Speed: {speed_mps:6.3f} m/s ({speed_kmh:6.3f} km/h) | "
                  f"Markers: {count:4d} | RPM: {rotations_per_sec * 60:6.1f}")

            # Reset for next measurement
            count = 0
            prev_time = curr_time

    except KeyboardInterrupt:
        print("\nStopped")
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()
