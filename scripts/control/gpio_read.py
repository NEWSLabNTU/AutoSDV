#!/usr/bin/env python3
"""Read GPIO pin state and events."""
import time
import Jetson.GPIO as GPIO

# Configuration
PIN = 15  # GPIO pin (BOARD numbering)

# State
count = 0
last_state = None

def on_event(channel):
    """Callback on GPIO event."""
    global count
    count += 1
    print(f"Event #{count:6d} detected on pin {channel}")

def main():
    global last_state

    # Setup GPIO
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(PIN, GPIO.IN)
    GPIO.add_event_detect(PIN, GPIO.RISING, callback=on_event)

    print(f"Reading GPIO pin {PIN}")
    print("Press Ctrl+C to stop\n")

    try:
        while True:
            state = GPIO.input(PIN)
            if state != last_state:
                print(f"State: {state} ({'HIGH' if state else 'LOW'})")
                last_state = state
            time.sleep(0.01)  # 10ms polling

    except KeyboardInterrupt:
        print(f"\nStopped. Total events: {count}")
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()
