#!/usr/bin/env python3
"""
Keyboard PWM control - direct PWM control via keyboard (non-ROS version).
This script allows direct PWM control of motor and steering using keyboard input.

Motor PWM Characteristics:
- Stop: 390
- Forward: Starts at 395
- Backward: Starts at 385
- Brake sequence (forward to backward): PWM 340 -> 390 -> decrease to 385

Steering PWM Characteristics (REVERSED):
- Center: 400
- Left: 450 (higher PWM)
- Right: 350 (lower PWM)
"""

import sys
import termios
import tty
import select
import time
from Adafruit_PCA9685 import PCA9685


class KeyboardPWMControl:
    """
    Keyboard-based PWM control.

    Controls motor and steering PWM directly using keyboard input.
    """

    def __init__(self, motor_step=5, steering_step=5):
        """Initialize PWM control."""
        # Parameters
        self.motor_step = motor_step
        self.steering_step = steering_step

        # Initialize PCA9685 PWM driver
        try:
            self.driver = PCA9685(address=64, busnum=7)
            self.driver.set_pwm_freq(60)
            print('PCA9685 initialized successfully')
        except Exception as e:
            print(f'ERROR: Failed to initialize PCA9685: {e}')
            raise

        # PWM channels
        self.MOTOR_CHANNEL = 0
        self.STEER_CHANNEL = 1

        # Current PWM values
        self.STOP_PWM = 390
        self.CENTER_STEER = 400
        self.motor_pwm = self.STOP_PWM
        self.steering_pwm = self.CENTER_STEER

        # PWM limits
        self.MIN_MOTOR_PWM = 340
        self.MAX_MOTOR_PWM = 440
        self.MIN_STEER_PWM = 350
        self.MAX_STEER_PWM = 450

        # Set initial values
        self.set_pwm()

        # Save terminal settings for raw input
        try:
            self.old_settings = termios.tcgetattr(sys.stdin)
            self.terminal_available = True
        except termios.error:
            print('WARNING: No terminal available - keyboard input disabled')
            self.old_settings = None
            self.terminal_available = False

        self.running = True

    def print_help(self):
        """Print keyboard control help."""
        print('=== Keyboard PWM Control ===')
        print('Motor Control:')
        print('  w/s : Increase/Decrease motor PWM')
        print('  q   : Emergency stop (with brake sequence if moving forward)')
        print('Steering Control:')
        print('  a/d : Right/Left steering (reversed)')
        print('  e   : Center steering (PWM = 400)')
        print('Other:')
        print('  h   : Show this help')
        print('  Ctrl-C : Quit')
        print('')
        print('Motor PWM: 390=stop, 395+=forward, 385-=backward')
        print('Steering PWM: 400=center, 450=left, 350=right')
        print('============================')
        self.log_status()

    def set_pwm(self):
        """Set PWM values to hardware."""
        try:
            self.driver.set_pwm(self.MOTOR_CHANNEL, 0, self.motor_pwm)
            self.driver.set_pwm(self.STEER_CHANNEL, 0, self.steering_pwm)
        except Exception as e:
            print(f'ERROR: Failed to set PWM: {e}')

    def execute_brake_sequence(self):
        """Execute brake sequence (340 -> 390) to allow backward motion."""
        try:
            print('Executing brake sequence: 340 -> 390')
            self.driver.set_pwm(self.MOTOR_CHANNEL, 0, 340)
            time.sleep(0.2)
            self.driver.set_pwm(self.MOTOR_CHANNEL, 0, self.STOP_PWM)
            time.sleep(0.1)
            print('Brake sequence complete')
        except Exception as e:
            print(f'ERROR: Brake sequence failed: {e}')

    def log_status(self):
        """Log current PWM status."""
        motor_status = "STOP"
        if self.motor_pwm > self.STOP_PWM:
            motor_status = "FORWARD"
        elif self.motor_pwm < self.STOP_PWM:
            motor_status = "REVERSE"

        steer_status = "CENTER"
        if self.steering_pwm < self.CENTER_STEER:
            steer_status = "RIGHT"  # Reversed: lower PWM = right
        elif self.steering_pwm > self.CENTER_STEER:
            steer_status = "LEFT"   # Reversed: higher PWM = left

        print(
            f'Motor: {self.motor_pwm} ({motor_status}), '
            f'Steering: {self.steering_pwm} ({steer_status})'
        )

    def get_key_nonblocking(self):
        """Get a single keypress (non-blocking)."""
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None

    def process_key(self, key):
        """Process keyboard input."""
        changed = False

        # Motor control
        if key == 'w' or key == 'W':
            self.motor_pwm = min(self.motor_pwm + self.motor_step, self.MAX_MOTOR_PWM)
            changed = True
        elif key == 's' or key == 'S':
            self.motor_pwm = max(self.motor_pwm - self.motor_step, self.MIN_MOTOR_PWM)
            changed = True
        elif key == 'q' or key == 'Q':
            print('EMERGENCY STOP!')
            # If moving forward, execute brake sequence
            if self.motor_pwm > self.STOP_PWM:
                self.execute_brake_sequence()
                self.motor_pwm = self.STOP_PWM
            else:
                # Not moving forward, just stop
                self.motor_pwm = self.STOP_PWM
            changed = True

        # Steering control (reversed: a=right, d=left)
        elif key == 'a' or key == 'A':
            self.steering_pwm = min(self.steering_pwm + self.steering_step, self.MAX_STEER_PWM)
            changed = True
        elif key == 'd' or key == 'D':
            self.steering_pwm = max(self.steering_pwm - self.steering_step, self.MIN_STEER_PWM)
            changed = True
        elif key == 'e' or key == 'E':
            self.steering_pwm = self.CENTER_STEER
            changed = True

        # Other commands
        elif key == 'h' or key == 'H':
            self.print_help()

        if changed:
            self.set_pwm()
            self.log_status()

    def run(self):
        """Main control loop."""
        if not self.terminal_available:
            print('ERROR: No terminal available for keyboard input')
            return

        self.print_help()

        # Set terminal to cbreak mode for immediate keypress detection
        tty.setcbreak(sys.stdin.fileno())

        try:
            while self.running:
                # Non-blocking key read
                key = self.get_key_nonblocking()
                if key:
                    self.process_key(key)
                else:
                    # Small sleep to prevent CPU spinning
                    time.sleep(0.01)
        except KeyboardInterrupt:
            print('\nInterrupted')
        finally:
            self.cleanup()

    def cleanup(self):
        """Cleanup on shutdown."""
        # Restore terminal settings
        if self.terminal_available and self.old_settings is not None:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            except:
                pass

        # Stop motor
        print('\nStopping motor...')
        try:
            self.driver.set_pwm(self.MOTOR_CHANNEL, 0, self.STOP_PWM)
        except:
            pass


def main():
    """Main function."""
    # Parse command line arguments for step sizes if needed
    motor_step = 5
    steering_step = 5

    if len(sys.argv) > 1:
        try:
            motor_step = int(sys.argv[1])
        except ValueError:
            print(f'Invalid motor_step: {sys.argv[1]}')
            sys.exit(1)

    if len(sys.argv) > 2:
        try:
            steering_step = int(sys.argv[2])
        except ValueError:
            print(f'Invalid steering_step: {sys.argv[2]}')
            sys.exit(1)

    controller = KeyboardPWMControl(motor_step=motor_step, steering_step=steering_step)
    controller.run()


if __name__ == '__main__':
    main()
