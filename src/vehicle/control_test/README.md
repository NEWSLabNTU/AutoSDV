# Control Test Package

This package provides testing utilities for the AutoSDV vehicle control system.

## Overview

The `control_test` package contains nodes and launch files for testing various aspects of vehicle control:

- **Static PWM Test**: Direct hardware PWM control for basic testing
- **PID Speed Control**: PID-based speed controller with keyboard input
- **Control Command Service**: Service-based control command publisher
- **Speedometer Test**: Velocity measurement testing

## Nodes

### 1. `static_pwm_test`
Sets constant PWM values for motor and steering hardware testing.

**Usage:**
```bash
ros2 launch control_test static_pwm_test.launch.xml
ros2 launch control_test static_pwm_test.launch.xml motor_pwm:=380 steering_pwm:=410
```

**Parameters:**
- `motor_pwm`: Motor PWM value (370=stop, >370=forward, <370=reverse)
- `steering_pwm`: Steering PWM value (400=center, 350=left, 450=right)
- `update_rate`: Update rate in Hz (default: 10.0)

### 2. `pid_speed_control`
PID-based speed controller with keyboard input for target speed adjustment.

**⚠️ Requires TTY**: Must be run in a terminal, not through systemd or background process.

**Usage:**
```bash
ros2 launch control_test pid_tuning.launch.xml
ros2 launch control_test pid_tuning.launch.xml kp:=2.0 ki:=0.2 kd:=0.3
```

**Keyboard Controls:**
- `w/s`: Increase/Decrease target speed
- `x`: Emergency stop (target = 0)
- `a/d`: Left/Right steering
- `c`: Center steering
- `q`: Quit
- `h`: Show help

**Parameters:**
- `kp`: Proportional gain (default: 0.1)
- `ki`: Integral gain (default: 0.0)
- `kd`: Derivative gain (default: 0.1)
- `speed_step`: Speed increment per keypress in m/s (default: 0.5)
- `steering_pwm_step`: Steering PWM increment per keypress (default: 5)

### 3. `control_command_service`
Service-based control command publisher. Accepts target speed/steering via parameters and publishes to `/control/command/control_cmd` when enabled.

**Usage:**
```bash
# Launch with default values (speed=0, steering=0)
ros2 launch control_test control_command_service.launch.xml

# Launch with specific target values
ros2 launch control_test control_command_service.launch.xml target_speed:=1.5 target_steering:=0.1

# Enable publishing
ros2 service call /control_command_service_node/enable example_interfaces/srv/SetBool "{data: true}"

# Disable publishing
ros2 service call /control_command_service_node/enable example_interfaces/srv/SetBool "{data: false}"
```

**Parameters:**
- `target_speed`: Target longitudinal speed in m/s (default: 0.0)
- `target_steering`: Target steering angle in rad (default: 0.0)
- `target_acceleration`: Target acceleration in m/s² (default: 0.0)
- `publish_rate`: Control command publish rate in Hz (default: 10.0)

**Services:**
- `~/enable` (example_interfaces/srv/SetBool): Enable/disable command publishing

### 4. `keyboard_pwm_control` (Non-ROS Script)
Direct keyboard control of motor and steering PWM values.

**⚠️ Moved to scripts/**: This tool has been converted to a standalone non-ROS script.

**Location:** `/scripts/control/keyboard_pwm_control.py`

**Usage:**
```bash
# Run from AutoSDV root directory
./scripts/control/keyboard_pwm_control.py

# Or with custom step sizes
./scripts/control/keyboard_pwm_control.py 5 5  # motor_step steering_step
```

**Keyboard Controls:**
- `w/s`: Increase/Decrease motor PWM
- `x`: Emergency stop (motor PWM = 370)
- `a/d`: Left/Right steering
- `c`: Center steering
- `q`: Quit
- `h`: Show help

## Launch Files

### `control_test.launch.xml`
Full vehicle interface stack for control testing, including:
- ZED IMU sensor
- Velocity report (speedometer)
- Actuator node
- Gear manager
- Control mode manager
- Steering status
- Signal manager
- Vehicle velocity converter

**Usage:**
```bash
ros2 launch control_test control_test.launch.xml
```

### `speedometer.launch.xml`
Standalone velocity report node for testing wheel speed sensor.

**Usage:**
```bash
ros2 launch control_test speedometer.launch.xml
```

## Workflow Examples

### Testing Hardware PWM
```bash
# Set motor to forward at PWM 380, steering centered
ros2 launch control_test static_pwm_test.launch.xml motor_pwm:=380 steering_pwm:=400
```

### PID Tuning
```bash
# Launch full vehicle stack
ros2 launch control_test control_test.launch.xml

# In another terminal, launch PID controller with keyboard input
ros2 launch control_test pid_tuning.launch.xml kp:=1.5 ki:=0.1 kd:=0.2

# Use keyboard to adjust target speed and observe response
```

### Autonomous Control Testing
```bash
# Launch full vehicle stack
ros2 launch control_test control_test.launch.xml

# Launch control command service with target speed
ros2 launch control_test control_command_service.launch.xml target_speed:=1.0

# Enable publishing
ros2 service call /control_command_service_node/enable example_interfaces/srv/SetBool "{data: true}"

# Monitor velocity
ros2 topic echo /vehicle/status/velocity_status
```

## Notes

- **Keyboard controllers** (pid_speed_control) require TTY access and cannot be run in background or systemd services.
- **keyboard_pwm_control** is now a standalone script in `scripts/control/` (non-ROS).
- **Service-based control** (control_command_service) is suitable for automated testing and integration with higher-level control systems.
- **PWM values** are calibrated for AutoSDV hardware:
  - Motor: 370=stop, 390-395=forward start, 350=reverse start
  - Steering: 400=center, 350=left limit, 450=right limit

## Dependencies

- `autosdv_vehicle_interface`: Vehicle interface nodes
- `autoware_control_msgs`: Control message definitions
- `simple-pid`: Python PID controller library
- `Adafruit_PCA9685`: PWM driver library

## See Also

- [Vehicle Interface Documentation](../autosdv_vehicle_launch/autosdv_vehicle_interface/README.md)
- [Control System Testing Guide](../../../docs/control_system_testing.md)

