#!/usr/bin/env python3
"""
Trajectory Player - Simple trajectory playback node

Reads a trajectory from a YAML file and publishes control commands
according to the schedule. No feedback control - just open-loop replay.

USAGE:
  ros2 run control_test trajectory_player \\
      --ros-args -p trajectory_file:=speed_tracking_medium.yaml

TOPICS:
Subscribed:
  - /vehicle/status/velocity_status (VelocityReport): Current vehicle speed (for logging only)

Published:
  - /control/command/control_cmd (Control): Velocity commands from trajectory
  - /control_test/status (Float32MultiArrayStamped): Status data for evaluation

PARAMETERS:
  - trajectory_file: Path to trajectory YAML file (default: speed_tracking_medium.yaml)
  - control_rate: Publishing rate in Hz (default: 20.0)
"""

import rclpy
from rclpy.node import Node
from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import VelocityReport

import os
import sys
import math
import yaml
import bisect
from dataclasses import dataclass
from typing import List
from ament_index_python.packages import get_package_share_directory


# ============================================================================
# TRAJECTORY UTILITIES
# ============================================================================

@dataclass
class TrajectoryPoint:
    """Single point in a trajectory"""
    time: float                          # Time from start (seconds)
    target_speed: float                  # Target longitudinal velocity (m/s)
    steering_angle_deg: float = 0.0      # Target steering angle (degrees)

    def __repr__(self):
        return f"TrajectoryPoint(t={self.time:.2f}s, v={self.target_speed:.2f}m/s, steer={self.steering_angle_deg:.1f}°)"


class TrajectoryPlayer:
    """
    Plays back a trajectory profile loaded from YAML file.

    YAML Format:
    ```yaml
    # Speed only (straight line):
    waypoints:
      - {time: 0.0, speed: 0.0}
      - {time: 2.0, speed: 1.5}
      - {time: 5.0, speed: 1.5}
      - {time: 7.0, speed: 0.0}

    # Speed + steering angle (turning):
    waypoints:
      - {time: 0.0, speed: 0.0, steering_angle: 0.0}
      - {time: 2.0, speed: 1.0, steering_angle: 10.0}  # 10 degrees left
      - {time: 5.0, speed: 1.0, steering_angle: -5.0}  # 5 degrees right
      - {time: 8.0, speed: 1.0, steering_angle: 0.0}   # Straight
    ```

    Note: steering_angle is in degrees (auto-converted to radians when publishing Control messages)
    """

    def __init__(self, waypoints: List[TrajectoryPoint]):
        """
        Initialize trajectory player with waypoints.

        Args:
            waypoints: List of TrajectoryPoint objects, must be sorted by time
        """
        if not waypoints:
            raise ValueError("Trajectory must have at least one waypoint")

        # Ensure sorted by time
        self.waypoints = sorted(waypoints, key=lambda p: p.time)
        self.duration = self.waypoints[-1].time

    @classmethod
    def from_yaml_file(cls, filepath: str) -> 'TrajectoryPlayer':
        """
        Load trajectory from YAML file.

        Args:
            filepath: Path to YAML file

        Returns:
            TrajectoryPlayer instance

        Raises:
            FileNotFoundError: If file doesn't exist
            ValueError: If YAML format is invalid
        """
        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)
        except FileNotFoundError:
            raise FileNotFoundError(f"Trajectory file not found: {filepath}")
        except yaml.YAMLError as e:
            raise ValueError(f"Invalid YAML format: {e}")

        if 'waypoints' not in data:
            raise ValueError("YAML file must contain 'waypoints' key")

        waypoints = []
        for wp_data in data['waypoints']:
            if 'time' not in wp_data or 'speed' not in wp_data:
                raise ValueError("Each waypoint must have 'time' and 'speed' fields")

            waypoint = TrajectoryPoint(
                time=float(wp_data['time']),
                target_speed=float(wp_data['speed']),
                steering_angle_deg=float(wp_data.get('steering_angle', 0.0))
            )
            waypoints.append(waypoint)

        return cls(waypoints)

    def get_target(self, time: float) -> TrajectoryPoint:
        """
        Get target values at given time using linear interpolation.

        Args:
            time: Current time since trajectory start (seconds)

        Returns:
            TrajectoryPoint with interpolated target values
        """
        # Clamp time to trajectory duration
        time = max(0.0, min(time, self.duration))

        # Find surrounding waypoints
        times = [wp.time for wp in self.waypoints]
        idx = bisect.bisect_left(times, time)

        # Handle edge cases
        if idx == 0:
            return self.waypoints[0]
        if idx >= len(self.waypoints):
            return self.waypoints[-1]

        # Linear interpolation between waypoints
        wp_before = self.waypoints[idx - 1]
        wp_after = self.waypoints[idx]

        # Interpolation factor
        dt = wp_after.time - wp_before.time
        if dt == 0:
            return wp_before

        alpha = (time - wp_before.time) / dt

        # Interpolate values
        return TrajectoryPoint(
            time=time,
            target_speed=wp_before.target_speed + alpha * (wp_after.target_speed - wp_before.target_speed),
            steering_angle_deg=wp_before.steering_angle_deg + alpha * (wp_after.steering_angle_deg - wp_before.steering_angle_deg)
        )

    def get_duration(self) -> float:
        """Get total trajectory duration in seconds"""
        return self.duration

    def __repr__(self):
        return f"TrajectoryPlayer(duration={self.duration:.1f}s, {len(self.waypoints)} waypoints)"


# ============================================================================
# ROS2 NODE
# ============================================================================


class TrajectoryPlayerNode(Node):
    """
    Simple trajectory playback node.

    Reads speed commands from trajectory file and publishes them at fixed rate.
    """

    def __init__(self):
        super().__init__('trajectory_player')

        # ===================================================================
        # PARAMETERS
        # ===================================================================
        self.declare_parameter('trajectory_file', 'speed_tracking_medium.yaml')
        self.declare_parameter('control_rate', 20.0)  # Hz

        # Get parameters
        traj_filename = self.get_parameter('trajectory_file').value
        # Backward compatibility: convert old hw1_* names
        if 'hw1' in traj_filename:
            traj_filename = traj_filename.replace('hw1_', 'speed_tracking_')

        control_rate = self.get_parameter('control_rate').value

        # Load trajectory from installed share directory
        traj_dir = os.path.join(
            get_package_share_directory('control_test'),
            'trajectories'
        )
        traj_path = os.path.join(traj_dir, traj_filename)

        try:
            self.trajectory = TrajectoryPlayer.from_yaml_file(traj_path)
            self.get_logger().info(f"Loaded trajectory: {self.trajectory}")
        except Exception as e:
            self.get_logger().error(f"Failed to load trajectory: {e}")
            sys.exit(1)

        # ===================================================================
        # STATE VARIABLES
        # ===================================================================
        self.current_velocity = 0.0      # Current speed from velocity_status (m/s)
        self.start_time = None

        # MSE computation
        self.squared_error_sum = 0.0
        self.sample_count = 0

        # ===================================================================
        # ROS2 INTERFACES
        # ===================================================================

        # Subscriber for velocity feedback (logging only)
        self.velocity_sub = self.create_subscription(
            VelocityReport,
            '/vehicle/status/velocity_status',
            self.velocity_callback,
            10
        )

        # Publishers
        self.control_pub = self.create_publisher(
            Control,
            '/control/command/control_cmd',
            10
        )

        # Control timer
        control_period = 1.0 / control_rate
        self.control_timer = self.create_timer(control_period, self.control_loop)

        self.get_logger().info(f"Trajectory player initialized")
        self.get_logger().info(f"Control rate: {control_rate} Hz")

    def velocity_callback(self, msg: VelocityReport):
        """Store current velocity for logging"""
        self.current_velocity = msg.longitudinal_velocity

    def control_loop(self):
        """
        Main control loop - publishes commands from trajectory at fixed rate.
        """
        current_time = self.get_clock().now()

        # Initialize start time
        if self.start_time is None:
            self.start_time = current_time
            return

        # Compute elapsed time
        elapsed = (current_time - self.start_time).nanoseconds / 1e9

        # Get target from trajectory
        target = self.trajectory.get_target(elapsed)

        # Convert steering angle from degrees to radians
        steering_angle_rad = math.radians(target.steering_angle_deg)

        # Publish control command
        control_msg = Control()
        control_msg.stamp = current_time.to_msg()
        control_msg.longitudinal.velocity = target.target_speed
        control_msg.lateral.steering_tire_angle = steering_angle_rad

        self.control_pub.publish(control_msg)

        # Accumulate squared error for MSE computation
        speed_error = target.target_speed - self.current_velocity
        self.squared_error_sum += speed_error ** 2
        self.sample_count += 1

        # Check if trajectory is finished
        if elapsed >= self.trajectory.get_duration():
            # Compute final MSE
            if self.sample_count > 0:
                mse = self.squared_error_sum / self.sample_count
                self.get_logger().info(
                    f"\n{'='*60}\n"
                    f"Trajectory playback completed!\n"
                    f"Duration: {elapsed:.2f}s\n"
                    f"Samples: {self.sample_count}\n"
                    f"Final Speed MSE: {mse:.6f} m²/s²\n"
                    f"{'='*60}"
                )
            else:
                self.get_logger().warn("No samples collected for MSE computation")

            # Exit the node
            raise SystemExit(0)

        # Log periodically (every 2 seconds)
        if int(elapsed) % 2 == 0 and int(elapsed * 10) % 10 == 0:
            current_mse = self.squared_error_sum / self.sample_count if self.sample_count > 0 else 0.0
            self.get_logger().info(
                f"t={elapsed:.1f}s: target_speed={target.target_speed:.2f} m/s, "
                f"actual_speed={self.current_velocity:.2f} m/s, "
                f"steering={target.steering_angle_deg:.1f}°, "
                f"MSE={current_mse:.6f} m²/s²"
            )


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlayerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
