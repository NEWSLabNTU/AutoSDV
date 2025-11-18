#!/usr/bin/env python3
"""
Control Command Service Node

Provides a simple service interface to set desired speed and steering,
then repeatedly publishes the control command to /control/command/control_cmd.

This is useful for testing the control pipeline without keyboard interaction.
"""

import rclpy
from rclpy.node import Node
from autoware_control_msgs.msg import Control
from std_srvs.srv import SetBool
from example_interfaces.srv import SetBool as ExampleSetBool

class ControlCommandService(Node):
    """
    Service node that accepts control commands and publishes them repeatedly.
    """

    def __init__(self):
        super().__init__('control_command_service_node')

        # Declare parameters
        self.declare_parameter('target_speed', 0.0)  # m/s
        self.declare_parameter('target_steering', 0.0)  # rad
        self.declare_parameter('target_acceleration', 0.0)  # m/s^2
        self.declare_parameter('publish_rate', 10.0)  # Hz

        # Get parameters
        self.target_speed = self.get_parameter('target_speed').value
        self.target_steering = self.get_parameter('target_steering').value
        self.target_acceleration = self.get_parameter('target_acceleration').value
        publish_rate = self.get_parameter('publish_rate').value

        # Control command publisher
        self.control_pub = self.create_publisher(
            Control,
            '/control/command/control_cmd',
            10
        )

        # Enable/disable publishing service
        self.enabled = False
        self.enable_service = self.create_service(
            ExampleSetBool,
            '~/enable',
            self.enable_callback
        )

        # Periodic publisher timer
        self.timer = self.create_timer(
            1.0 / publish_rate,
            self.publish_control_cmd
        )

        self.get_logger().info('Control Command Service initialized')
        self.get_logger().info(f'Target: speed={self.target_speed:.2f} m/s, '
                             f'steering={self.target_steering:.3f} rad, '
                             f'acceleration={self.target_acceleration:.2f} m/s^2')
        self.get_logger().info('Call ~/enable service to start/stop publishing')
        self.get_logger().info('  ros2 service call ~/enable example_interfaces/srv/SetBool "{data: true}"')

    def enable_callback(self, request, response):
        """Enable or disable command publishing."""
        self.enabled = request.data
        response.success = True
        
        if self.enabled:
            response.message = f'Publishing enabled: speed={self.target_speed:.2f} m/s'
            self.get_logger().info(f'Publishing ENABLED: {response.message}')
        else:
            response.message = 'Publishing disabled'
            self.get_logger().info('Publishing DISABLED')
        
        return response

    def publish_control_cmd(self):
        """Publish control command if enabled."""
        if not self.enabled:
            return

        msg = Control()
        msg.stamp = self.get_clock().now().to_msg()
        
        # Longitudinal control
        msg.longitudinal.speed = float(self.target_speed)
        msg.longitudinal.acceleration = float(self.target_acceleration)
        
        # Lateral control
        msg.lateral.steering_tire_angle = float(self.target_steering)
        msg.lateral.steering_tire_rotation_rate = 0.0
        
        self.control_pub.publish(msg)

    def update_target(self, speed=None, steering=None, acceleration=None):
        """Update target values dynamically (for future service extensions)."""
        if speed is not None:
            self.target_speed = speed
        if steering is not None:
            self.target_steering = steering
        if acceleration is not None:
            self.target_acceleration = acceleration
        
        self.get_logger().info(f'Target updated: speed={self.target_speed:.2f} m/s, '
                             f'steering={self.target_steering:.3f} rad, '
                             f'acceleration={self.target_acceleration:.2f} m/s^2')


def main(args=None):
    """Main function."""
    rclpy.init(args=args)

    node = None
    try:
        node = ControlCommandService()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

