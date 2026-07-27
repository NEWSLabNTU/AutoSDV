#!/usr/bin/env python3
"""``scan_qos_bridge`` -- republish a BEST_EFFORT LaserScan as RELIABLE.

Promoted from the runtime-generated ``tmp/scan_qos_bridge.py`` heredoc in
``scripts/2dlidar/run-particle-filter.sh`` (Task 7, see
``docs/superpowers/plans/2026-07-27-2dlidar-phase-5-mcl-as-pose-source.md``)
into a real package node, unchanged in behaviour.

Why this exists: ``particle_filter``'s ``LaserScan`` subscription uses
rclpy's default (depth-only) QoS, which resolves to RELIABLE/VOLATILE.
``pointcloud_to_laserscan``'s output publisher is hardcoded to
``rclcpp::SensorDataQoS`` (BEST_EFFORT). Those two are incompatible per
ROS 2 QoS matching rules (a RELIABLE reader cannot receive from a
BEST_EFFORT writer) -- confirmed via ``ros2 topic info /scan --verbose``
and the particle_filter log ("New publisher discovered on topic '/scan',
offering incompatible QoS... Last incompatible policy: RELIABILITY") during
the original Phase 3 investigation. This node sits between them: subscribe
SensorDataQoS on the input topic (default ``/scan_raw``), republish
RELIABLE on the output topic (default ``/scan``).
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class ScanQosBridge(Node):
    def __init__(self):
        super().__init__('scan_qos_bridge')

        self.declare_parameter('input_topic', '/scan_raw')
        self.declare_parameter('output_topic', '/scan')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5)
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        self.pub = self.create_publisher(LaserScan, output_topic, reliable_qos)
        self.sub = self.create_subscription(
            LaserScan, input_topic, self.pub.publish, sensor_qos)

        self.get_logger().info(
            f'scan_qos_bridge: {input_topic} (BEST_EFFORT) -> '
            f'{output_topic} (RELIABLE)')


def main(args=None):
    rclpy.init(args=args)
    node = ScanQosBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
