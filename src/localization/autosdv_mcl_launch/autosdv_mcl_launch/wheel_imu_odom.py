#!/usr/bin/env python3
"""``mcl_wheel_imu_odom`` -- planar wheel+IMU odometry for the ``mcl`` pose
source, ported (verbatim logic) from ``scripts/2dlidar/wheel_imu_odom.py``
into a real package node (Task 7).

The original script stays under ``scripts/2dlidar/`` as the standalone
experiment harness used by ``run-particle-filter.sh``; this copy is what
``autosdv_mcl_launch``'s launch file starts under ``pose_source:=mcl``. Kept
as a duplicate (not a shared import) so the harness script continues to run
outside a colcon workspace exactly as before -- see the module docstring of
the original for the full derivation of ``imu_yaw_sign``.

Integrates ``VelocityReport.longitudinal_velocity`` with the IMU yaw rate
into a unicycle-model ``nav_msgs/Odometry`` on ``/odom`` (frame
``odom -> base_link``). Intentionally minimal: no TF broadcast by default,
no covariance tuning (the particle filter's motion model uses pose deltas
only).

``imu_yaw_sign`` (double, default 1.0): multiplies ``angular_velocity.z``
before integration. The sample_sensor_kit Tamagawa IMU reports z-axis rate
in a sign convention inverted relative to the map-yaw convention this
integrator assumes; ``imu_yaw_sign:=-1.0`` compensates (see Phase 3/4
reports for the dead-reckoning root-cause numbers). This package's launch
file defaults to ``-1.0`` to match the Phase 4 gate configuration.
"""
import math

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from tf2_ros import TransformBroadcaster

    from autoware_vehicle_msgs.msg import VelocityReport
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
except ImportError as e:  # pytest runs outside a ROS env; node runtime must fail loudly
    rclpy = None
    _IMPORT_ERROR = e
else:
    _IMPORT_ERROR = None


def integrate_step(x, y, theta, v, omega, dt):
    """One unicycle step; exact-enough Euler for 20 Hz updates."""
    if dt <= 0.0:
        return (x, y, theta)
    x = x + v * math.cos(theta) * dt
    y = y + v * math.sin(theta) * dt
    theta = theta + omega * dt
    return (x, y, theta)


if rclpy is not None:

    class WheelImuOdom(Node):
        def __init__(self):
            super().__init__("wheel_imu_odom")
            self.declare_parameter("velocity_topic", "/vehicle/status/velocity_status")
            self.declare_parameter("imu_topic", "/sensing/camera/zedxm/imu/data")
            self.declare_parameter("odom_topic", "/odom")
            self.declare_parameter("imu_yaw_sign", 1.0)
            self.declare_parameter("publish_tf", False)

            self.x = 0.0
            self.y = 0.0
            self.theta = 0.0
            self.omega = 0.0
            self.last_stamp = None

            self.tf_broadcaster = TransformBroadcaster(self)

            qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(
                Imu, self.get_parameter("imu_topic").value, self.on_imu, qos)
            self.create_subscription(
                VelocityReport, self.get_parameter("velocity_topic").value,
                self.on_velocity, qos)
            self.pub = self.create_publisher(
                Odometry, self.get_parameter("odom_topic").value, 10)

        def on_imu(self, msg: "Imu"):
            sign = self.get_parameter("imu_yaw_sign").value
            self.omega = sign * msg.angular_velocity.z

        def on_velocity(self, msg: "VelocityReport"):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if self.last_stamp is not None:
                dt = stamp - self.last_stamp
                if 0.0 < dt < 1.0:  # skip replay loops/jumps
                    self.x, self.y, self.theta = integrate_step(
                        self.x, self.y, self.theta,
                        msg.longitudinal_velocity, self.omega, dt)
            self.last_stamp = stamp

            odom = Odometry()
            odom.header.stamp = msg.header.stamp
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
            odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
            odom.twist.twist.linear.x = msg.longitudinal_velocity
            odom.twist.twist.angular.z = self.omega
            self.pub.publish(odom)

            if self.get_parameter("publish_tf").value:
                tf_msg = TransformStamped()
                tf_msg.header.stamp = msg.header.stamp
                tf_msg.header.frame_id = "odom"
                tf_msg.child_frame_id = "base_link"
                tf_msg.transform.translation.x = self.x
                tf_msg.transform.translation.y = self.y
                tf_msg.transform.translation.z = 0.0
                tf_msg.transform.rotation.z = odom.pose.pose.orientation.z
                tf_msg.transform.rotation.w = odom.pose.pose.orientation.w
                self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    if rclpy is None:
        raise ImportError(f"ROS environment not sourced or package missing: {_IMPORT_ERROR}")
    rclpy.init(args=args)
    node = WheelImuOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
