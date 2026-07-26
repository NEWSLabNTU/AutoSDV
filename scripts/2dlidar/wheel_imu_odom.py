#!/usr/bin/env python3
# scripts/2dlidar/wheel_imu_odom.py
"""Planar wheel+IMU odometry for bags that carry no /odom.

Integrates VelocityReport.longitudinal_velocity with the IMU yaw rate into a
unicycle-model nav_msgs/Odometry on /odom (frame odom -> base_link). Built for
particle_filter's motion model during rosbag replay; intentionally minimal —
no TF broadcast, no covariance tuning (PF uses pose deltas only).

`imu_yaw_sign` (double, default 1.0): multiplies angular_velocity.z before
integration. Some IMUs (e.g. the sample_sensor_kit Tamagawa unit) report
z-axis rate in a z-down (NED-style) mounting convention, which is
sign-inverted relative to the map yaw convention this integrator assumes
(z-up, positive=CCW). Root-caused via dead-reckoning: GT total yaw over a
28s run was +1.486 rad vs IMU-integrated -1.535 rad; negating omega drops
final dead-reckoning error from 229.6 m to 4.05 m (mean 0.79 m). Setting
`imu_yaw_sign:=-1.0` compensates for exactly this case. The correct
long-term fix is a TF-aware axis mapping (transform the IMU's angular
velocity vector through its static orientation relative to base_link
instead of assuming raw z passthrough); that is deferred — this parameter
is a pragmatic workaround for the one IMU mounting convention seen so far.
"""
import math

# ROS imports are only needed to run the node; the pure integrate_step()
# function below must stay importable in a plain (non-ROS-sourced) Python
# env so unit tests can run without sourcing Autoware/ROS setup files.
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
            # Phase 3c Lever 4 (AMCL cross-check): AMCL is a lifecycle node
            # that requires a live odom -> base_link TF (it has no other way
            # to advance particles between scans -- it does not subscribe to
            # /odom directly). This node only ever published an /odom TOPIC
            # before, so AMCL would see a TF tree with a missing link.
            # publish_tf (default False) broadcasts the same integrated pose
            # as a TF transform in addition to the existing /odom topic;
            # default-off so every existing caller (particle_filter, which
            # never used TF) is byte-identical to before this parameter was
            # added.
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


def main():
    if rclpy is None:
        raise ImportError(f"ROS environment not sourced or package missing: {_IMPORT_ERROR}")
    rclpy.init()
    node = WheelImuOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
