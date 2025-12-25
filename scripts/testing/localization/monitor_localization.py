#!/usr/bin/env python3
"""
Localization Monitoring Script
Compares GPS pose with NDT localization and shows diagnostic information
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import math


class LocalizationMonitor(Node):
    def __init__(self):
        super().__init__('localization_monitor')

        # Subscribe to localization topics
        self.gps_pose_sub = self.create_subscription(
            PoseStamped,
            '/sensing/gnss/pose',
            self.gps_pose_callback,
            10
        )

        self.ndt_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/localization/pose_estimator/pose_with_covariance',
            self.ndt_pose_callback,
            10
        )

        self.kinematic_state_sub = self.create_subscription(
            Odometry,
            '/localization/kinematic_state',
            self.kinematic_state_callback,
            10
        )

        # Latest data
        self.gps_pose = None
        self.ndt_pose = None
        self.kinematic_state = None

        # Print header
        print("\n" + "="*80)
        print("LOCALIZATION MONITORING - GPS vs NDT")
        print("="*80)
        print()

    def gps_pose_callback(self, msg):
        self.gps_pose = msg
        self.print_status()

    def ndt_pose_callback(self, msg):
        self.ndt_pose = msg
        self.print_status()

    def kinematic_state_callback(self, msg):
        self.kinematic_state = msg

    def calculate_distance(self, pose1, pose2):
        """Calculate 3D Euclidean distance between two poses"""
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        dz = pose1.position.z - pose2.position.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def calculate_2d_distance(self, pose1, pose2):
        """Calculate 2D distance (ignoring Z)"""
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        return math.sqrt(dx*dx + dy*dy)

    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle in degrees"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        return math.degrees(yaw_rad)

    def print_status(self):
        if self.gps_pose is None or self.ndt_pose is None:
            return

        gps = self.gps_pose.pose
        ndt = self.ndt_pose.pose.pose
        cov = self.ndt_pose.pose.covariance

        # Calculate differences
        dist_3d = self.calculate_distance(gps, ndt)
        dist_2d = self.calculate_2d_distance(gps, ndt)
        dz = abs(gps.position.z - ndt.position.z)

        gps_yaw = self.quaternion_to_yaw(gps.orientation)
        ndt_yaw = self.quaternion_to_yaw(ndt.orientation)
        dyaw = abs(gps_yaw - ndt_yaw)
        if dyaw > 180:
            dyaw = 360 - dyaw

        # NDT covariance
        ndt_std_x = math.sqrt(cov[0]) if cov[0] > 0 else 0.0
        ndt_std_y = math.sqrt(cov[7]) if cov[7] > 0 else 0.0
        ndt_std_z = math.sqrt(cov[14]) if cov[14] > 0 else 0.0

        # Clear screen and print
        print("\033[2J\033[H")  # Clear screen
        print("="*80)
        print("LOCALIZATION MONITORING - GPS vs NDT")
        print("="*80)
        print()

        print("GPS POSE (from GNSS):")
        print(f"  Position: X={gps.position.x:10.3f} Y={gps.position.y:10.3f} Z={gps.position.z:10.3f} m")
        print(f"  Heading:  {gps_yaw:10.2f}°")
        print()

        print("NDT POSE (from LiDAR localization):")
        print(f"  Position: X={ndt.position.x:10.3f} Y={ndt.position.y:10.3f} Z={ndt.position.z:10.3f} m")
        print(f"  Heading:  {ndt_yaw:10.2f}°")
        print(f"  Std Dev:  X={ndt_std_x:10.3f} Y={ndt_std_y:10.3f} Z={ndt_std_z:10.3f} m")
        print()

        print("DIFFERENCE (GPS - NDT):")
        print(f"  2D Distance:    {dist_2d:8.3f} m")
        print(f"  Height Diff:    {dz:8.3f} m")
        print(f"  3D Distance:    {dist_3d:8.3f} m")
        print(f"  Heading Diff:   {dyaw:8.2f}°")
        print()

        # Status assessment
        print("STATUS:")
        if dist_2d < 2.0:
            print("  ✓ GPS and NDT are well aligned (< 2m)")
        elif dist_2d < 5.0:
            print("  ⚠ GPS and NDT have moderate difference (2-5m)")
        else:
            print("  ✗ GPS and NDT have large difference (> 5m)")
            print("    - Check if GPS has good satellite fix")
            print("    - Check if NDT is converged")

        if dz < 1.0:
            print("  ✓ Height alignment is good (< 1m)")
        else:
            print(f"  ⚠ Height difference is {dz:.2f}m")
            print("    - May need to adjust pointcloud map height")
            print("    - Or check GPS altitude reference")

        if self.kinematic_state:
            kin_pose = self.kinematic_state.pose.pose
            print()
            print("FUSED KINEMATIC STATE:")
            print(f"  Position: X={kin_pose.position.x:10.3f} Y={kin_pose.position.y:10.3f} Z={kin_pose.position.z:10.3f} m")
            kin_yaw = self.quaternion_to_yaw(kin_pose.orientation)
            print(f"  Heading:  {kin_yaw:10.2f}°")

        print()
        print("="*80)
        print("Press Ctrl+C to exit")
        print("="*80)


def main(args=None):
    rclpy.init(args=args)

    try:
        monitor = LocalizationMonitor()
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\n\nShutting down localization monitor...")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
