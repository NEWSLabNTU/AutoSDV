#!/usr/bin/env python3
"""
Localization Data Logger
Records GPS and localization data to CSV file for later analysis
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import csv
import datetime
import math
import os


class LocalizationLogger(Node):
    def __init__(self, output_file):
        super().__init__('localization_logger')

        self.output_file = output_file
        self.csv_writer = None
        self.file_handle = None

        # Latest data
        self.latest_gps_raw = None
        self.latest_gps_pose = None
        self.latest_ndt_pose = None
        self.latest_kinematic = None

        # Subscribe to topics
        self.gps_raw_sub = self.create_subscription(
            NavSatFix,
            '/sensing/gnss/ublox/nav_sat_fix',
            self.gps_raw_callback,
            10
        )

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

        self.kinematic_sub = self.create_subscription(
            Odometry,
            '/localization/kinematic_state',
            self.kinematic_callback,
            10
        )

        # Initialize CSV file
        self.init_csv()

        # Create timer for periodic logging (1 Hz)
        self.timer = self.create_timer(1.0, self.log_data)

        print(f"\nLogging localization data to: {output_file}")
        print("Press Ctrl+C to stop logging\n")

    def init_csv(self):
        """Initialize CSV file with headers"""
        self.file_handle = open(self.output_file, 'w', newline='')
        self.csv_writer = csv.writer(self.file_handle)

        # Write header
        self.csv_writer.writerow([
            'timestamp',
            'gps_lat', 'gps_lon', 'gps_alt', 'gps_status',
            'gps_x', 'gps_y', 'gps_z',
            'ndt_x', 'ndt_y', 'ndt_z', 'ndt_yaw',
            'ndt_std_x', 'ndt_std_y', 'ndt_std_z',
            'kinematic_x', 'kinematic_y', 'kinematic_z', 'kinematic_yaw',
            'diff_2d', 'diff_z', 'diff_3d'
        ])
        self.file_handle.flush()

    def gps_raw_callback(self, msg):
        self.latest_gps_raw = msg

    def gps_pose_callback(self, msg):
        self.latest_gps_pose = msg

    def ndt_pose_callback(self, msg):
        self.latest_ndt_pose = msg

    def kinematic_callback(self, msg):
        self.latest_kinematic = msg

    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw in degrees"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.degrees(math.atan2(siny_cosp, cosy_cosp))

    def log_data(self):
        """Log current data to CSV"""
        if self.latest_gps_raw is None:
            return

        timestamp = datetime.datetime.now().isoformat()

        # GPS raw
        gps_lat = self.latest_gps_raw.latitude
        gps_lon = self.latest_gps_raw.longitude
        gps_alt = self.latest_gps_raw.altitude
        gps_status = self.latest_gps_raw.status.status

        # GPS pose
        gps_x = gps_y = gps_z = 0.0
        if self.latest_gps_pose:
            gps_x = self.latest_gps_pose.pose.position.x
            gps_y = self.latest_gps_pose.pose.position.y
            gps_z = self.latest_gps_pose.pose.position.z

        # NDT pose
        ndt_x = ndt_y = ndt_z = ndt_yaw = 0.0
        ndt_std_x = ndt_std_y = ndt_std_z = 0.0
        if self.latest_ndt_pose:
            ndt = self.latest_ndt_pose.pose.pose
            ndt_x = ndt.position.x
            ndt_y = ndt.position.y
            ndt_z = ndt.position.z
            ndt_yaw = self.quaternion_to_yaw(ndt.orientation)

            cov = self.latest_ndt_pose.pose.covariance
            ndt_std_x = math.sqrt(cov[0]) if cov[0] > 0 else 0.0
            ndt_std_y = math.sqrt(cov[7]) if cov[7] > 0 else 0.0
            ndt_std_z = math.sqrt(cov[14]) if cov[14] > 0 else 0.0

        # Kinematic state
        kin_x = kin_y = kin_z = kin_yaw = 0.0
        if self.latest_kinematic:
            kin = self.latest_kinematic.pose.pose
            kin_x = kin.position.x
            kin_y = kin.position.y
            kin_z = kin.position.z
            kin_yaw = self.quaternion_to_yaw(kin.orientation)

        # Calculate differences
        diff_2d = 0.0
        diff_z = 0.0
        diff_3d = 0.0
        if self.latest_gps_pose and self.latest_ndt_pose:
            dx = gps_x - ndt_x
            dy = gps_y - ndt_y
            dz = gps_z - ndt_z
            diff_2d = math.sqrt(dx*dx + dy*dy)
            diff_z = abs(dz)
            diff_3d = math.sqrt(dx*dx + dy*dy + dz*dz)

        # Write row
        self.csv_writer.writerow([
            timestamp,
            gps_lat, gps_lon, gps_alt, gps_status,
            gps_x, gps_y, gps_z,
            ndt_x, ndt_y, ndt_z, ndt_yaw,
            ndt_std_x, ndt_std_y, ndt_std_z,
            kin_x, kin_y, kin_z, kin_yaw,
            diff_2d, diff_z, diff_3d
        ])
        self.file_handle.flush()

        print(f"\rLogged: GPS({gps_lat:.6f}, {gps_lon:.6f}) -> ({gps_x:.2f}, {gps_y:.2f}, {gps_z:.2f}) | "
              f"NDT({ndt_x:.2f}, {ndt_y:.2f}, {ndt_z:.2f}) | Diff: {diff_2d:.2f}m", end='')

    def __del__(self):
        if self.file_handle:
            self.file_handle.close()


def main(args=None):
    rclpy.init(args=args)

    # Generate output filename with timestamp
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = os.path.expanduser("~/AutoSDV/localization_logs")
    os.makedirs(output_dir, exist_ok=True)
    output_file = os.path.join(output_dir, f"localization_{timestamp}.csv")

    try:
        logger = LocalizationLogger(output_file)
        rclpy.spin(logger)
    except KeyboardInterrupt:
        print(f"\n\nStopped logging. Data saved to: {output_file}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
