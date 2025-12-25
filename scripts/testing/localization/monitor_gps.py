#!/usr/bin/env python3
"""
GPS Monitoring Script for Outdoor Testing
Displays GPS data and converted meter coordinates in real-time
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
import sys


class GPSMonitor(Node):
    def __init__(self):
        super().__init__('gps_monitor')

        # Map origin from COSS-map-planning/map_projector_info.yaml
        self.map_origin_lat = 25.0201
        self.map_origin_lon = 121.5423
        self.map_origin_alt = 25.0

        # Subscribe to GPS topics
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/sensing/gnss/ublox/nav_sat_fix',
            self.gps_callback,
            10
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/sensing/gnss/pose',
            self.pose_callback,
            10
        )

        # Latest data
        self.latest_gps = None
        self.latest_pose = None

        # Print header
        print("\n" + "="*80)
        print("GPS MONITORING - OUTDOOR LOCALIZATION TEST")
        print("="*80)
        print(f"Map Origin: Lat={self.map_origin_lat}, Lon={self.map_origin_lon}, Alt={self.map_origin_alt}m")
        print("="*80)
        print()

    def gps_callback(self, msg):
        self.latest_gps = msg
        self.print_status()

    def pose_callback(self, msg):
        self.latest_pose = msg

    def print_status(self):
        if self.latest_gps is None:
            return

        gps = self.latest_gps

        # GPS status
        status_str = "NO FIX"
        if gps.status.status == 0:
            status_str = "FIX"
        elif gps.status.status == 1:
            status_str = "SBAS FIX"
        elif gps.status.status == 2:
            status_str = "GBAS FIX"

        # Calculate distance from map origin
        dlat = gps.latitude - self.map_origin_lat
        dlon = gps.longitude - self.map_origin_lon
        dalt = gps.altitude - self.map_origin_alt

        # Clear screen and print
        print("\033[2J\033[H")  # Clear screen
        print("="*80)
        print(f"GPS MONITORING - {status_str}")
        print("="*80)
        print()

        print("RAW GPS DATA:")
        print(f"  Latitude:  {gps.latitude:12.8f}°  (Δ from origin: {dlat:+.8f}°)")
        print(f"  Longitude: {gps.longitude:12.8f}°  (Δ from origin: {dlon:+.8f}°)")
        print(f"  Altitude:  {gps.altitude:12.3f} m  (Δ from origin: {dalt:+.3f} m)")
        print()

        # Position covariance (uncertainty)
        cov = gps.position_covariance
        eph = (cov[0] ** 0.5) if cov[0] > 0 else 0.0  # Horizontal position uncertainty
        epv = (cov[8] ** 0.5) if cov[8] > 0 else 0.0  # Vertical position uncertainty

        print("GPS ACCURACY:")
        print(f"  Horizontal Error: {eph:8.3f} m")
        print(f"  Vertical Error:   {epv:8.3f} m")
        print()

        if self.latest_pose:
            pose = self.latest_pose.pose.position
            print("CONVERTED METER COORDINATES (from gnss_poser):")
            print(f"  X: {pose.x:12.3f} m")
            print(f"  Y: {pose.y:12.3f} m")
            print(f"  Z: {pose.z:12.3f} m")
            print()
            print("NOTE: These coordinates are in the map frame")
            print("      Origin is at the map_projector_info.yaml reference point")
        else:
            print("CONVERTED COORDINATES: Waiting for /sensing/gnss/pose...")

        print()
        print("="*80)
        print("Press Ctrl+C to exit")
        print("="*80)


def main(args=None):
    rclpy.init(args=args)

    try:
        monitor = GPSMonitor()
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\n\nShutting down GPS monitor...")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
