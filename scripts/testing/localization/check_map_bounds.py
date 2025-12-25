#!/usr/bin/env python3
"""
Map Boundary Checker
Analyzes the point cloud map and checks if GPS position is within map bounds
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
import struct
import sys


class MapBoundsChecker(Node):
    def __init__(self):
        super().__init__('map_bounds_checker')

        # Map bounds (will be calculated from point cloud)
        self.map_bounds = None
        self.map_analyzed = False

        # Subscribe to topics
        self.map_sub = self.create_subscription(
            PointCloud2,
            '/map/pointcloud_map',
            self.map_callback,
            10
        )

        self.gps_pose_sub = self.create_subscription(
            PoseStamped,
            '/sensing/gnss/pose',
            self.gps_pose_callback,
            10
        )

        print("\n" + "="*80)
        print("MAP BOUNDARY CHECKER")
        print("="*80)
        print("Waiting for map data...")
        print()

    def map_callback(self, msg):
        """Analyze point cloud map to determine bounds"""
        if self.map_analyzed:
            return

        print("Analyzing point cloud map...")

        # Parse point cloud data
        point_step = msg.point_step
        min_x = min_y = min_z = float('inf')
        max_x = max_y = max_z = float('-inf')

        # Sample every Nth point to speed up analysis
        sample_rate = 10
        points_analyzed = 0

        for i in range(0, len(msg.data), point_step * sample_rate):
            if i + 12 > len(msg.data):
                break

            # Extract x, y, z (assuming float32 format)
            x = struct.unpack('f', msg.data[i:i+4])[0]
            y = struct.unpack('f', msg.data[i+4:i+8])[0]
            z = struct.unpack('f', msg.data[i+8:i+12])[0]

            # Skip invalid points
            if abs(x) > 1000 or abs(y) > 1000 or abs(z) > 1000:
                continue

            min_x = min(min_x, x)
            max_x = max(max_x, x)
            min_y = min(min_y, y)
            max_y = max(max_y, y)
            min_z = min(min_z, z)
            max_z = max(max_z, z)
            points_analyzed += 1

        if points_analyzed > 0:
            self.map_bounds = {
                'min_x': min_x, 'max_x': max_x,
                'min_y': min_y, 'max_y': max_y,
                'min_z': min_z, 'max_z': max_z
            }
            self.map_analyzed = True

            print()
            print("="*80)
            print("MAP BOUNDS ANALYSIS")
            print("="*80)
            print(f"Points analyzed: {points_analyzed}")
            print()
            print(f"X Range: {min_x:10.3f} to {max_x:10.3f} m  (width: {max_x-min_x:8.3f} m)")
            print(f"Y Range: {min_y:10.3f} to {max_y:10.3f} m  (depth: {max_y-min_y:8.3f} m)")
            print(f"Z Range: {min_z:10.3f} to {max_z:10.3f} m  (height: {max_z-min_z:8.3f} m)")
            print()
            print("Map center approximately at:")
            print(f"  X: {(min_x + max_x)/2:10.3f} m")
            print(f"  Y: {(min_y + max_y)/2:10.3f} m")
            print(f"  Z: {(min_z + max_z)/2:10.3f} m")
            print("="*80)
            print()

    def gps_pose_callback(self, msg):
        """Check if GPS position is within map bounds"""
        if not self.map_analyzed:
            return

        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        bounds = self.map_bounds

        # Check if within bounds
        in_x = bounds['min_x'] <= x <= bounds['max_x']
        in_y = bounds['min_y'] <= y <= bounds['max_y']
        in_z = bounds['min_z'] <= z <= bounds['max_z']
        in_bounds = in_x and in_y and in_z

        # Calculate distances to boundaries
        dx_min = x - bounds['min_x']
        dx_max = bounds['max_x'] - x
        dy_min = y - bounds['min_y']
        dy_max = bounds['max_y'] - y
        dz_min = z - bounds['min_z']
        dz_max = bounds['max_z'] - z

        # Clear screen and print
        print("\033[2J\033[H")
        print("="*80)
        print("GPS POSITION vs MAP BOUNDS")
        print("="*80)
        print()

        print("CURRENT GPS POSITION:")
        print(f"  X: {x:10.3f} m")
        print(f"  Y: {y:10.3f} m")
        print(f"  Z: {z:10.3f} m")
        print()

        print("MAP BOUNDS:")
        print(f"  X: {bounds['min_x']:10.3f} to {bounds['max_x']:10.3f} m")
        print(f"  Y: {bounds['min_y']:10.3f} to {bounds['max_y']:10.3f} m")
        print(f"  Z: {bounds['min_z']:10.3f} to {bounds['max_z']:10.3f} m")
        print()

        print("DISTANCE TO BOUNDARIES:")
        print(f"  X: {dx_min:10.3f} m from min, {dx_max:10.3f} m from max  {'✓' if in_x else '✗'}")
        print(f"  Y: {dy_min:10.3f} m from min, {dy_max:10.3f} m from max  {'✓' if in_y else '✗'}")
        print(f"  Z: {dz_min:10.3f} m from min, {dz_max:10.3f} m from max  {'✓' if in_z else '✗'}")
        print()

        print("STATUS:")
        if in_bounds:
            print("  ✓ GPS position is WITHIN map bounds")
            print("  ✓ NDT localization should be able to work")
        else:
            print("  ✗ GPS position is OUTSIDE map bounds!")
            if not in_x:
                print(f"    - X is out of range (by {min(abs(dx_min), abs(dx_max)):.2f}m)")
            if not in_y:
                print(f"    - Y is out of range (by {min(abs(dy_min), abs(dy_max)):.2f}m)")
            if not in_z:
                print(f"    - Z is out of range (by {min(abs(dz_min), abs(dz_max)):.2f}m)")
            print("  ✗ NDT localization will likely fail")
            print("  → Move vehicle closer to mapped area")

        # Height check
        if abs(dz_min) < 2.0 and abs(dz_max) < 2.0:
            print()
            print("  ⚠ GPS height may need adjustment:")
            print(f"    Current Z: {z:.2f}m vs Map Z range: [{bounds['min_z']:.2f}, {bounds['max_z']:.2f}]m")

        print()
        print("="*80)
        print("Press Ctrl+C to exit")
        print("="*80)


def main(args=None):
    rclpy.init(args=args)

    try:
        checker = MapBoundsChecker()
        rclpy.spin(checker)
    except KeyboardInterrupt:
        print("\n\nShutting down map bounds checker...")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
