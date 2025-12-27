#!/usr/bin/env python3
"""
Generate a dummy PCD (Point Cloud Data) file with a 1000x1000 grid of points
distributed equally across specified longitudinal and latitude ranges.
"""

import numpy as np

def generate_pcd(output_file: str,
                 x_min: float, x_max: float,
                 y_min: float, y_max: float,
                 x_points: int = 1000,
                 y_points: int = 1000,
                 z_value: float = 0.0):
    """
    Generate a PCD file with points distributed in a grid pattern.

    Args:
        output_file: Output PCD file path
        x_min, x_max: Longitudinal range (X-axis)
        y_min, y_max: Latitude range (Y-axis)
        x_points: Number of points along X-axis
        y_points: Number of points along Y-axis
        z_value: Z coordinate value (height, default=0)
    """

    # Create evenly spaced coordinates
    x_coords = np.linspace(x_min, x_max, x_points)
    y_coords = np.linspace(y_min, y_max, y_points)

    # Create meshgrid for all combinations
    xx, yy = np.meshgrid(x_coords, y_coords)

    # Flatten to 1D arrays
    x_flat = xx.flatten()
    y_flat = yy.flatten()
    z_flat = np.full_like(x_flat, z_value)

    # Total number of points
    total_points = x_points * y_points

    print(f"Generating PCD with {total_points:,} points...")
    print(f"X range: {x_min:.2f} ~ {x_max:.2f} m ({x_points} points)")
    print(f"Y range: {y_min:.2f} ~ {y_max:.2f} m ({y_points} points)")
    print(f"Z value: {z_value:.2f} m")

    # Write PCD file
    with open(output_file, 'w') as f:
        # PCD header
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("COUNT 1 1 1\n")
        f.write(f"WIDTH {total_points}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {total_points}\n")
        f.write("DATA ascii\n")

        # Write point data
        for x, y, z in zip(x_flat, y_flat, z_flat):
            f.write(f"{x:.6f} {y:.6f} {z:.6f}\n")

    print(f"PCD file saved to: {output_file}")
    print(f"File size: {np.array([x_flat, y_flat, z_flat]).nbytes / 1024 / 1024:.2f} MB (data only)")


def main():
    # Configuration
    output_file = "/home/jetson/AutoSDV/data/COSS-map-planning/pointcloud_map_dummy.pcd"

    # Coordinate ranges (UTM or local coordinates)
    x_min = 304686.60   # Longitudinal min
    x_max = 304791.59   # Longitudinal max
    y_min = 2768074.59  # Latitude min
    y_max = 2768174.70  # Latitude max

    # Grid size
    x_points = 400
    y_points = 400

    # Generate the PCD file
    generate_pcd(
        output_file=output_file,
        x_min=x_min, x_max=x_max,
        y_min=y_min, y_max=y_max,
        x_points=x_points,
        y_points=y_points,
        z_value=0.0  # Flat ground at z=0
    )


if __name__ == "__main__":
    main()
