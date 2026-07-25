#!/usr/bin/env python3
"""Slice a z-band from a PCD point-cloud map into a 2D occupancy grid.

The output .pgm/.yaml pair keeps the PCD's metric frame: origin is the
grid's lower-left corner in map coordinates, so a grid generated from the
same PCD that the Lanelet2 map was aligned to needs no further alignment.

Encoding (map_server convention, negate: 0):
  0 = occupied, 254 = free, 205 = unknown.
A cell is occupied when >= min_points fall inside [z_min, z_max]; free when
the column has points (e.g. ground) but none in the band; unknown when the
column has no points at all.
"""
import argparse
from pathlib import Path

import numpy as np
from PIL import Image

OCCUPIED, FREE, UNKNOWN = 0, 254, 205


def read_pcd(path):
    """Read PCD v0.7 (ascii or binary), return float32 array (N, 3) = x,y,z."""
    with open(path, "rb") as f:
        fields, sizes, types, data_mode = [], [], [], None
        n_points = 0
        while True:
            line = f.readline().decode("ascii", errors="replace").strip()
            if line.startswith("#") or not line:
                continue
            key, _, rest = line.partition(" ")
            if key == "FIELDS":
                fields = rest.split()
            elif key == "SIZE":
                sizes = [int(v) for v in rest.split()]
            elif key == "TYPE":
                types = rest.split()
            elif key == "POINTS":
                n_points = int(rest)
            elif key == "DATA":
                data_mode = rest
                break
        if data_mode not in ("ascii", "binary"):
            raise ValueError(f"unsupported PCD DATA mode: {data_mode}")
        if any(t != "F" or s != 4 for t, s in zip(types, sizes)):
            raise ValueError("only all-FLOAT32 PCD files are supported")
        n_fields = len(fields)
        if data_mode == "binary":
            raw = np.frombuffer(f.read(4 * n_fields * n_points), np.float32)
            pts = raw.reshape(n_points, n_fields)
        else:
            pts = np.loadtxt(f, dtype=np.float32, max_rows=n_points)
            pts = pts.reshape(n_points, n_fields)
    idx = [fields.index(a) for a in ("x", "y", "z")]
    return np.ascontiguousarray(pts[:, idx])


def rasterize(points, z_min, z_max, resolution, min_points):
    """Return (grid uint8 [rows, cols], origin (x_min, y_min)).

    Row 0 corresponds to y_min (bottom of the map); the PGM writer flips
    vertically because image row 0 is the top.
    """
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    x_min, y_min = float(x.min()), float(y.min())
    cols = int(np.ceil((float(x.max()) - x_min) / resolution)) + 1
    rows = int(np.ceil((float(y.max()) - y_min) / resolution)) + 1
    ci = ((x - x_min) / resolution).astype(np.int64).clip(0, cols - 1)
    ri = ((y - y_min) / resolution).astype(np.int64).clip(0, rows - 1)
    flat = ri * cols + ci

    any_count = np.bincount(flat, minlength=rows * cols)
    band = (z >= z_min) & (z <= z_max)
    band_count = np.bincount(flat[band], minlength=rows * cols)

    grid = np.full(rows * cols, UNKNOWN, np.uint8)
    grid[any_count > 0] = FREE
    grid[band_count >= min_points] = OCCUPIED
    return grid.reshape(rows, cols), (x_min, y_min)


def write_map(grid, origin, resolution, prefix: Path):
    pgm = prefix.with_suffix(".pgm")
    yml = prefix.with_suffix(".yaml")
    Image.fromarray(np.flipud(grid)).save(pgm)   # image row 0 = top = y_max
    yml.write_text(
        f"image: {pgm.name}\n"
        f"resolution: {resolution}\n"
        f"origin: [{origin[0]:.3f}, {origin[1]:.3f}, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n"
    )
    return pgm, yml


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("pcd", type=Path)
    ap.add_argument("out_prefix", type=Path)
    ap.add_argument("--z-min", type=float, required=True)
    ap.add_argument("--z-max", type=float, required=True)
    ap.add_argument("--resolution", type=float, default=0.05)
    ap.add_argument("--min-points", type=int, default=2)
    args = ap.parse_args()

    pts = read_pcd(args.pcd)
    print(f"{len(pts)} points, z range [{pts[:,2].min():.2f}, {pts[:,2].max():.2f}]")
    grid, origin = rasterize(pts, args.z_min, args.z_max,
                             args.resolution, args.min_points)
    occ = int((grid == OCCUPIED).sum())
    free = int((grid == FREE).sum())
    print(f"grid {grid.shape[1]}x{grid.shape[0]} cells, "
          f"{occ} occupied, {free} free")
    pgm, yml = write_map(grid, origin, args.resolution, args.out_prefix)
    print(f"wrote {pgm} and {yml}")


if __name__ == "__main__":
    main()
