#!/usr/bin/env python3
"""Extrude a ROS occupancy grid into a watertight OBJ scene for a simulator.

The result is deliberately crude: occupied cells become boxes of a fixed
height, standing on one ground quad. It is metrically exact and in the map
frame, so a simulated LiDAR hitting it produces returns that NDT can match
against the PCD the grid came from -- which is the point. Fidelity comes
later, from meshing the point cloud itself.

    python3 scripts/sim/grid_to_mesh.py data/COSS-map-planning \
        --floor-z 8.9 --wall-height 2.0 -o tmp/coss_scene.obj

Run counts and areas are printed so a bad threshold is visible before the
file is imported anywhere.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


def read_map_yaml(path: Path) -> dict:
    """Minimal reader for the handful of keys map_server writes.

    Deliberately not PyYAML: this script runs on machines where the ROS
    environment has not been sourced.
    """
    out: dict = {}
    for line in path.read_text().splitlines():
        line = line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        value = value.strip()
        if value.startswith("["):
            out[key.strip()] = [float(v) for v in value.strip("[]").split(",")]
        else:
            try:
                out[key.strip()] = float(value)
            except ValueError:
                out[key.strip()] = value
    return out


def read_pgm(path: Path):
    """Read a binary (P5) PGM into a list of rows of ints, top row first."""
    data = path.read_bytes()
    fields, pos = [], 0
    while len(fields) < 4:
        while pos < len(data) and data[pos : pos + 1].isspace():
            pos += 1
        if data[pos : pos + 1] == b"#":
            while pos < len(data) and data[pos] != 0x0A:
                pos += 1
            continue
        start = pos
        while pos < len(data) and not data[pos : pos + 1].isspace():
            pos += 1
        fields.append(data[start:pos])
    if fields[0] != b"P5":
        raise SystemExit(f"{path}: only binary P5 PGM is supported, got {fields[0]!r}")
    width, height, maxval = (int(f) for f in fields[1:4])
    if maxval > 255:
        raise SystemExit(f"{path}: 16-bit PGM is not supported")
    pos += 1  # single whitespace byte after the header
    pixels = data[pos : pos + width * height]
    if len(pixels) != width * height:
        raise SystemExit(f"{path}: truncated: {len(pixels)} of {width * height} bytes")
    return width, height, pixels


def occupied_runs(width, height, pixels, negate, occupied_thresh):
    """Yield (row, x_start, x_end_exclusive) for horizontal runs of occupied cells.

    Merging along a row is what keeps the triangle count sane: a solid wall
    becomes one long box rather than one box per 5 cm cell.
    """
    cutoff = occupied_thresh * 255.0
    for row in range(height):
        base = row * width
        x = 0
        while x < width:
            value = pixels[base + x]
            occ = (value if negate else 255 - value) > cutoff
            if not occ:
                x += 1
                continue
            start = x
            while x < width:
                value = pixels[base + x]
                if not ((value if negate else 255 - value) > cutoff):
                    break
                x += 1
            yield row, start, x


def box_obj(f, xmin, xmax, ymin, ymax, zmin, zmax, base):
    """Write one axis-aligned box; returns the number of vertices written."""
    verts = [
        (xmin, ymin, zmin), (xmax, ymin, zmin), (xmax, ymax, zmin), (xmin, ymax, zmin),
        (xmin, ymin, zmax), (xmax, ymin, zmax), (xmax, ymax, zmax), (xmin, ymax, zmax),
    ]
    for vx, vy, vz in verts:
        f.write(f"v {vx:.4f} {vy:.4f} {vz:.4f}\n")
    # Triangles, not quads: some importers and most mesh libraries silently
    # drop a quad face, which turns a verified scene into an empty one.
    quads = [
        (1, 2, 3, 4), (8, 7, 6, 5), (1, 5, 6, 2),
        (2, 6, 7, 3), (3, 7, 8, 4), (4, 8, 5, 1),
    ]
    for a, b, c, d in quads:
        f.write(f"f {base+a} {base+b} {base+c}\n")
        f.write(f"f {base+a} {base+c} {base+d}\n")
    return 8


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("map_dir", type=Path, help="map directory holding occupancy_grid.yaml")
    ap.add_argument("-o", "--output", type=Path, default=Path("tmp/scene.obj"))
    ap.add_argument("--floor-z", type=float, required=True,
                    help="ground height in the map frame; read it off the point cloud")
    ap.add_argument("--wall-height", type=float, default=2.0)
    ap.add_argument("--no-floor", action="store_true", help="omit the ground quad")
    ap.add_argument("--grid", default="occupancy_grid.yaml")
    args = ap.parse_args()

    meta = read_map_yaml(args.map_dir / args.grid)
    image = args.map_dir / str(meta["image"])
    resolution = float(meta["resolution"])
    ox, oy = float(meta["origin"][0]), float(meta["origin"][1])
    negate = bool(meta.get("negate", 0))
    occupied_thresh = float(meta.get("occupied_thresh", 0.65))

    width, height, pixels = read_pgm(image)
    print(f"grid {width}x{height} @ {resolution} m = "
          f"{width*resolution:.1f} x {height*resolution:.1f} m, origin ({ox}, {oy})")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    zmin, zmax = args.floor_z, args.floor_z + args.wall_height
    runs = cells = verts = 0
    with args.output.open("w") as f:
        f.write("# generated by scripts/sim/grid_to_mesh.py\n")
        f.write(f"o {args.map_dir.name}_walls\n")
        for row, x0, x1 in occupied_runs(width, height, pixels, negate, occupied_thresh):
            # PGM row 0 is the TOP of the image, which is the HIGHEST y in the
            # map frame. Getting this backwards mirrors the whole scene and is
            # invisible until localization fails.
            y_hi = oy + (height - row) * resolution
            y_lo = y_hi - resolution
            verts += box_obj(f, ox + x0 * resolution, ox + x1 * resolution,
                             y_lo, y_hi, zmin, zmax, verts)
            runs += 1
            cells += x1 - x0
        if not args.no_floor:
            f.write(f"o {args.map_dir.name}_floor\n")
            x0, x1 = ox, ox + width * resolution
            y0, y1 = oy, oy + height * resolution
            for vx, vy in ((x0, y0), (x1, y0), (x1, y1), (x0, y1)):
                f.write(f"v {vx:.4f} {vy:.4f} {zmin:.4f}\n")
            f.write(f"f {verts+1} {verts+2} {verts+3}\n")
            f.write(f"f {verts+1} {verts+3} {verts+4}\n")
            verts += 4

    occupancy = 100.0 * cells / (width * height)
    print(f"occupied cells {cells} ({occupancy:.2f}%) merged into {runs} boxes")
    print(f"walls z {zmin:.2f}..{zmax:.2f}, {verts} vertices, "
          f"{args.output} ({args.output.stat().st_size/1e6:.1f} MB)")
    if occupancy > 40:
        print("warning: over 40% of the grid is occupied -- check occupied_thresh "
              "and negate before importing this", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
