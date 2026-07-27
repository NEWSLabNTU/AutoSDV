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
import sys
from pathlib import Path

import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).parent))
import map_sidecar

OCCUPIED, FREE, UNKNOWN = 0, 254, 205


def read_pcd(path):
    """Read PCD v0.7 (ascii or binary), return float32 array (N, 3) = x,y,z.

    Supports mixed field types (e.g., x/y/z as F, padding as U). x, y, z must
    be FLOAT32; other fields are skipped.
    """
    with open(path, "rb") as f:
        fields, sizes, types, counts, data_mode = [], [], [], [], None
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
            elif key == "COUNT":
                counts = [int(v) for v in rest.split()]
            elif key == "POINTS":
                n_points = int(rest)
            elif key == "DATA":
                data_mode = rest
                break
        if data_mode not in ("ascii", "binary"):
            raise ValueError(f"unsupported PCD DATA mode: {data_mode}")

        if not counts:
            counts = [1] * len(fields)
        if len(sizes) != len(fields):
            raise ValueError(
                f"PCD header: SIZE has {len(sizes)} entries but FIELDS has {len(fields)}"
            )
        if len(types) != len(fields):
            raise ValueError(
                f"PCD header: TYPE has {len(types)} entries but FIELDS has {len(fields)}"
            )
        if len(counts) != len(fields):
            raise ValueError(
                f"PCD header: COUNT has {len(counts)} entries but FIELDS has {len(fields)}"
            )

        # Ensure x, y, z are FLOAT32
        for field in ("x", "y", "z"):
            if field not in fields:
                raise ValueError(f"missing required field: {field}")
            idx = fields.index(field)
            if types[idx] != "F" or sizes[idx] != 4:
                raise ValueError(f"field {field} must be FLOAT32 (TYPE F, SIZE 4)")

        # Uniquify duplicate non-xyz field names (e.g. repeated "_" padding
        # fields) so the structured dtype below doesn't collide on name.
        # x/y/z are guaranteed unique by PCD convention and must keep their
        # exact names so they remain findable after this pass.
        seen = {}
        unique_fields = []
        for fname in fields:
            if fname in ("x", "y", "z"):
                unique_fields.append(fname)
                continue
            if fname not in seen:
                seen[fname] = 1
                unique_fields.append(fname)
            else:
                seen[fname] += 1
                unique_fields.append(f"{fname}_{seen[fname]}")
        fields = unique_fields

        n_fields = len(fields)
        if data_mode == "binary":
            # Build structured dtype from field specs
            dtype_fields = []
            for fname, fsize, ftype, fcount in zip(fields, sizes, types, counts):
                # Map PCD type to numpy dtype
                if ftype == "F":
                    base_dtype = f"f{fsize}"  # f4 for float32, f8 for float64
                elif ftype == "U":
                    base_dtype = f"u{fsize}"  # u1, u2, u4 for uint8, uint16, uint32
                elif ftype == "I":
                    base_dtype = f"i{fsize}"  # i1, i2, i4 for int8, int16, int32
                else:
                    raise ValueError(f"unsupported PCD type: {ftype}")

                if fcount > 1:
                    dtype_fields.append((fname, base_dtype, (fcount,)))
                else:
                    dtype_fields.append((fname, base_dtype))

            struct_dtype = np.dtype(dtype_fields)
            raw = np.frombuffer(f.read(struct_dtype.itemsize * n_points), struct_dtype)

            # Extract x, y, z as float32
            xyz = np.column_stack([
                raw["x"].astype(np.float32),
                raw["y"].astype(np.float32),
                raw["z"].astype(np.float32)
            ])
            return np.ascontiguousarray(xyz)
        else:
            # ASCII mode: use loadtxt and extract by column index
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


def ground_estimate(points, resolution):
    """Per-cell ground height, as the median of each column's minimum z.

    A single global z-minimum is the wrong reference on any site with a slope
    or a basement: the band has to sit above the ground *locally*. Taking each
    grid column's lowest point and then the median across columns gives a
    ground level robust to both outliers below the surface and to tall
    structures above it.
    """
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    x_min, y_min = float(x.min()), float(y.min())
    cols = int(np.ceil((float(x.max()) - x_min) / resolution)) + 1
    ci = ((x - x_min) / resolution).astype(np.int64).clip(0, cols - 1)
    ri = ((y - y_min) / resolution).astype(np.int64)
    flat = ri * cols + ci
    order = np.argsort(flat, kind="stable")
    flat_sorted, z_sorted = flat[order], z[order]
    starts = np.flatnonzero(np.r_[True, flat_sorted[1:] != flat_sorted[:-1]])
    per_cell_min = np.minimum.reduceat(z_sorted, starts)
    return float(np.median(per_cell_min))


def suggest_band(points, resolution):
    """A suggested (z_min, z_max) sitting just above the estimated ground.

    0.2-0.5 m above ground is the band that worked on this project's sites: high
    enough to clear kerbs and ground noise, low enough to catch walls, parked
    cars and posts rather than tree canopy or ceilings.
    """
    ground = ground_estimate(points, resolution)
    return ground + 0.2, ground + 0.5


def format_band_guidance(points, resolution):
    """The text printed when no band was given. Pure, so it is testable."""
    z = points[:, 2]
    ground = ground_estimate(points, resolution)
    lo, hi = suggest_band(points, resolution)
    percentiles = [1, 5, 10, 25, 50, 75, 90, 95, 99]
    values = np.percentile(z, percentiles)
    lines = [
        "No z band given, and this tool will not guess one.",
        "",
        "The band selects which heights count as obstacles. Getting it wrong",
        "does not fail loudly: it produces a valid-looking grid that localizes",
        "badly (this project once shipped a 230-cell grid that way).",
        "",
        f"z distribution over {len(z)} points:",
    ]
    lines += [f"  p{p:<3d} {v:9.2f} m" for p, v in zip(percentiles, values)]
    lines += [
        f"  min  {float(z.min()):9.2f} m",
        f"  max  {float(z.max()):9.2f} m",
        "",
        f"estimated ground (median of per-cell minimum z): {ground:.2f} m",
        "",
        "Suggested band, 0.2-0.5 m above that ground estimate:",
        f"  --z-min {lo:.2f} --z-max {hi:.2f}",
        "",
        "Check it against the site: the band must clear the ground and catch",
        "walls, not canopy. Then re-run with both flags.",
    ]
    return "\n".join(lines)


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
    # Not required: omitting both prints the z distribution and a suggested
    # band, then exits without writing. Silently picking a band is how the
    # unusable Phase 1 grid happened.
    ap.add_argument("--z-min", type=float)
    ap.add_argument("--z-max", type=float)
    ap.add_argument("--resolution", type=float, default=0.05)
    ap.add_argument("--min-points", type=int, default=2)
    ap.add_argument("--sidecar", action="store_true",
                    help="write/update autosdv_map.yaml beside the output grid")
    args = ap.parse_args()

    pts = read_pcd(args.pcd)
    print(f"{len(pts)} points, z range [{pts[:,2].min():.2f}, {pts[:,2].max():.2f}]")

    if args.z_min is None or args.z_max is None:
        if args.z_min is not None or args.z_max is not None:
            print("error: --z-min and --z-max must be given together.\n",
                  file=sys.stderr)
        print(format_band_guidance(pts, args.resolution), file=sys.stderr)
        return 2
    if args.z_max <= args.z_min:
        print(f"error: --z-max ({args.z_max}) must exceed --z-min "
              f"({args.z_min}).", file=sys.stderr)
        return 2

    grid, origin = rasterize(pts, args.z_min, args.z_max,
                             args.resolution, args.min_points)
    occ = int((grid == OCCUPIED).sum())
    free = int((grid == FREE).sum())
    print(f"grid {grid.shape[1]}x{grid.shape[0]} cells, "
          f"{occ} occupied, {free} free")
    if occ == 0:
        print("warning: no occupied cells -- the band caught nothing. "
              "Re-run without --z-min/--z-max to see the distribution.",
              file=sys.stderr)
    pgm, yml = write_map(grid, origin, args.resolution, args.out_prefix)
    print(f"wrote {pgm} and {yml}")

    if args.sidecar:
        map_dir = args.out_prefix.parent
        pcd_ref = (args.pcd.name if args.pcd.parent == map_dir
                   else str(args.pcd))
        path = map_sidecar.save(
            map_dir,
            geometry={"pointcloud": pcd_ref, "occupancy_grid": yml.name},
            grid_provenance=map_sidecar.build_grid_provenance(
                map_sidecar.METHOD_PCD_SLICE, pcd_ref, args.resolution,
                z_band=(args.z_min, args.z_max),
                extra={"min_points": int(args.min_points)}))
        print(f"wrote {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
