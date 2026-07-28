#!/usr/bin/env python3
"""Normalise a sensor kit's LaserScan into MCL's input contract.

The sensor kit owns the driver and the choice of physical plane -- which ring of
a 3-D LiDAR to use depends on the vehicle height and the mounting, which is what
a kit describes. MCL takes one LaserScan in any TF-connected frame and does the
rest, so no kit author has to know MCL's internals.

Why the frame matters: `particle_filter.py` consumes ranges and bearings with no
notion of a sensor offset -- it treats the scan as originating at the particle
pose, i.e. at `base_link`. A scan published in a laser frame mounted 0.5 m
forward would therefore bias every range by 0.5 m, an error the filter cannot
distinguish from a pose error. This node re-expresses the ranges about
`base_link` so that assumption holds.

The geometry: a beam at index i ends, in the sensor frame, at
`p_L = (r*cos(theta), r*sin(theta))` with `theta = angle_min + i*angle_increment`.
Given a planar offset (dx, dy, dyaw) from base_link to the sensor, the same point
in base_link is `p_B = R(dyaw) * p_L + (dx, dy)`. Its range and bearing are
`hypot(p_B)` and `atan2(p_B)`, which are re-binned into the output scan keeping
the MINIMUM range per bin: a nearer return occludes a farther one, so taking the
maximum (or the last writer) would invent free space where an obstacle stands.

The pure functions below import no rclpy, so they are unit-testable in a plain
shell.
"""
from __future__ import annotations

import math
from typing import List, NamedTuple, Sequence, Tuple


class ScanSpec(NamedTuple):
    """Angular and radial extent of a LaserScan."""

    angle_min: float
    angle_max: float
    angle_increment: float
    range_min: float
    range_max: float

    def bin_count(self) -> int:
        if self.angle_increment <= 0.0:
            raise ValueError("angle_increment must be positive")
        return int(round((self.angle_max - self.angle_min) / self.angle_increment)) + 1


def planar_offset(translation: Sequence[float],
                  rotation_quat: Sequence[float]) -> Tuple[float, float, float, float]:
    """Reduce a 3-D transform to a planar (dx, dy, dyaw) plus a tilt magnitude.

    A 2-D scan lives in a plane, so only the planar part of the mounting
    transform can be applied to it. Roll or pitch tilts the scan plane itself,
    which no planar re-target can represent -- so the tilt is returned rather
    than silently discarded, and the caller warns on it.

    Args:
        translation: (x, y, z).
        rotation_quat: (x, y, z, w).

    Returns:
        (dx, dy, dyaw, tilt_rad) where tilt_rad is the angle between the
        sensor's z axis and the parent's.
    """
    x, y, z, w = rotation_quat
    # yaw about the parent z, standard ZYX extraction
    dyaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    # The sensor's z axis expressed in the parent frame; its deviation from
    # (0, 0, 1) is the tilt, independent of yaw.
    zx = 2.0 * (x * z + w * y)
    zy = 2.0 * (y * z - w * x)
    zz = 1.0 - 2.0 * (x * x + y * y)
    tilt = math.atan2(math.hypot(zx, zy), abs(zz))
    return float(translation[0]), float(translation[1]), dyaw, tilt


def retarget_ranges(ranges: Sequence[float], angle_min: float, angle_increment: float,
                    offset: Tuple[float, float, float, float],
                    out: ScanSpec) -> List[float]:
    """Re-express ranges measured at a sensor about the parent frame's origin.

    Args:
        ranges: measured ranges, non-finite entries meaning "no return".
        angle_min, angle_increment: the input scan's angular frame.
        offset: (dx, dy, dyaw, tilt) from `planar_offset`; tilt is ignored here
            and is the caller's business to warn about.
        out: the output scan's extent.

    Returns:
        A list of `out.bin_count()` ranges, `inf` where no beam landed.
    """
    dx, dy, dyaw, _tilt = offset
    n_out = out.bin_count()
    result = [math.inf] * n_out
    cos_y, sin_y = math.cos(dyaw), math.sin(dyaw)
    identity = (abs(dx) < 1e-9 and abs(dy) < 1e-9 and abs(dyaw) < 1e-12)

    for i, r in enumerate(ranges):
        if r is None or not math.isfinite(r) or r <= 0.0:
            continue
        theta = angle_min + i * angle_increment
        if identity:
            r_b, bearing = r, theta
        else:
            px, py = r * math.cos(theta), r * math.sin(theta)
            bx = cos_y * px - sin_y * py + dx
            by = sin_y * px + cos_y * py + dy
            r_b = math.hypot(bx, by)
            bearing = math.atan2(by, bx)
        if r_b < out.range_min or r_b > out.range_max:
            continue
        # Wrap the bearing into the output's angular window before binning, so a
        # yawed sensor's beams do not fall off the ends.
        span = out.angle_max - out.angle_min
        b = bearing
        if span >= 2.0 * math.pi - 1e-9:
            while b < out.angle_min:
                b += 2.0 * math.pi
            while b > out.angle_max:
                b -= 2.0 * math.pi
        elif b < out.angle_min or b > out.angle_max:
            continue
        idx = int(round((b - out.angle_min) / out.angle_increment))
        if idx < 0 or idx >= n_out:
            continue
        if r_b < result[idx]:
            result[idx] = r_b
    return result


def is_identity(offset: Tuple[float, float, float, float],
                trans_eps: float = 1e-3, yaw_eps: float = 1.75e-3) -> bool:
    """True when the offset is negligible, so the scan can pass through.

    Defaults: 1 mm and 0.1 degrees. Below that, re-binning would only add
    quantisation noise for no benefit.
    """
    dx, dy, dyaw, _tilt = offset
    return abs(dx) <= trans_eps and abs(dy) <= trans_eps and abs(dyaw) <= yaw_eps


def main(args=None):
    """Node entry point; imports ROS lazily so the pure part stays importable."""
    from autosdv_mcl_launch.scan_normalizer_node import run
    run(args)


if __name__ == "__main__":
    main()
