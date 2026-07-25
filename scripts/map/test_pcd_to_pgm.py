import struct
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).parent))
from pcd_to_pgm import read_pcd, rasterize


def write_pcd(path: Path, pts: np.ndarray, binary: bool = True) -> None:
    """Minimal PCD v0.7 writer for fixtures (FIELDS x y z, FLOAT32)."""
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        f"WIDTH {len(pts)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(pts)}\nDATA {'binary' if binary else 'ascii'}\n"
    )
    with open(path, "wb") as f:
        f.write(header.encode())
        if binary:
            f.write(pts.astype(np.float32).tobytes())
        else:
            for p in pts:
                f.write(f"{p[0]} {p[1]} {p[2]}\n".encode())


@pytest.fixture
def wall_cloud() -> np.ndarray:
    """Ground plane 10x10 m at z=0 plus a wall at x=5, z 0..2."""
    xs, ys = np.meshgrid(np.arange(0, 10, 0.05), np.arange(0, 10, 0.05))
    ground = np.column_stack([xs.ravel(), ys.ravel(), np.zeros(xs.size)])
    wy, wz = np.meshgrid(np.arange(0, 10, 0.05), np.arange(0.0, 2.0, 0.05))
    wall = np.column_stack([np.full(wy.size, 5.0), wy.ravel(), wz.ravel()])
    return np.vstack([ground, wall])


def test_read_pcd_binary(tmp_path, wall_cloud):
    p = tmp_path / "cloud.pcd"
    write_pcd(p, wall_cloud, binary=True)
    pts = read_pcd(p)
    assert pts.shape == wall_cloud.shape
    np.testing.assert_allclose(pts, wall_cloud, atol=1e-6)


def test_read_pcd_ascii(tmp_path, wall_cloud):
    p = tmp_path / "cloud.pcd"
    write_pcd(p, wall_cloud, binary=False)
    pts = read_pcd(p)
    assert pts.shape == wall_cloud.shape


def test_read_pcd_xyzrgb(tmp_path):
    """COSS map layout: FIELDS x y z rgb, all FLOAT32 — rgb must be dropped."""
    pts = np.array([[1.0, 2.0, 3.0, 0.5], [4.0, 5.0, 6.0, 0.5]], np.float32)
    header = (
        "VERSION 0.7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\n"
        "COUNT 1 1 1 1\nWIDTH 2\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
        "POINTS 2\nDATA binary\n"
    )
    p = tmp_path / "rgb.pcd"
    with open(p, "wb") as f:
        f.write(header.encode())
        f.write(pts.tobytes())
    out = read_pcd(p)
    assert out.shape == (2, 3)
    np.testing.assert_allclose(out, pts[:, :3])


def test_read_pcd_with_padding_field(tmp_path):
    """Autoware sample maps have a uint padding field `_` — must be skipped."""
    pts = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], np.float32)
    pad = np.zeros((2, 2), np.uint8)  # 2-byte padding per point
    header = (
        "VERSION 0.7\nFIELDS x y z _\nSIZE 4 4 4 2\nTYPE F F F U\n"
        "COUNT 1 1 1 1\nWIDTH 2\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
        "POINTS 2\nDATA binary\n"
    )
    p = tmp_path / "pad.pcd"
    with open(p, "wb") as f:
        f.write(header.encode())
        for row, prow in zip(pts, pad):
            f.write(row.tobytes()); f.write(prow.tobytes())
    out = read_pcd(p)
    assert out.shape == (2, 3)
    np.testing.assert_allclose(out, pts)


def test_read_pcd_rejects_non_float_xyz(tmp_path):
    """x/y/z typed anything but FLOAT32 must raise, even when other fields parse."""
    header = (
        "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 8\nTYPE F F F\n"
        "COUNT 1 1 1\nWIDTH 1\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
        "POINTS 1\nDATA binary\n"
    )
    p = tmp_path / "bad.pcd"
    with open(p, "wb") as f:
        f.write(header.encode())
        f.write(b"\x00" * 16)
    with pytest.raises(ValueError):
        read_pcd(p)


def test_read_pcd_count_subarray(tmp_path):
    """A non-xyz field with COUNT>1 must parse as a subarray and be skipped."""
    header = (
        "VERSION 0.7\nFIELDS x y z extra\nSIZE 4 4 4 4\nTYPE F F F F\n"
        "COUNT 1 1 1 3\nWIDTH 2\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
        "POINTS 2\nDATA binary\n"
    )
    pts = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], np.float32)
    extra = np.zeros((2, 3), np.float32)
    p = tmp_path / "count.pcd"
    with open(p, "wb") as f:
        f.write(header.encode())
        for row, erow in zip(pts, extra):
            f.write(row.tobytes()); f.write(erow.tobytes())
    out = read_pcd(p)
    assert out.shape == (2, 3)
    np.testing.assert_allclose(out, pts)


def test_rasterize_wall_occupied_ground_free(wall_cloud):
    grid, origin = rasterize(wall_cloud, z_min=0.2, z_max=0.5,
                             resolution=0.1, min_points=1)
    assert origin == pytest.approx((0.0, 0.0), abs=0.1)
    # wall column x=5 → occupied (0); open ground x=2 → free (254):
    # row index counts from map bottom (row 0 = y_min) before PGM flip.
    col_wall = int(round((5.0 - origin[0]) / 0.1))
    col_free = int(round((2.0 - origin[0]) / 0.1))
    row = int(round((5.0 - origin[1]) / 0.1))
    assert grid[row, col_wall] == 0
    assert grid[row, col_free] == 254


def test_rasterize_unknown_outside_data(wall_cloud):
    grid, origin = rasterize(wall_cloud, z_min=0.2, z_max=0.5,
                             resolution=0.1, min_points=1)
    # cells with zero points anywhere in the column are unknown — probe by
    # padding: rasterize a cloud with a hole
    hole = wall_cloud[~((wall_cloud[:, 0] > 7) & (wall_cloud[:, 1] > 7))]
    g2, o2 = rasterize(hole, 0.2, 0.5, 0.1, 1)
    r = int(round((9.0 - o2[1]) / 0.1)); c = int(round((9.0 - o2[0]) / 0.1))
    assert g2[r, c] == 205


def test_cli_writes_pgm_and_yaml(tmp_path, wall_cloud):
    pcd = tmp_path / "in.pcd"
    write_pcd(pcd, wall_cloud)
    prefix = tmp_path / "map"
    subprocess.run(
        [sys.executable, str(Path(__file__).parent / "pcd_to_pgm.py"),
         str(pcd), str(prefix), "--z-min", "0.2", "--z-max", "0.5",
         "--resolution", "0.1"],
        check=True,
    )
    assert (tmp_path / "map.pgm").exists()
    meta = yaml.safe_load((tmp_path / "map.yaml").read_text())
    assert meta["resolution"] == 0.1
    assert meta["image"] == "map.pgm"
    assert meta["negate"] == 0
    assert len(meta["origin"]) == 3
