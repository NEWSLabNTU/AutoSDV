"""Minimal PLY and PCD reading and writing, with intensity preserved.

GLIM's dump and its viewer's export carry intensity; Autoware's map loader reads PCD.
Only the subset those tools emit is supported: PLY ``binary_little_endian`` and
``ascii``, PCD ``ascii`` and ``binary``. Compressed PCD is rejected rather than
half-parsed. (Copied from reflective_pose_core.pointcloud_io, so Lab 3 does not
depend on the reflective-board packages.)
"""

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

_PLY_TYPES = {
    "char": np.int8, "int8": np.int8,
    "uchar": np.uint8, "uint8": np.uint8,
    "short": np.int16, "int16": np.int16,
    "ushort": np.uint16, "uint16": np.uint16,
    "int": np.int32, "int32": np.int32,
    "uint": np.uint32, "uint32": np.uint32,
    "float": np.float32, "float32": np.float32,
    "double": np.float64, "float64": np.float64,
}

_PCD_TYPES = {
    ("F", 4): np.float32, ("F", 8): np.float64,
    ("U", 1): np.uint8, ("U", 2): np.uint16, ("U", 4): np.uint32,
    ("I", 1): np.int8, ("I", 2): np.int16, ("I", 4): np.int32,
}

# GLIM writes "scalar_intensity"; PCL and Autoware use "intensity".
_INTENSITY_ALIASES = ("intensity", "scalar_intensity", "i", "reflectivity")


@dataclass
class PointCloud:
    """Points with an optional intensity channel."""

    points: np.ndarray  # (N, 3) float64
    intensity: Optional[np.ndarray] = None  # (N,) float64

    def __len__(self) -> int:
        return len(self.points)

    @property
    def has_intensity(self) -> bool:
        return self.intensity is not None


def _pick_intensity(fields: Dict[str, np.ndarray]) -> Optional[np.ndarray]:
    for name in _INTENSITY_ALIASES:
        if name in fields:
            return np.asarray(fields[name], dtype=np.float64)
    return None


def read_ply(path: str) -> PointCloud:
    """Read a binary-little-endian or ascii PLY vertex element."""
    with open(path, "rb") as handle:
        if handle.readline().strip() != b"ply":
            raise ValueError(f"{path}: not a PLY file")

        fmt = None
        count = None
        properties: List[Tuple[str, type]] = []
        in_vertex = False

        while True:
            line = handle.readline()
            if not line:
                raise ValueError(f"{path}: PLY header never ended")
            tokens = line.decode("ascii", "replace").split()
            if not tokens:
                continue
            if tokens[0] == "format":
                fmt = tokens[1]
            elif tokens[0] == "element":
                in_vertex = tokens[1] == "vertex"
                if in_vertex:
                    count = int(tokens[2])
            elif tokens[0] == "property" and in_vertex:
                if tokens[1] == "list":
                    raise ValueError(f"{path}: list properties are not supported")
                properties.append((tokens[2], _PLY_TYPES[tokens[1]]))
            elif tokens[0] == "end_header":
                break

        if fmt == "binary_big_endian":
            raise ValueError(f"{path}: big-endian PLY is not supported")
        if count is None:
            raise ValueError(f"{path}: no vertex element")

        names = [name for name, _ in properties]
        if fmt == "ascii":
            values = np.loadtxt(handle, max_rows=count, ndmin=2)
            fields = {name: values[:, i] for i, name in enumerate(names)}
        else:
            dtype = np.dtype([(name, kind) for name, kind in properties])
            # One read for the whole block: reading per field would consume the
            # buffer on the first field and hand back nothing for the rest.
            raw = np.frombuffer(handle.read(dtype.itemsize * count), dtype=dtype)
            fields = {name: raw[name] for name in names}

    missing = {"x", "y", "z"} - set(names)
    if missing:
        raise ValueError(f"{path}: missing coordinate fields {sorted(missing)}")

    points = np.column_stack(
        [np.asarray(fields[axis], dtype=np.float64) for axis in ("x", "y", "z")]
    )
    return PointCloud(points=points, intensity=_pick_intensity(fields))


def read_pcd(path: str) -> PointCloud:
    """Read an ascii or uncompressed-binary PCD."""
    header: Dict[str, List[str]] = {}
    with open(path, "rb") as handle:
        while True:
            line = handle.readline()
            if not line:
                raise ValueError(f"{path}: PCD header never ended")
            text = line.decode("ascii", "replace").strip()
            if not text or text.startswith("#"):
                continue
            key, _, rest = text.partition(" ")
            header[key.upper()] = rest.split()
            if key.upper() == "DATA":
                break

        data_kind = header["DATA"][0]
        if data_kind == "binary_compressed":
            raise ValueError(f"{path}: binary_compressed PCD is not supported")

        names = header["FIELDS"]
        sizes = [int(v) for v in header["SIZE"]]
        kinds = header["TYPE"]
        counts = [int(v) for v in header.get("COUNT", ["1"] * len(names))]
        if any(c != 1 for c in counts):
            raise ValueError(f"{path}: multi-count fields are not supported")
        total = int(header["POINTS"][0]) if "POINTS" in header else (
            int(header["WIDTH"][0]) * int(header["HEIGHT"][0])
        )

        if data_kind == "ascii":
            values = np.loadtxt(handle, max_rows=total, ndmin=2)
            fields = {name: values[:, i] for i, name in enumerate(names)}
        else:
            dtype = np.dtype(
                [(name, _PCD_TYPES[(kind, size)])
                 for name, kind, size in zip(names, kinds, sizes)]
            )
            raw = np.frombuffer(handle.read(dtype.itemsize * total), dtype=dtype)
            fields = {name: raw[name] for name in names}

    missing = {"x", "y", "z"} - set(names)
    if missing:
        raise ValueError(f"{path}: missing coordinate fields {sorted(missing)}")

    points = np.column_stack(
        [np.asarray(fields[axis], dtype=np.float64) for axis in ("x", "y", "z")]
    )
    return PointCloud(points=points, intensity=_pick_intensity(fields))


def read_cloud(path: str) -> PointCloud:
    """Read by extension."""
    lowered = path.lower()
    if lowered.endswith(".ply"):
        return read_ply(path)
    if lowered.endswith(".pcd"):
        return read_pcd(path)
    raise ValueError(f"{path}: expected a .ply or .pcd file")


def write_pcd(path: str, cloud: PointCloud, binary: bool = True) -> None:
    """Write a PCD with ``x y z`` and, when present, ``intensity``.

    Field order and naming follow what PCL and the Autoware map loader expect,
    so the output drops straight into ``autoware_pointcloud_divider``.
    """
    points = np.asarray(cloud.points, dtype=np.float32)
    if cloud.has_intensity:
        names = ["x", "y", "z", "intensity"]
        columns = [points[:, 0], points[:, 1], points[:, 2],
                   np.asarray(cloud.intensity, dtype=np.float32)]
    else:
        names = ["x", "y", "z"]
        columns = [points[:, 0], points[:, 1], points[:, 2]]

    count = len(points)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        f"FIELDS {' '.join(names)}\n"
        f"SIZE {' '.join(['4'] * len(names))}\n"
        f"TYPE {' '.join(['F'] * len(names))}\n"
        f"COUNT {' '.join(['1'] * len(names))}\n"
        f"WIDTH {count}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {count}\n"
        f"DATA {'binary' if binary else 'ascii'}\n"
    )

    with open(path, "wb") as handle:
        handle.write(header.encode("ascii"))
        stacked = np.column_stack(columns).astype(np.float32)
        if binary:
            handle.write(stacked.tobytes())
        else:
            np.savetxt(handle, stacked, fmt="%.6f")


def convert(source: str, destination: str) -> PointCloud:
    """Read any supported cloud and write it as PCD. Intensity survives."""
    cloud = read_cloud(source)
    write_pcd(destination, cloud)
    return cloud
