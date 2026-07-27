#!/usr/bin/env python3
"""Read and write `autosdv_map.yaml`, the optional per-map-directory sidecar.

Autoware's map directory records the projection but not "which geometry maps
does this directory hold, and how was the occupancy grid produced". A user
looking at `pointcloud_map.pcd` and `occupancy_grid.pgm` cannot answer the
second question at all, and getting it wrong is not a quiet failure: Phase 1
of the 2-D MCL work sliced a PCD at the wrong height, produced a 230-cell
grid, and lost days to localization that "just didn't work".

The sidecar exists so a tool can answer it. See
docs/design/map-handling-per-localization-method.md Sec. 4 for the schema.
It is deliberately optional -- absent means "infer from what is on disk", which
is what every existing map directory relies on.

The functions here are pure apart from `load`/`save`, so they can be tested
without a map directory or a ROS environment.
"""
from __future__ import annotations

from pathlib import Path
from typing import Optional

import yaml

SIDECAR_NAME = "autosdv_map.yaml"

# Provenance methods, matching the design doc.
METHOD_PCD_SLICE = "pcd_slice"
METHOD_SCAN_ACCUMULATION = "scan_accumulation"
METHOD_SLAM = "slam"
METHODS = (METHOD_PCD_SLICE, METHOD_SCAN_ACCUMULATION, METHOD_SLAM)


def sidecar_path(map_dir: Path) -> Path:
    return Path(map_dir) / SIDECAR_NAME


def load(map_dir: Path) -> Optional[dict]:
    """Parse the sidecar, or None when absent.

    A malformed sidecar raises: a file that exists but cannot be parsed is a
    problem to report, not one to silently treat as missing.
    """
    path = sidecar_path(map_dir)
    if not path.is_file():
        return None
    data = yaml.safe_load(path.read_text())
    if data is None:
        return {}
    if not isinstance(data, dict):
        raise ValueError(f"{path}: expected a mapping at the top level, "
                         f"got {type(data).__name__}")
    return data


def build_grid_provenance(method: str, source: str, resolution: float,
                          z_band: Optional[tuple] = None,
                          frame_consistent_with_lanelet2: bool = True,
                          extra: Optional[dict] = None) -> dict:
    """The `occupancy_grid_provenance` block for a grid just written.

    `z_band` is None for methods that do not slice a height band. Values are
    coerced to plain floats so PyYAML emits numbers rather than numpy scalar
    tags, which would make the file unreadable to anything but Python.
    """
    if method not in METHODS:
        raise ValueError(f"unknown provenance method {method!r}; "
                         f"expected one of {', '.join(METHODS)}")
    block = {
        "method": method,
        "source": str(source),
        "resolution": float(resolution),
        "frame_consistent_with_lanelet2": bool(frame_consistent_with_lanelet2),
    }
    if z_band is not None:
        lo, hi = z_band
        if float(hi) <= float(lo):
            raise ValueError(f"z_band must be increasing, got [{lo}, {hi}]")
        block["z_band"] = [float(lo), float(hi)]
    if extra:
        block.update(extra)
    return block


def merge(existing: Optional[dict], *, geometry: dict,
          grid_provenance: Optional[dict] = None) -> dict:
    """Fold a newly built grid into whatever the sidecar already said.

    Rebuilding the grid must not drop unrelated facts a previous tool
    recorded, so `geometry` entries are merged rather than replacing the
    block, and any key outside the two blocks this module owns is preserved
    untouched.
    """
    out = dict(existing or {})
    merged_geometry = dict(out.get("geometry") or {})
    merged_geometry.update({k: v for k, v in geometry.items() if v is not None})
    out["geometry"] = merged_geometry
    if grid_provenance is not None:
        out["occupancy_grid_provenance"] = grid_provenance
    return out


def dumps(data: dict) -> str:
    """Serialize with a header explaining what the file is."""
    body = yaml.safe_dump(data, sort_keys=False, default_flow_style=None)
    return (
        "# autosdv_map.yaml -- AutoSDV-only map directory sidecar (optional).\n"
        "# Records which geometry maps this directory holds and how the\n"
        "# occupancy grid was produced. Written by the map-building tools;\n"
        "# read by `just map-check`. Safe to delete: absent means \"infer\n"
        "# from what is on disk\".\n"
        "# Schema: docs/design/map-handling-per-localization-method.md Sec. 4\n"
        + body
    )


def save(map_dir: Path, *, geometry: dict,
         grid_provenance: Optional[dict] = None) -> Path:
    """Merge into any existing sidecar and write it. Returns the path."""
    map_dir = Path(map_dir)
    path = sidecar_path(map_dir)
    data = merge(load(map_dir), geometry=geometry,
                 grid_provenance=grid_provenance)
    path.write_text(dumps(data))
    return path


def describe(data: Optional[dict]) -> list:
    """Human-readable lines for `map-check` to print. Empty when absent.

    Tolerates a partial sidecar: a hand-written file missing keys should
    produce less output, not a traceback.
    """
    if not data:
        return []
    lines = []
    prov = data.get("occupancy_grid_provenance") or {}
    if prov:
        method = prov.get("method", "?")
        source = prov.get("source", "?")
        detail = f"built by {method} from {source}"
        if prov.get("z_band"):
            band = prov["z_band"]
            try:
                detail += f", z band [{float(band[0]):.2f}, {float(band[1]):.2f}] m"
            except (TypeError, ValueError, IndexError):
                detail += f", z band {band}"
        if prov.get("resolution") is not None:
            try:
                detail += f", {float(prov['resolution']):.3f} m/px"
            except (TypeError, ValueError):
                pass
        lines.append(detail)
        if prov.get("frame_consistent_with_lanelet2") is False:
            lines.append("the building tool did NOT assert frame consistency "
                         "with lanelet2")
    geometry = data.get("geometry") or {}
    if geometry:
        lines.append("declares: " + ", ".join(
            f"{k}={v}" for k, v in sorted(geometry.items())))
    return lines
