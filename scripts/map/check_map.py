#!/usr/bin/env python3
"""Validate a map directory for a given pose_source before you launch it.

Usage:
    check_map.py <map_dir> [--pose-source mcl|cuda_ndt|ndt]
                            [--grid-yaml NAME] [--autoware-setup PATH]

Exits 0 iff the directory is usable for the given pose_source. Prints one
line per artefact (ok / FAIL / - for "not required"), and — this is the
point of the whole check — compares the lanelet2 map's bounding box in
*map-frame metres* against the occupancy grid's coverage. A grid built in
the wrong frame is a silent, catastrophic localization failure (30 m+
errors, no error message); this is what catches it before launch.

Honesty requirement: for a georeferenced map (anything but
projector_type: Local), the lanelet2 bounding box in map coordinates can
only be computed by projecting its lat/lon the same way
`lanelet2_map_loader` does. This script does that by compiling and calling
Autoware's own projection code (`autoware_geography_utils` +
`autoware_map_projection_loader`) via scripts/map/_projector_cpp — it does
NOT reimplement any projection math. If that tool cannot be built or run
(no Autoware environment available), the frame check reports
"cannot verify" and FAILS rather than silently passing or guessing from
unrelated tags. `local_x`/`local_y` tags are only ever used directly when
projector_type is Local, which is the one case where Autoware itself
defines those tags to already be in the map frame.
"""
from __future__ import annotations

import argparse
import glob
import os
import subprocess
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import yaml

sys.path.insert(0, str(Path(__file__).parent))
import map_sidecar  # noqa: E402
from PIL import Image

SCRIPT_DIR = Path(__file__).resolve().parent
PROJECTOR_DIR = SCRIPT_DIR / "_projector_cpp"
PROJECTOR_BIN = PROJECTOR_DIR / "install" / "lib" / "map_check_projector" / "project_points"

VALID_PROJECTOR_TYPES = {
    "MGRS", "LocalCartesian", "LocalCartesianUTM", "TransverseMercator",
    "Local", "local",
}

OCCUPIED, FREE, UNKNOWN = 0, 254, 205


# ---------------------------------------------------------------------------
# Pure data model / maths (unit-testable without any ROS environment)
# ---------------------------------------------------------------------------

@dataclass
class BBox:
    xmin: float
    ymin: float
    xmax: float
    ymax: float

    @property
    def width(self) -> float:
        return max(0.0, self.xmax - self.xmin)

    @property
    def height(self) -> float:
        return max(0.0, self.ymax - self.ymin)

    @property
    def area(self) -> float:
        return self.width * self.height

    def contains_point(self, x: float, y: float) -> bool:
        return self.xmin <= x <= self.xmax and self.ymin <= y <= self.ymax


def bbox_from_points(points):
    """points: iterable of (x, y). Raises ValueError if empty."""
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    if not xs:
        raise ValueError("no points to build a bounding box from")
    return BBox(min(xs), min(ys), max(xs), max(ys))


def overlap_pct(inner: BBox, outer: BBox) -> float:
    """Percent of `inner`'s area covered by its intersection with `outer`.

    `inner` is the reference (the lanelet2 bounding box) — the design-doc
    wording is "grid covers X% of the lanelet2 bounding box". Degenerate
    (zero-area) inner boxes fall back to a point-in-box test.
    """
    ix0 = max(inner.xmin, outer.xmin)
    iy0 = max(inner.ymin, outer.ymin)
    ix1 = min(inner.xmax, outer.xmax)
    iy1 = min(inner.ymax, outer.ymax)
    inter_w = max(0.0, ix1 - ix0)
    inter_h = max(0.0, iy1 - iy0)
    inter_area = inter_w * inter_h

    if inner.area > 0:
        return 100.0 * inter_area / inner.area
    # degenerate inner bbox (e.g. a single point): binary in/out
    return 100.0 if outer.contains_point(inner.xmin, inner.ymin) else 0.0


# ---------------------------------------------------------------------------
# File parsers
# ---------------------------------------------------------------------------

class CheckError(Exception):
    """Raised by parsers; message becomes the FAIL line's remedy text."""


def parse_lanelet2_osm(path: Path):
    """Return (list of (lat, lon), dict node_id -> (local_x, local_y))."""
    try:
        tree = ET.parse(path)
    except ET.ParseError as e:
        raise CheckError(f"not well-formed XML ({e}) -> re-export the lanelet2 map") from e
    root = tree.getroot()
    latlon = []
    local_tags = {}
    for node in root.findall("node"):
        lat = node.get("lat")
        lon = node.get("lon")
        if lat is None or lon is None:
            continue
        latlon.append((float(lat), float(lon)))
        tags = {t.get("k"): t.get("v") for t in node.findall("tag")}
        if "local_x" in tags and "local_y" in tags:
            try:
                local_tags[node.get("id")] = (float(tags["local_x"]), float(tags["local_y"]))
            except (TypeError, ValueError):
                pass
    if not latlon:
        raise CheckError("no <node lat=.. lon=..> elements found -> is this a valid lanelet2_map.osm?")
    return latlon, local_tags


def parse_projector_info(path: Path) -> dict:
    try:
        data = yaml.safe_load(path.read_text())
    except yaml.YAMLError as e:
        raise CheckError(f"not valid YAML ({e})") from e
    if not isinstance(data, dict) or "projector_type" not in data:
        raise CheckError("missing 'projector_type' key -> regenerate with map_projector_info format")
    ptype = data["projector_type"]
    if ptype not in VALID_PROJECTOR_TYPES:
        raise CheckError(
            f"unknown projector_type '{ptype}' -> must be one of "
            f"{sorted(VALID_PROJECTOR_TYPES)}"
        )
    return data


def parse_grid_yaml(path: Path) -> dict:
    try:
        data = yaml.safe_load(path.read_text())
    except yaml.YAMLError as e:
        raise CheckError(f"not valid YAML ({e})") from e
    if not isinstance(data, dict):
        raise CheckError("grid yaml did not parse to a mapping")
    for key in ("image", "resolution", "origin"):
        if key not in data:
            raise CheckError(f"grid yaml missing required key '{key}'")
    if not isinstance(data["origin"], (list, tuple)) or len(data["origin"]) < 2:
        raise CheckError("grid yaml 'origin' must be a [x, y, theta] list")
    return data


def read_pgm_counts(path: Path):
    """Return (width, height, occupied, free, unknown, other)."""
    try:
        img = Image.open(path)
    except Exception as e:  # noqa: BLE001 - surfaced as a remedy message
        raise CheckError(f"could not open PGM ({e})") from e
    img = img.convert("L")
    w, h = img.size
    import numpy as np
    arr = np.asarray(img)
    occ = int((arr == OCCUPIED).sum())
    free = int((arr == FREE).sum())
    unk = int((arr == UNKNOWN).sum())
    other = int(arr.size - occ - free - unk)
    return w, h, occ, free, unk, other


def grid_bbox(grid_meta: dict, width: int, height: int) -> BBox:
    ox, oy = float(grid_meta["origin"][0]), float(grid_meta["origin"][1])
    res = float(grid_meta["resolution"])
    return BBox(ox, oy, ox + width * res, oy + height * res)


# ---------------------------------------------------------------------------
# Autoware projection reuse (the only honest way to get the lanelet2 bbox
# in map-frame metres for a georeferenced map)
# ---------------------------------------------------------------------------

class ProjectionUnavailable(Exception):
    pass


def find_autoware_setup(explicit: Optional[str]) -> Optional[str]:
    if explicit:
        return explicit if Path(explicit).exists() else None
    env = os.environ.get("AUTOWARE_SETUP_BASH")
    if env and Path(env).exists():
        return env
    candidates = sorted(glob.glob("/opt/autoware/*/setup.bash"), reverse=True)
    return candidates[0] if candidates else None


def ensure_projector_binary(autoware_setup: Optional[str]) -> Path:
    """Build scripts/map/_projector_cpp on demand if needed. Raises
    ProjectionUnavailable with a human remedy if it cannot be built/run."""
    if PROJECTOR_BIN.exists():
        return PROJECTOR_BIN
    if not autoware_setup:
        raise ProjectionUnavailable(
            "no Autoware installation found (looked for /opt/autoware/*/setup.bash and "
            "$AUTOWARE_SETUP_BASH) -> source an Autoware setup.bash and pass "
            "--autoware-setup, or install Autoware per docs/guides"
        )
    cmd = (
        f"source {autoware_setup} >/dev/null 2>&1 && "
        f"colcon build --base-paths . --build-base build --install-base install "
        f"--merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release"
    )
    result = subprocess.run(
        ["bash", "-c", cmd], cwd=str(PROJECTOR_DIR),
        capture_output=True, text=True, timeout=300,
    )
    if result.returncode != 0 or not PROJECTOR_BIN.exists():
        raise ProjectionUnavailable(
            "failed to build scripts/map/_projector_cpp (Autoware's own projection code) "
            f"-> {result.stderr.strip()[-500:]}"
        )
    return PROJECTOR_BIN


def project_latlon(yaml_path: Path, points, autoware_setup: Optional[str]):
    """Project (lat, lon) points into map-frame (x, y) using Autoware's own
    projection code. Raises ProjectionUnavailable if the tool cannot be
    built or run."""
    binpath = ensure_projector_binary(autoware_setup)
    setup = autoware_setup or find_autoware_setup(None)
    if not setup:
        raise ProjectionUnavailable(
            "projector binary exists but no Autoware setup.bash to source at run time "
            "-> pass --autoware-setup /path/to/setup.bash"
        )
    stdin = "\n".join(f"{lat} {lon}" for lat, lon in points) + "\n"
    cmd = f"source {setup} >/dev/null 2>&1 && exec {binpath} {yaml_path}"
    try:
        result = subprocess.run(
            ["bash", "-c", cmd], input=stdin,
            capture_output=True, text=True, timeout=120,
        )
    except subprocess.TimeoutExpired as e:
        raise ProjectionUnavailable(f"projection tool timed out -> {e}") from e
    if result.returncode != 0:
        raise ProjectionUnavailable(
            f"Autoware projection tool failed -> {result.stderr.strip()[-500:]}"
        )
    lines = result.stdout.strip().splitlines()
    if not lines or not lines[0].startswith("PROJECTOR_TYPE"):
        raise ProjectionUnavailable(f"unexpected projector tool output: {result.stdout[:200]!r}")
    projector_type = lines[0].split(maxsplit=1)[1] if " " in lines[0] else ""
    projected = []
    for line in lines[1:]:
        x_str, y_str = line.split()
        projected.append((float(x_str), float(y_str)))
    return projector_type, projected


# ---------------------------------------------------------------------------
# Report line
# ---------------------------------------------------------------------------

@dataclass
class Line:
    name: str
    status: str  # "ok" | "FAIL" | "-"
    message: str = ""

    def render(self) -> str:
        pad_name = self.name.ljust(28)
        pad_status = self.status.ljust(5)
        if self.message:
            return f"  {pad_name} {pad_status} {self.message}"
        return f"  {pad_name} {pad_status}"


# ---------------------------------------------------------------------------
# Main check
# ---------------------------------------------------------------------------

def human_size(num_bytes: int) -> str:
    for unit in ("B", "KB", "MB", "GB"):
        if num_bytes < 1024:
            return f"{num_bytes:.0f} {unit}" if unit == "B" else f"{num_bytes:.1f} {unit}"
        num_bytes /= 1024
    return f"{num_bytes:.1f} TB"


def check_sidecar(map_dir: Path) -> list:
    """Report autosdv_map.yaml's provenance, or note that it is absent.

    Never fails a check: the sidecar is optional by design, and a map
    directory without one is the normal legacy case. Its value is answering
    "how was this grid built, and at what height band" -- a question the two
    binary files cannot answer, and getting it wrong once cost this project a
    whole phase.
    """
    try:
        data = map_sidecar.load(map_dir)
    except (ValueError, yaml.YAMLError) as e:
        return [Line(map_sidecar.SIDECAR_NAME, "FAIL", f"unreadable: {e}")]
    if data is None:
        return [Line(map_sidecar.SIDECAR_NAME, "-",
                      "absent (optional; grid provenance unrecorded)")]
    described = map_sidecar.describe(data)
    if not described:
        return [Line(map_sidecar.SIDECAR_NAME, "-", "present but records nothing")]
    lines = [Line(map_sidecar.SIDECAR_NAME, "ok", described[0])]
    lines += [Line("", "", extra) for extra in described[1:]]
    return lines


def run_check(map_dir: Path, pose_source: str, grid_yaml_name: str,
              autoware_setup: Optional[str]):
    """Returns (list[Line], ready: bool)."""
    lines = []
    require_pcd = pose_source in ("ndt", "cuda_ndt")
    require_grid = pose_source == "mcl"
    ready = True

    # -- lanelet2_map.osm --------------------------------------------------
    lanelet2_path = map_dir / "lanelet2_map.osm"
    latlon_points = None
    local_tags = None
    if not lanelet2_path.exists():
        lines.append(Line("lanelet2_map.osm", "FAIL",
                           "missing -> every pose_source needs this file"))
        ready = False
    else:
        try:
            latlon_points, local_tags = parse_lanelet2_osm(lanelet2_path)
            size = human_size(lanelet2_path.stat().st_size)
            lines.append(Line("lanelet2_map.osm", "ok",
                               f"({size}, {len(latlon_points)} nodes)"))
        except CheckError as e:
            lines.append(Line("lanelet2_map.osm", "FAIL", str(e)))
            ready = False

    # -- map_projector_info.yaml --------------------------------------------
    proj_path = map_dir / "map_projector_info.yaml"
    proj_info = None
    if not proj_path.exists():
        lines.append(Line("map_projector_info.yaml", "FAIL",
                           "missing -> every pose_source needs this file"))
        ready = False
    else:
        try:
            proj_info = parse_projector_info(proj_path)
            ptype = proj_info["projector_type"]
            is_local = ptype.lower() == "local"
            if is_local:
                msg = "Local -- GNSS init unavailable (lanelet2 must carry local_x/local_y tags)"
            else:
                origin = proj_info.get("map_origin", {}) or {}
                lat = origin.get("latitude")
                lon = origin.get("longitude")
                if lat is not None and lon is not None:
                    where = f"@ {lat}, {lon}"
                elif proj_info.get("mgrs_grid"):
                    where = f"grid {proj_info['mgrs_grid']}"
                else:
                    where = ""
                msg = f"{ptype} {where} -- georeferenced, GNSS init available".replace("  ", " ")
            lines.append(Line("map_projector_info.yaml", "ok", msg))
        except CheckError as e:
            lines.append(Line("map_projector_info.yaml", "FAIL", str(e)))
            ready = False

    # -- pointcloud_map.pcd / pointcloud_map_metadata.yaml ------------------
    pcd_path = map_dir / "pointcloud_map.pcd"
    if not require_pcd:
        lines.append(Line("pointcloud_map.pcd", "-",
                           f"not required for pose_source={pose_source}"))
        lines.append(Line("pointcloud_map_metadata.yaml", "-",
                           f"not required for pose_source={pose_source}"))
    else:
        if not pcd_path.exists():
            lines.append(Line("pointcloud_map.pcd", "FAIL",
                               f"missing -> required for pose_source={pose_source}, "
                               "run just map grid-from-pcd or copy the PCD map here"))
            ready = False
        else:
            lines.append(Line("pointcloud_map.pcd", "ok",
                               f"({human_size(pcd_path.stat().st_size)})"))
        meta_path = map_dir / "pointcloud_map_metadata.yaml"
        if not meta_path.exists():
            lines.append(Line("pointcloud_map_metadata.yaml", "FAIL",
                               f"missing -> required for pose_source={pose_source}"))
            ready = False
        else:
            try:
                yaml.safe_load(meta_path.read_text())
                lines.append(Line("pointcloud_map_metadata.yaml", "ok"))
            except yaml.YAMLError as e:
                lines.append(Line("pointcloud_map_metadata.yaml", "FAIL", f"not valid YAML ({e})"))
                ready = False

    # -- occupancy_grid.yaml / .pgm -----------------------------------------
    grid_yaml_path = map_dir / grid_yaml_name
    grid_meta = None
    grid_counts = None
    if not require_grid and not grid_yaml_path.exists():
        lines.append(Line("occupancy_grid.yaml", "-",
                           f"not required for pose_source={pose_source}"))
        lines.append(Line("occupancy_grid.pgm", "-",
                           f"not required for pose_source={pose_source}"))
    else:
        if not grid_yaml_path.exists():
            lines.append(Line(grid_yaml_name, "FAIL",
                               f"missing -> required for pose_source={pose_source}, "
                               "run just map grid-from-pcd"))
            if require_grid:
                ready = False
        else:
            try:
                grid_meta = parse_grid_yaml(grid_yaml_path)
                lines.append(Line(grid_yaml_name, "ok",
                                   f"resolution {grid_meta['resolution']} m, "
                                   f"origin {list(grid_meta['origin'])}"))
                pgm_path = grid_yaml_path.parent / grid_meta["image"]
                if not pgm_path.exists():
                    lines.append(Line(grid_meta["image"], "FAIL",
                                       f"referenced by {grid_yaml_name} but missing"))
                    if require_grid:
                        ready = False
                else:
                    w, h, occ, free, unk, other = read_pgm_counts(pgm_path)
                    extra = f", {other} other" if other else ""
                    lines.append(Line(grid_meta["image"], "ok",
                                       f"{w}x{h} @ {grid_meta['resolution']} m -- "
                                       f"{occ} occupied / {free} free / {unk} unknown{extra}"))
                    grid_counts = (w, h)
            except CheckError as e:
                lines.append(Line(grid_yaml_name, "FAIL", str(e)))
                if require_grid:
                    ready = False

    # -- grid vs lanelet2 extent ---------------------------------------------
    have_grid = grid_meta is not None and grid_counts is not None
    if not have_grid:
        lines.append(Line("grid vs lanelet2 extent", "-",
                           "no occupancy grid to compare" if not require_grid
                           else "skipped -- grid missing/invalid above"))
    elif latlon_points is None or proj_info is None:
        lines.append(Line("grid vs lanelet2 extent", "-",
                           "skipped -- lanelet2 map or projector info missing/invalid above"))
    else:
        w, h = grid_counts
        gbbox = grid_bbox(grid_meta, w, h)
        ptype = proj_info["projector_type"]
        is_local = ptype.lower() == "local"
        try:
            if is_local:
                if not local_tags:
                    raise CheckError(
                        "projector_type is Local but no lanelet2 node carries "
                        "local_x/local_y tags -> author the map in local coordinates "
                        "or switch to a georeferenced projector_type"
                    )
                n_missing = len(latlon_points) - len(local_tags)
                if n_missing:
                    raise CheckError(
                        f"projector_type is Local but {n_missing}/{len(latlon_points)} "
                        "lanelet2 nodes are missing local_x/local_y tags"
                    )
                lbbox = bbox_from_points(local_tags.values())
                path_desc = "local_x/local_y tags (projector_type: Local)"
            else:
                try:
                    resolved_type, projected = project_latlon(
                        proj_path, latlon_points, autoware_setup)
                except ProjectionUnavailable as e:
                    lines.append(Line("grid vs lanelet2 extent", "FAIL",
                                       f"cannot verify: {e}"))
                    if require_grid:
                        ready = False
                    projected = None
                if projected is not None:
                    lbbox = bbox_from_points(projected)
                    path_desc = f"Autoware {resolved_type} projection"
                else:
                    lbbox = None
            if lbbox is not None:
                pct = overlap_pct(lbbox, gbbox)
                status = "ok" if pct > 0.0 else "FAIL"
                if status == "ok":
                    msg = (f"grid covers {pct:.0f}% of the lanelet2 bounding box "
                           f"(via {path_desc})")
                    lines.append(Line("grid vs lanelet2 extent", "ok", msg))
                else:
                    msg = (
                        f"lanelet2 spans x[{lbbox.xmin:.1f},{lbbox.xmax:.1f}] "
                        f"y[{lbbox.ymin:.1f},{lbbox.ymax:.1f}] but the grid covers "
                        f"x[{gbbox.xmin:.1f},{gbbox.xmax:.1f}] y[{gbbox.ymin:.1f},{gbbox.ymax:.1f}] "
                        f"(via {path_desc}) -> the grid is not in the map frame. Either "
                        "rebuild it from the PCD (just map grid-from-pcd), or set "
                        "projector_type: Local and author lanelet2 in local coords."
                    )
                    lines.append(Line("grid vs lanelet2 extent", "FAIL", msg))
                    if require_grid:
                        ready = False
        except CheckError as e:
            lines.append(Line("grid vs lanelet2 extent", "FAIL", f"cannot verify: {e}"))
            if require_grid:
                ready = False

    lines.extend(check_sidecar(map_dir))

    return lines, ready


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("map_dir", type=Path)
    ap.add_argument("--pose-source", choices=["mcl", "cuda_ndt", "ndt"], default="cuda_ndt")
    ap.add_argument("--grid-yaml", default="occupancy_grid.yaml",
                     help="grid yaml filename (relative to map_dir) to check "
                          "when pose_source needs a grid, or one happens to exist "
                          "(default: occupancy_grid.yaml)")
    ap.add_argument("--autoware-setup", default=None,
                     help="path to Autoware's setup.bash (default: autodetect "
                          "$AUTOWARE_SETUP_BASH or /opt/autoware/*/setup.bash)")
    args = ap.parse_args()

    map_dir = args.map_dir
    print(f"map: {map_dir}")
    if not map_dir.is_dir():
        print(f"  ERROR: {map_dir} is not a directory")
        return 1

    autoware_setup = find_autoware_setup(args.autoware_setup)
    lines, ready = run_check(map_dir, args.pose_source, args.grid_yaml, autoware_setup)
    for line in lines:
        print(line.render())

    if ready:
        print(f"READY for pose_source:={args.pose_source}")
        return 0
    print(f"NOT READY for pose_source:={args.pose_source}")
    return 1


if __name__ == "__main__":
    sys.exit(main())
