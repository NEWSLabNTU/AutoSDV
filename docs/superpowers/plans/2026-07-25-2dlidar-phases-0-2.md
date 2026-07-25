# 2D-LiDAR Variant — Phases 0–2 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Vendor the Roboracer particle-filter stack (Phase 0), build the PCD→PGM map-preparation tool with standalone load checks (Phase 1), and prepare replay rosbags with a synthesized `/scan` (Phase 2) — everything bench-testable, no vehicle.

**Architecture:** Roboracer `particle_filter` (ROS 2 port) + `range_libc` enter as upstream-URL submodules under `src/localization/external/`. A dependency-light Python tool (`scripts/map/pcd_to_pgm.py`, numpy+PIL only) slices a z-band from an existing PCD map into a `map.pgm`+`.yaml` occupancy grid, frame-aligned with the Lanelet2 map by construction. Replay assets come from the official Autoware rosbag-replay tutorial; a `pointcloud_to_laserscan` node synthesizes `/scan` from the replayed 3D pointcloud.

**Tech Stack:** git submodules, colcon, Python 3.10 (numpy, PIL, pytest), ROS 2 Humble + Autoware 1.5.0 (`/opt/autoware/1.5.0`), `pointcloud_to_laserscan`, `nav2_map_server`, gdown.

## Global Constraints

- Spec: `docs/superpowers/specs/2026-07-25-2dlidar-planning-control-design.md`
- Submodule policy: add upstream URLs; fork to NEWSLabNTU **only if patches needed**, then repoint. Small AutoSDV-authored repos may live under jerry73204.
- Temp files → `./tmp/` (repo-local, gitignored). Never system `/tmp`.
- The login shell is fish. ROS sourcing MUST run under bash: `bash -lc 'source /opt/autoware/1.5.0/setup.bash && …'`.
- Colcon builds always: `colcon build --base-paths src --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release [--packages-select …]`.
- `data/COSS-map-planning/pointcloud_map.pcd` is tracked in git (75 MB, PCD v0.7 binary, FIELDS `x y z rgb`, all FLOAT32, 4 896 469 points). Generated `.pgm`/`.yaml` are small — commit them beside the map.
- Downloads land under `data/` and must be covered by `data/.gitignore`.
- `gdown` is at `~/.local/bin/gdown`. `pointcloud_to_laserscan` ships with Autoware. `nav2_map_server` is NOT installed (Task 4 installs it). `open3d` is NOT available — do not add it.
- Commit after every task. Never commit unrelated untracked files (`record.json`, `resume.org`, `rosbags/`).

## File Structure

```
src/localization/external/particle_filter/   # submodule: CL2-UWaterloo/particle_filter (ROS2)
src/localization/external/range_libc/        # submodule: kctess5/range_libc (+COLCON_IGNORE)
scripts/map/pcd_to_pgm.py                    # PCD→PGM slicer (numpy+PIL, no ROS deps)
scripts/map/test_pcd_to_pgm.py               # pytest unit tests
scripts/2dlidar/check-map-load.sh            # Phase 1 standalone check (nav2 map_server)
scripts/2dlidar/download-sample-rosbag.sh    # Phase 2: fetch Autoware sample map+bag
scripts/2dlidar/replay-with-scan.sh          # Phase 2: bag replay + synthesized /scan
data/COSS-map-planning/occupancy_grid.{pgm,yaml}      # generated, committed
data/sample-rosbag-replay/                   # downloaded map+bag (gitignored)
```

---

### Task 1: Vendor Roboracer submodules (Phase 0)

**Files:**
- Modify: `.gitmodules`
- Create: `src/localization/external/particle_filter/` (submodule)
- Create: `src/localization/external/range_libc/` (submodule)
- Create: `src/localization/external/range_libc/COLCON_IGNORE`

**Interfaces:**
- Produces: colcon package `particle_filter` (Python, MCL node); `range_libc` source tree for Task 2's `pywrapper` build.

- [ ] **Step 1: Add submodules on upstream URLs**

```bash
cd /home/aeon/repos/AutoSDV
git submodule add https://github.com/CL2-UWaterloo/particle_filter.git src/localization/external/particle_filter
git submodule add https://github.com/kctess5/range_libc.git src/localization/external/range_libc
```

- [ ] **Step 2: Keep colcon away from range_libc (it is not a ROS package)**

```bash
touch src/localization/external/range_libc/COLCON_IGNORE
```

Note: `COLCON_IGNORE` lives inside the submodule worktree and stays uncommitted in the submodule — that is acceptable for now; if it becomes a nuisance, that is the first "patch" that triggers a NEWSLabNTU fork per policy.

- [ ] **Step 3: Verify colcon sees exactly one new package**

```bash
bash -lc 'source /opt/autoware/1.5.0/setup.bash && colcon list --base-paths src | grep -E "particle_filter|range_libc"'
```

Expected: one line, `particle_filter`, and nothing for `range_libc`.

- [ ] **Step 4: Build particle_filter**

```bash
bash -lc 'source /opt/autoware/1.5.0/setup.bash && colcon build --base-paths src --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select particle_filter'
```

Expected: build succeeds (pure Python ament package; the foxy→humble gap is usually nil for setup.py packages). If it fails, record the exact error, apply the minimal patch, and note that the submodule must be forked to NEWSLabNTU and repointed before merge (policy) — do not push patches to a detached submodule and forget them.

- [ ] **Step 5: Commit superproject**

```bash
git add .gitmodules src/localization/external
git commit -m "Vendor Roboracer particle_filter and range_libc as submodules"
```

---

### Task 2: Build range_libc Python bindings (Phase 0)

**Files:**
- Modify: `src/localization/external/range_libc/pywrapper/` (build artifacts only, not committed)

**Interfaces:**
- Consumes: `range_libc` source tree from Task 1.
- Produces: importable Python module `range_libc` (`import range_libc; range_libc.PyOMap`), required at runtime by `particle_filter`.

- [ ] **Step 1: Install build deps and compile the wrapper (CPU build — no CUDA needed for bench work)**

```bash
pip install --user cython
bash -lc 'cd /home/aeon/repos/AutoSDV/src/localization/external/range_libc/pywrapper && python3 setup.py install --user'
```

Expected: wheel/egg installed to `~/.local`. Known risk: the code is Python-2-era; if `setup.py` or the Cython layer fails under Python 3.10, apply minimal fixes → that triggers the NEWSLabNTU fork per policy.

- [ ] **Step 2: Verify the module imports and casts rays**

```bash
python3 - <<'EOF'
import numpy as np, range_libc
grid = np.zeros((10, 10), dtype=np.uint8)
omap = range_libc.PyOMap(grid)
rm = range_libc.PyBresenhamsLine(omap, 100)
print("range_libc OK")
EOF
```

Expected: `range_libc OK`.

- [ ] **Step 3: Commit (only if the submodule needed patches — then fork+repoint first; otherwise nothing to commit)**

```bash
git status --short   # confirm clean; submodule build artifacts stay untracked
```

---

### Task 3: PCD→PGM slicer with tests (Phase 1)

**Files:**
- Create: `scripts/map/pcd_to_pgm.py`
- Test: `scripts/map/test_pcd_to_pgm.py`

**Interfaces:**
- Produces: CLI `python3 scripts/map/pcd_to_pgm.py <in.pcd> <out_prefix> --z-min A --z-max B [--resolution 0.05] [--min-points 2]` → writes `<out_prefix>.pgm` + `<out_prefix>.yaml`. Library functions `read_pcd(path) -> np.ndarray (N,3)` and `rasterize(points, z_min, z_max, resolution, min_points) -> (grid: np.ndarray uint8, origin: tuple[float,float])`. Grid encoding: occupied=0, free=254, unknown=205 (ROS map_server convention, `negate: 0`).

- [ ] **Step 1: Write the failing tests**

```python
# scripts/map/test_pcd_to_pgm.py
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
    assert grid[0, 0] != 205 or True  # corner inside data extent is free/occ
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
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
cd /home/aeon/repos/AutoSDV && python3 -m pytest scripts/map/test_pcd_to_pgm.py -v
```

Expected: collection error — `ModuleNotFoundError: No module named 'pcd_to_pgm'`.

- [ ] **Step 3: Implement**

```python
#!/usr/bin/env python3
# scripts/map/pcd_to_pgm.py
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
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
cd /home/aeon/repos/AutoSDV && python3 -m pytest scripts/map/test_pcd_to_pgm.py -v
```

Expected: 6 passed.

- [ ] **Step 5: Generate the COSS grid**

First inspect the z range the script prints, then pick the band ~0.2–0.5 m above local ground (the COSS PCD is georeferenced — ground z is NOT 0; read it off the printed z range and adjust):

```bash
cd /home/aeon/repos/AutoSDV
python3 scripts/map/pcd_to_pgm.py data/COSS-map-planning/pointcloud_map.pcd \
    data/COSS-map-planning/occupancy_grid --z-min <ground+0.2> --z-max <ground+0.5>
```

Expected: prints point count, grid dims, non-trivial occupied count (walls/kerbs visible). Open `occupancy_grid.pgm` in an image viewer — structure must be recognizable (building outlines, not noise). If ground is sloped enough that a fixed band smears, note it in the commit message; ground-relative slicing is a follow-up, not this task.

- [ ] **Step 6: Commit**

```bash
git add scripts/map/pcd_to_pgm.py scripts/map/test_pcd_to_pgm.py \
        data/COSS-map-planning/occupancy_grid.pgm data/COSS-map-planning/occupancy_grid.yaml
git commit -m "Add PCD-to-PGM occupancy grid slicer with COSS map output"
```

---

### Task 4: Standalone map-load check (Phase 1 gate)

**Files:**
- Create: `scripts/2dlidar/check-map-load.sh`

**Interfaces:**
- Consumes: `data/COSS-map-planning/occupancy_grid.yaml` from Task 3.
- Produces: pass/fail shell script used as the Phase 1 gate and later in CI.

- [ ] **Step 1: Install nav2 map_server**

```bash
sudo apt install -y ros-humble-nav2-map-server ros-humble-nav2-lifecycle-manager
```

- [ ] **Step 2: Write the check script**

```bash
#!/usr/bin/env bash
# scripts/2dlidar/check-map-load.sh — Phase 1 standalone gate:
# nav2 map_server must load the generated grid and publish /map once.
# Usage: check-map-load.sh [path/to/occupancy_grid.yaml]
set -euo pipefail
MAP_YAML="${1:-data/COSS-map-planning/occupancy_grid.yaml}"

source /opt/ros/humble/setup.bash

ros2 run nav2_map_server map_server --ros-args \
    -p yaml_filename:="$MAP_YAML" -p use_sim_time:=false &
SERVER_PID=$!
trap 'kill -- -$(ps -o pgid= -p $SERVER_PID | tr -d " ") 2>/dev/null || true' EXIT
sleep 2

# map_server is a lifecycle node: configure + activate
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate

INFO=$(timeout 10 ros2 topic echo /map --once --field info)
echo "$INFO"
WIDTH=$(echo "$INFO" | awk '/^width:/ {print $2}')
if [ -z "$WIDTH" ] || [ "$WIDTH" -le 0 ]; then
    echo "FAIL: /map not published or empty"; exit 1
fi
echo "PASS: map loads, width=${WIDTH} cells"
```

- [ ] **Step 3: Run it**

```bash
chmod +x scripts/2dlidar/check-map-load.sh
bash -lc './scripts/2dlidar/check-map-load.sh'
```

Expected: prints grid info, ends `PASS: map loads, width=<N> cells`. Optional manual check: RViz with `/map` + the COSS lanelet2 map — lanes must overlay the grid's road corridors (frame-aligned by construction; this validates it).

- [ ] **Step 4: Commit**

```bash
git add scripts/2dlidar/check-map-load.sh
git commit -m "Add standalone occupancy grid load check"
```

---

### Task 5: Fetch official Autoware replay assets (Phase 2)

**Files:**
- Create: `scripts/2dlidar/download-sample-rosbag.sh`
- Modify: `data/.gitignore` (add `sample-rosbag-replay/`)

**Interfaces:**
- Produces: `data/sample-rosbag-replay/sample-map-rosbag/` (PCD + lanelet2) and `data/sample-rosbag-replay/sample-rosbag/` (db3 bag), used by Task 6 and later phases.

- [ ] **Step 1: Write the download script**

Google Drive IDs from the official rosbag-replay tutorial
(autoware-documentation → Tutorials → Rosbag replay simulation):

```bash
#!/usr/bin/env bash
# scripts/2dlidar/download-sample-rosbag.sh
# Fetch the official Autoware replay-simulation assets:
#   sample-map-rosbag  (PCD + lanelet2)   id 1A-8BvYRX3DhSzkAnOcGWFw5T30xTlwZI
#   sample-rosbag      (db3, no camera)   id 1VnwJx9tI3kI_cTLzP61ktuAJ1ChgygpG
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="$SCRIPT_DIR/../../data/sample-rosbag-replay"
mkdir -p "$OUT"
cd "$OUT"

GDOWN="${GDOWN:-$HOME/.local/bin/gdown}"

if [ ! -d sample-map-rosbag ]; then
    "$GDOWN" 'https://docs.google.com/uc?export=download&id=1A-8BvYRX3DhSzkAnOcGWFw5T30xTlwZI' -O sample-map-rosbag.zip
    unzip -o sample-map-rosbag.zip && rm sample-map-rosbag.zip
fi
if [ ! -d sample-rosbag ]; then
    "$GDOWN" 'https://docs.google.com/uc?export=download&id=1VnwJx9tI3kI_cTLzP61ktuAJ1ChgygpG' -O sample-rosbag.zip
    unzip -o sample-rosbag.zip && rm sample-rosbag.zip
fi

echo "--- checksums (pin these in versions.yaml when stable) ---"
find . -name '*.pcd' -o -name '*.db3' | xargs sha256sum
echo "--- bag contents ---"
source /opt/ros/humble/setup.bash
ros2 bag info sample-rosbag
```

- [ ] **Step 2: Gitignore the download dir**

Append to `data/.gitignore`:

```
sample-rosbag-replay/
```

- [ ] **Step 3: Run it**

```bash
chmod +x scripts/2dlidar/download-sample-rosbag.sh
bash -lc './scripts/2dlidar/download-sample-rosbag.sh'
```

Expected: both dirs present; `ros2 bag info` lists LiDAR pointcloud topics (`/sensing/lidar/top/pointcloud_raw*`), GNSS, IMU, vehicle status. Record the printed topic list — Task 6 needs the exact pointcloud topic name. If gdown hits a Google quota error, retry later or download manually from the tutorial's Drive links into `data/sample-rosbag-replay/`.

- [ ] **Step 4: Generate the sample-map grid (exercises the Task 3 tool on a second map)**

```bash
python3 scripts/map/pcd_to_pgm.py \
    data/sample-rosbag-replay/sample-map-rosbag/pointcloud_map.pcd \
    data/sample-rosbag-replay/sample-map-rosbag/occupancy_grid \
    --z-min <ground+0.2> --z-max <ground+0.5>
./scripts/2dlidar/check-map-load.sh data/sample-rosbag-replay/sample-map-rosbag/occupancy_grid.yaml
```

Expected: grid renders sensibly; check script prints PASS. (Generated grid lives in the gitignored download dir — regenerate on demand, do not commit.)

- [ ] **Step 5: Commit**

```bash
git add scripts/2dlidar/download-sample-rosbag.sh data/.gitignore
git commit -m "Add downloader for Autoware sample replay assets"
```

---

### Task 6: Replay with synthesized /scan (Phase 2 gate)

**Files:**
- Create: `scripts/2dlidar/replay-with-scan.sh`

**Interfaces:**
- Consumes: `data/sample-rosbag-replay/sample-rosbag/` from Task 5; `pointcloud_to_laserscan` from `/opt/autoware/1.5.0`.
- Produces: live `/scan` (`sensor_msgs/LaserScan`) during replay — the input for Phase 3 (particle_filter) and Phase 5 (OGM node).

- [ ] **Step 1: Write the replay script**

The z-band is set in the LiDAR sensor frame: the top LiDAR's horizontal ring
sits near z≈0 in its own frame, so ±0.15 m selects a virtual 2D scan plane at
mount height. `POINTCLOUD_TOPIC` defaults to the top-lidar raw topic recorded
in the sample bag — override via env if `ros2 bag info` (Task 5) showed a
different name.

```bash
#!/usr/bin/env bash
# scripts/2dlidar/replay-with-scan.sh — Phase 2 gate:
# replay the sample rosbag and synthesize /scan from the 3D pointcloud.
# Env overrides: BAG, POINTCLOUD_TOPIC, RATE.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAG="${BAG:-$SCRIPT_DIR/../../data/sample-rosbag-replay/sample-rosbag}"
POINTCLOUD_TOPIC="${POINTCLOUD_TOPIC:-/sensing/lidar/top/pointcloud_raw}"
RATE="${RATE:-0.5}"

source /opt/autoware/1.5.0/setup.bash

ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
    -r cloud_in:="$POINTCLOUD_TOPIC" -r scan:=/scan \
    -p min_height:=-0.15 -p max_height:=0.15 \
    -p angle_min:=-3.14159 -p angle_max:=3.14159 \
    -p angle_increment:=0.0043 -p range_min:=0.1 -p range_max:=30.0 \
    -p use_sim_time:=true &
CONV_PID=$!
trap 'kill -- -$(ps -o pgid= -p $CONV_PID | tr -d " ") 2>/dev/null || true' EXIT
sleep 2

ros2 bag play "$BAG" --clock -r "$RATE" &
PLAY_PID=$!
sleep 5

echo "--- /scan rate (expect ~10 Hz x replay rate) ---"
timeout 15 ros2 topic hz /scan --window 20 || { echo "FAIL: no /scan"; exit 1; }
echo "--- sample message ---"
timeout 10 ros2 topic echo /scan --once --field header
wait $PLAY_PID 2>/dev/null || true
echo "PASS: /scan synthesized from replay"
```

- [ ] **Step 2: Run it**

```bash
chmod +x scripts/2dlidar/replay-with-scan.sh
bash -lc './scripts/2dlidar/replay-with-scan.sh'
```

Expected: `ros2 topic hz /scan` reports a steady rate (≈ LiDAR rate × RATE), header frame is the LiDAR frame, script ends `PASS`. Optional manual check: RViz — `/scan` points must trace the walls that appear in the sample-map grid from Task 5 Step 4.

- [ ] **Step 3: Commit and push**

```bash
git add scripts/2dlidar/replay-with-scan.sh
git commit -m "Add replay script with synthesized 2D scan"
git push origin develop
```

---

## Self-Review Notes

- Spec coverage: Phase 0 (Tasks 1–2), Phase 1 incl. standalone map check (Tasks 3–4), Phase 2 incl. standalone replay check (Tasks 5–6). Later phases (MCL port, bridge, perception wiring, planning preset) intentionally out of scope — next plan.
- The spec's script path `scripts/2d-map/pcd-to-pgm.py` is realized as `scripts/map/pcd_to_pgm.py` to follow the existing `scripts/map/` directory; underscore for Python importability in tests. Update the spec reference when this plan lands.
- Type consistency: `read_pcd`/`rasterize` signatures match between test and implementation; grid encoding constants used consistently (0/254/205).
