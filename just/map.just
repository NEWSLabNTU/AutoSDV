# Map directory validation and occupancy-grid construction.
#
# Recipes here run from the REPO ROOT; see the note in just/tool.just.
set working-directory := '..'

# `just <module>` runs the module's FIRST recipe, so it has to be harmless.
[private]
default:
    @just --list map

# Check a map directory is usable for a given pose_source (default: cuda_ndt).
# Verifies lanelet2/projector/PCD/grid presence, and -- the point of the
# whole check -- that the occupancy grid's frame matches the lanelet2 map's,
# which catches the "grid built in the wrong frame" failure class.
# Further flags pass through, e.g. --grid-yaml NAME to check a grid variant
# other than occupancy_grid.yaml, or --autoware-setup PATH.
# just --list shows only the LAST comment line, so the description goes here.
# Validate a map directory for a pose_source (default: cuda_ndt)
check MAP_DIR POSE_SOURCE="cuda_ndt" *FLAGS:
    python3 ./scripts/map/check_map.py "{{MAP_DIR}}" --pose-source "{{POSE_SOURCE}}" {{FLAGS}}

# Build MAP_DIR/occupancy_grid.{pgm,yaml} by slicing a height band out of the
# PCD map already in MAP_DIR, record how it was built in autosdv_map.yaml, then
# validate the result for pose_source:=mcl.
#
# The z band is the one judgement you have to make, and it is not forgiving:
# a wrong band yields a valid-looking grid that localizes badly rather than an
# error. Run without FLAGS to see the height distribution, the estimated ground
# level and a suggested band, then re-run with them:
#
#   just map grid-from-pcd data/COSS-map-planning
#   just map grid-from-pcd data/COSS-map-planning --z-min 9.1 --z-max 9.4
#
# Further flags pass straight through: --resolution (default 0.05 m/px), and
# --min-points (points needed in the band before a cell counts as occupied).
# just --list shows only the LAST comment line, so the description goes here.
# Build an occupancy grid by slicing a height band out of the PCD map
grid-from-pcd MAP_DIR *FLAGS:
    #!/usr/bin/env bash
    set -euo pipefail
    PCD="{{MAP_DIR}}/pointcloud_map.pcd"
    if [ ! -f "$PCD" ]; then
        echo "no $PCD -- this recipe slices an existing PCD map." >&2
        echo "For a site with no PCD, build the grid from a recorded drive:" >&2
        echo "  just map grid-from-bag <bag> {{MAP_DIR}}" >&2
        exit 1
    fi
    python3 ./scripts/map/pcd_to_pgm.py "$PCD" "{{MAP_DIR}}/occupancy_grid" \
        --sidecar {{FLAGS}}
    echo
    # Addressed through the module, because `just` here resolves against the
    # root justfile (the cwd), where `check` does not exist.
    just map check "{{MAP_DIR}}" mcl

# Build MAP_DIR/occupancy_grid.{pgm,yaml} by accumulating 2-D scans from a
# recorded drive (BAG) at their ground-truth poses, for a site with no PCD map.
# Records provenance in autosdv_map.yaml, then validates for pose_source:=mcl.
#
# The band here is relative to the scan plane rather than to site ground level,
# so its defaults (-0.15..0.15 m) are meaningful; override them, and
# --resolution (default 0.1) / --min-hits (default 3), by passing them through.
# just --list shows only the LAST comment line, so the description goes here.
# Build an occupancy grid by accumulating 2-D scans from a recorded drive
grid-from-bag BAG MAP_DIR *FLAGS:
    #!/usr/bin/env bash
    set -euo pipefail
    python3 ./scripts/2dlidar/scan_accumulate_grid.py "{{BAG}}" \
        "{{MAP_DIR}}/occupancy_grid" --sidecar {{FLAGS}}
    echo
    just map check "{{MAP_DIR}}" mcl
