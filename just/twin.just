# Car A's digital twin on Isaac Sim (roadmap 15, docs/guides/isaac-twin.md): the vehicle
# spec's generated assets and their checks, the COSS world, and Autoware driving the twin.
#
#   just twin generate        the rig, catalog, spec_hash and VLP-16 lidar, from the spec
#   just twin check           stale generated files, or the spec and the twin instance apart
#   just twin vehicle-check   the twin car alone on a flat plane: topics, frames, accel/brake/steer
#
# scenario-world-bridge is an external checkout (SWB_ROOT, default ~/repos/swb-twin), and
# Isaac Sim 6.1 a pip venv (ISAAC_ROOT, default /mnt/p5plus/aeon/isaac-sim).

set working-directory := '..'

swb := env_var_or_default('SWB_ROOT', env_var('HOME') / 'repos/swb-twin')
isaac_root := env_var_or_default('ISAAC_ROOT', '/mnt/p5plus/aeon/isaac-sim')
twin := 'src/simulation/autosdv_twin'
spec := twin / 'vehicle/autosdv_car_a.yaml'
dae := 'src/vehicle/autosdv_vehicle_launch/autosdv_vehicle_description/mesh/car_a.dae'
# The twin's ROS domain. Never one another stack uses: aib's own default ego pins 1.
domain := env_var_or_default('TWIN_DOMAIN', '47')

[private]
default:
    @just --list twin

# Write the twin's generated files from its vehicle spec
generate:
    PYTHONPATH={{swb}}/src/autoware_isaac_bridge/src/aib python3 -m aib.gen --spec {{spec}} -o {{twin}}/isaac/autosdv_car_a.rig.yaml
    PYTHONPATH={{swb}}/src/sim_vehicle_spec python3 -m sim_vehicle_spec.gen --spec {{spec}} --catalog {{twin}}/catalogs/vehicles.xosc --spec-hash {{twin}}/isaac/spec_hash.yaml
    {{isaac_root}}/venv/bin/python {{twin}}/isaac/make_lidars.py {{twin}}/isaac/lidars
    {{isaac_root}}/venv/bin/python {{twin}}/body/make_body.py {{twin}}/isaac/body --dae {{dae}}

# Fail when a generated file is stale or the spec and the twin instance disagree
check:
    #!/usr/bin/env bash
    set -euo pipefail
    PYTHONPATH={{swb}}/src/autoware_isaac_bridge/src/aib python3 -m aib.gen --spec {{spec}} -o {{twin}}/isaac/autosdv_car_a.rig.yaml --check
    PYTHONPATH={{swb}}/src/sim_vehicle_spec python3 -m sim_vehicle_spec.gen --spec {{spec}} --catalog {{twin}}/catalogs/vehicles.xosc --spec-hash {{twin}}/isaac/spec_hash.yaml --check
    t=$(mktemp -d tmp/twin_check.XXXX)
    trap 'rm -rf "$t"' EXIT
    {{isaac_root}}/venv/bin/python {{twin}}/isaac/make_lidars.py "$t/lidars" >/dev/null
    {{isaac_root}}/venv/bin/python {{twin}}/body/make_body.py "$t/body" --dae "$t/car_a.dae" >/dev/null
    diff -r "$t/lidars" {{twin}}/isaac/lidars >/dev/null || { echo "lidars: STALE, run just twin generate"; exit 1; }
    diff -r "$t/body" {{twin}}/isaac/body >/dev/null || { echo "body: STALE, run just twin generate"; exit 1; }
    cmp -s "$t/car_a.dae" {{dae}} || { echo "car_a.dae: STALE, run just twin generate"; exit 1; }
    set +u; source install/setup.bash >/dev/null 2>&1; set -u
    python3 -m pytest -q {{twin}}/test

# The environment every twin process runs in: the twin's DDS profile, CUDA without another
# user's MPS daemon, Isaac, the twin's domain
[private]
env:
    #!/usr/bin/env bash
    mkdir -p tmp/no-mps
    # cuda_ndt compiles its kernels with NVRTC at start-up, and an NVRTC older than the GPU
    # refuses its architecture (CUDA 12.6 on an RTX 5090, sm_120): the newest toolkit wins
    cuda_lib=${TWIN_CUDA_LIB:-$(ls -d /usr/local/cuda-*.*/lib64 2>/dev/null | sort -V | tail -1)}
    cat <<EOF
    export CYCLONEDDS_URI=file://$PWD/{{twin}}/config/cyclonedds-unicast-lo.xml
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CUDA_MPS_PIPE_DIRECTORY=$PWD/tmp/no-mps CUDA_MPS_LOG_DIRECTORY=$PWD/tmp/no-mps
    export ISAAC_ROOT={{isaac_root}}
    export ROS_DOMAIN_ID={{domain}}
    export TWIN_CUDA_LIB=$cuda_lib
    EOF

# The twin car alone on a flat plane, against its spec (aib's vehicle-check): topics, types,
# rates, frames, stamps, /robot_description, then acceleration, braking and steering steps
vehicle-check:
    #!/usr/bin/env bash
    set -eo pipefail
    eval "$(just twin env)"
    set +u; source install/setup.bash >/dev/null 2>&1
    out=$PWD/tmp/twin_vc/$(date +%F_%H-%M-%S)
    echo "[twin] vehicle-check on ROS domain $ROS_DOMAIN_ID, report in $out"
    just --justfile {{swb}}/src/autoware_isaac_bridge/justfile vehicle-check \
        ego:=$PWD/{{twin}}/isaac/ego.yaml out:=$out \
        spec:=$PWD/{{spec}} vehicle_model:=autosdv_vehicle sensor_model:=autosdv_sensor_kit \
        config_dir:=$PWD/install/individual_params/share/individual_params/config/twin/autosdv_sensor_kit

# The COSS world for Isaac, from the scan (scenario-world-bridge scripts/ntu_twin, site coss):
# the map cloud coloured from the survey's originals, its ground (DTM and orthophoto), the
# objects' meshes, the USD world. Into data/coss-isaac/ (gitignored); minutes, not hours.
# `from` picks the first step to (re)run: color, ground, refill, smooth, meshes, usd.
world from='color':
    #!/usr/bin/env bash
    set -eo pipefail
    eval "$(just twin env)"
    py=${TWIN_TOOLS_PY:-/mnt/p5plus/aeon/twin-tools/bin/python}
    s={{swb}}/scripts/ntu_twin
    out=$PWD/data/coss-isaac
    src=$PWD/data/COSS-map-planning
    originals="${COSS_ORIGINALS:-$HOME/nas/autoveh/dataset/2024-04 COSS Map HDMap/點雲}"
    export SWB_TWIN_SITE=coss COSS_MAP_DIR=$out/map
    mkdir -p "$out/logs" "$out/map"
    # The map dir aib and Autoware read: the lanelets, the projector and the very PCD the world
    # is meshed from, so NDT localizes against what the lidar sees.
    for f in lanelet2_map.osm map_projector_info.yaml pointcloud_map.pcd; do ln -sfn "$src/$f" "$out/map/$f"; done
    run() { echo "[twin world] $1"; shift; env -u PYTHONPATH "$py" "$@" 2>&1 | tee -a "$out/logs/world.log"; }
    steps=(color ground refill smooth meshes usd); go=0
    for st in "${steps[@]}"; do
        [ "$st" = "{{from}}" ] && go=1
        [ $go = 1 ] || continue
        case $st in
            color)  run "colour transfer" "$s/color_transfer.py" --site coss "$out/map/pointcloud_map.pcd" "$out/tiles" \
                        "$originals/pt01_TWD97.pcd" "$originals/pt02_TWD97.pcd" "$originals/pt03_TWD97.pcd" --tight-tol 0.06 --fallback 1.5 ;;
            ground) run "ground: zmin, DTM, orthophoto" "$s/ground_ortho.py" "$out/tiles" "$out/ground" --workers 4 ;;
            refill) run "orthophoto refill" "$s/ortho_refill.py" "$out/ground/ortho" "$out/ground/ortho_refill" --workers 4 ;;
            smooth) run "DTM smoothing" "$s/smooth_dtm.py" "$out/ground" --lanes "$out/map/lanelet2_map.osm" ;;
            meshes) run "meshes" "$s/build_meshes.py" "$out/tiles" "$out/ground" "$out/meshes" --workers 2 \
                        --lanes "$out/map/lanelet2_map.osm" ;;
            usd)    echo "[twin world] USD"; env -u PYTHONPATH {{isaac_root}}/venv/bin/python "$s/world_usd.py" \
                        "$out/meshes" "$out/ground/ortho_refill" "$out/map" --clear-road "$out/ground" --clear-margin 0.5 \
                        2>&1 | tee -a "$out/logs/world.log" ;;
        esac
    done
    [ $go = 1 ] || { echo "from: one of ${steps[*]}" >&2; exit 2; }

# Bring the twin up on the COSS world: aib's runner (the world, car A, its sensors, on
# TWIN_DOMAIN) at the route's start, and AutoSDV's Autoware (twin_autoware.launch.xml).
# Both in process groups of their own, recorded in tmp/twin.pgids for `just twin down`.
up route='coss_loop' world='data/coss-isaac/map/isaac/world.yaml':
    #!/usr/bin/env bash
    set -eo pipefail
    [ -f tmp/twin.pgids ] && { echo "tmp/twin.pgids exists: just twin down first" >&2; exit 1; }
    [ -f {{world}} ] || { echo "no world {{world}}: just twin world" >&2; exit 1; }
    set +u; source scripts/env.sh >/dev/null 2>&1; set -u
    eval "$(just twin env)"
    mkdir -p tmp/twin_run
    pose=$(python3 {{twin}}/scripts/twin_go.py --spawn --route {{route}})
    echo "[twin] domain $ROS_DOMAIN_ID, spawn $pose, logs in tmp/twin_run/"
    setsid bash -c "unset DISPLAY; exec just --justfile {{swb}}/src/autoware_isaac_bridge/justfile isaac-ego \
        ego:=$PWD/{{twin}}/isaac/ego.yaml world:=$(realpath {{world}}) \
        pose:=$pose realtime:=true" > tmp/twin_run/runner.log 2>&1 < /dev/null &
    echo $! > tmp/twin.pgids
    LD_LIBRARY_PATH=$TWIN_CUDA_LIB:$LD_LIBRARY_PATH setsid play_launch launch --disable-web-ui --log-dir tmp/twin_run/play_log autosdv_twin twin_autoware.launch.xml \
        twin_map_path:=$PWD/data/coss-isaac/map > tmp/twin_run/autoware.log 2>&1 < /dev/null &
    echo $! >> tmp/twin.pgids
    echo "[twin] up: runner pgid $(sed -n 1p tmp/twin.pgids), Autoware pgid $(sed -n 2p tmp/twin.pgids)"

# Localize the twin at the route's start, set its goal, engage, wait for arrival
drive route='coss_loop':
    #!/usr/bin/env bash
    set -eo pipefail
    set +u; source scripts/env.sh >/dev/null 2>&1; set -u
    eval "$(just twin env)"
    python3 {{twin}}/scripts/twin_go.py --route {{route}}

# Stop what `just twin up` started (its process groups only)
down:
    #!/usr/bin/env bash
    [ -f tmp/twin.pgids ] || { echo "nothing up (no tmp/twin.pgids)"; exit 0; }
    for g in $(cat tmp/twin.pgids); do kill -INT -"$g" 2>/dev/null || true; done
    sleep 15
    for g in $(cat tmp/twin.pgids); do kill -TERM -"$g" 2>/dev/null || true; done
    sleep 5
    for g in $(cat tmp/twin.pgids); do kill -KILL -"$g" 2>/dev/null || true; done
    rm -f tmp/twin.pgids
    echo "[twin] down"
