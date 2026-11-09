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
    cat <<EOF
    export CYCLONEDDS_URI=file://$PWD/{{twin}}/config/cyclonedds-unicast-lo.xml
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CUDA_MPS_PIPE_DIRECTORY=$PWD/tmp/no-mps CUDA_MPS_LOG_DIRECTORY=$PWD/tmp/no-mps
    export ISAAC_ROOT={{isaac_root}}
    export ROS_DOMAIN_ID={{domain}}
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
