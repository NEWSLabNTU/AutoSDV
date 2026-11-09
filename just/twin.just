# Car A's digital twin on Isaac Sim (roadmap 15, docs/guides/isaac-twin.md): the vehicle
# spec's generated assets and their checks, the COSS world, and Autoware driving the twin.
#
#   just twin generate        the rig, catalog, spec_hash and VLP-16 lidar, from the spec
#   just twin check           stale generated files, or the spec and the twin instance apart
#
# scenario-world-bridge is an external checkout (SWB_ROOT, default ~/repos/swb-twin), and
# Isaac Sim 6.1 a pip venv (ISAAC_ROOT, default /mnt/p5plus/aeon/isaac-sim).

set working-directory := '..'

swb := env_var_or_default('SWB_ROOT', env_var('HOME') / 'repos/swb-twin')
isaac_root := env_var_or_default('ISAAC_ROOT', '/mnt/p5plus/aeon/isaac-sim')
twin := 'src/simulation/autosdv_twin'
spec := twin / 'vehicle/autosdv_car_a.yaml'

[private]
default:
    @just --list twin

# Write the twin's generated files from its vehicle spec
generate:
    PYTHONPATH={{swb}}/src/autoware_isaac_bridge/src/aib python3 -m aib.gen --spec {{spec}} -o {{twin}}/isaac/autosdv_car_a.rig.yaml
    PYTHONPATH={{swb}}/src/sim_vehicle_spec python3 -m sim_vehicle_spec.gen --spec {{spec}} --catalog {{twin}}/catalogs/vehicles.xosc --spec-hash {{twin}}/isaac/spec_hash.yaml
    {{isaac_root}}/venv/bin/python {{twin}}/isaac/make_lidars.py {{twin}}/isaac/lidars

# Fail when a generated file is stale or the spec and the twin instance disagree
check:
    PYTHONPATH={{swb}}/src/autoware_isaac_bridge/src/aib python3 -m aib.gen --spec {{spec}} -o {{twin}}/isaac/autosdv_car_a.rig.yaml --check
    PYTHONPATH={{swb}}/src/sim_vehicle_spec python3 -m sim_vehicle_spec.gen --spec {{spec}} --catalog {{twin}}/catalogs/vehicles.xosc --spec-hash {{twin}}/isaac/spec_hash.yaml --check
    bash -c 'source install/setup.bash >/dev/null 2>&1; python3 -m pytest -q {{twin}}/test'
