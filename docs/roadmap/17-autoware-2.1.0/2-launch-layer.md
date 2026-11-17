# Phase 2: launch layer

Every change is launch side; the golf cart had none at build time.

- `tier4_{map,localization,planning,control,perception}_launch` are gone: components are
  `autoware_launch/launch/components/component_*.launch.xml`, params in
  `autoware_*_config`. Ours to adapt: `launch/components/autosdv_{map,control,planning,
  perception,pursuit_control,dead_reckoning}_component.launch.xml`,
  `tier4_localization_component.launch.xml`, `aruco_localization.launch.xml`,
  `mcl_localization.launch.xml`, and the fork `src/localization/tier4_localization_launch`.
- `component_localization` reads `localization_config_pkg` and takes no param paths: with
  our own NDT/EKF/pose-initializer params we include the nodes ourselves.
- `automatic_pose_initializer` became `autoware_pose_initialization_requester`;
  `mission_planner` became `autoware_mission_planner_universe` (check
  `config/planning_rc/mission_planner.param.yaml`).
- Planning: `autoware_planning_launch/planning.launch.xml` takes every param path as an
  argument; the wrapper passes only what it overrides. Rebase each param copy: diff ours
  against 1.5.0 upstream, apply that diff onto 2.1.0 upstream (`patch -F3`).
- New required keys: EKF `max_pose_queue_size`, `max_twist_queue_size`,
  `diagnostics_publish_frequency`; NDT `publish_loaded_map`; the CUDA preprocessor's three.
  Selected per version, since 1.5.0 rejects nothing extra but 2.1.0 requires them.
- Diagnostic graphs: drop `/autoware/debug/tools`; `autosdv_rc_sim`'s
  `rc_sim_diagnostics.yaml` copies 1.5.0's `autoware-main.yaml` (its test compares with the
  installed one and fails on 2.1.0): one copy per version. Planning simulator:
  `launch_fault_injection`, `perception_simulation.launch.xml`.
- Nebula 1.2 (car A, VLP16): paths passed as sensor-kit arguments, so one kit serves both.

**Exit:** with play_launch >= 0.37.0, `autosdv.launch.yaml`, `logging_simulation.launch.yaml`
and the planning simulation resolve on 2.1.0 with no dead include, no missing parameter file
and no missing required key; 1.5.0 resolves as before.
