# planning_profile:=rc

Autoware's planning files, changed where the RC car (and the classroom it drives in)
needs them. Loaded by `launch/components/autosdv_planning_component.launch.xml` when
`planning_profile:=rc`; the default `stock` loads `autoware_launch`'s files unchanged.
Each file here is a copy of upstream's (Autoware 1.5.0) with the changed lines marked
`AutoSDV rc:`. Re-copy after an Autoware upgrade.

| file | change | why |
|---|---|---|
| `start_planner.param.yaml` | `enable_back` false, freespace pull-out off | the car cannot reverse; a pull-out that backs up first leaves it parked at the start (Lab 3's second lap, from a stop 13 deg off the lane) |
| `goal_planner.param.yaml` | backward-arc and freespace parking off | both reverse; freespace parking also runs its own planning thread, and the behavior planner once crashed tearing the goal planner's threads down on a route change |
| `behavior_path_planner.param.yaml` | path interval 2.0 -> 0.5 m | a 2 m interval cuts a 1.5 m corner into straight chords |
| (module preset) | `launch_start_planner_module` false | from a stop a few degrees off the lane its pull-outs reverse, or with reversing off find no forward path and hold the car; a loop has nothing to pull out of. Set in the component, not a file |
| `mission_planner.param.yaml` | arrival overshoot 2.0 -> 0.2 m | a lap is one route whose goal sits about 0.5-1 m behind the start; with 2.0 the route counts as arrived the moment Auto is pressed |
| `behavior_path_planner.param.yaml` | `backward_path_length` 5.0 -> 0.2 m | the path behind the car reached the goal lanelet behind it and was cut at the goal, so the car never left |
| `path_optimizer.param.yaml` | `enable_outside_drivable_area_stop` true -> false | a lap's path wraps round the loop; the drivable-area check misjudges it and holds the car at 0 m/s at the start. The lanes, the speed limit and the transmitter remain |
| `trajectory_checker.param.yaml` | curvature 1.0 -> 1.2 | the car turns at 0.88 m radius (curvature 1.14); 1.0 rejects curves it can drive |

**Copies to tune, unchanged.** `obstacle_stop.param.yaml`, `obstacle_slow_down.param.yaml`
and `velocity_smoother.param.yaml` are byte-for-byte copies of upstream's (Autoware 1.5.0,
2026-11-23), here so a Lab 3 team can tune them in its fork; the stack reads `/opt`
otherwise, which is root-owned. Their values are upstream's on purpose: nothing was
pre-tuned. `docs/guides/lab3-tuning.md` says what each parameter does. Control's four
are in `../control_rc/`.

Used by Lab 3 (`lab3/run.sh`, `lab3_sim_lap`).

**One goal per lap.** With these, a single 2D Goal Pose just behind the start drives the whole
loop (RC simulator, 2026-11-16: 96.2 m in 122 s). The goal must lie in the lanelet *before* the
start's: with goal and start in one lanelet Autoware routes the 1 m behind the car, and Auto
never becomes available. Measured: start 0.1, 0.3 or 0.5 m past a lanelet boundary with the goal
1.0 m behind, all arrived; goal 0.5 m behind with the start 0.3 m past (goal 0.2 m before the
boundary) did not move. The car stops about 0.3 m short of its goal (the longitudinal PID's
smooth stop, `stopping_state_stop_dist` 0.49). Every other planning file
comes from `autoware_launch` unchanged.
