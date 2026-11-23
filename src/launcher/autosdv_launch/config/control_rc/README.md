# control_profile:=rc

Copies of four of Autoware's control files, byte for byte (Autoware 1.5.0,
`autoware_launch/config/control/`, 2026-11-23), so a Lab 3 team can tune them in its
fork; `/opt` is root-owned. Loaded by `launch/components/autosdv_control_component.launch.xml`
when `control_profile:=rc`, which follows `planning_profile` (so `lab3/run.sh`'s
`planning_profile:=rc` selects them); `stock` loads `autoware_launch`'s files.

| file | node | what a team may tune |
|---|---|---|
| `mpc.param.yaml` | the lateral controller (MPC) | weights, prediction horizon, steering filters; MPC stays the controller |
| `autonomous_emergency_braking.param.yaml` | AEB | the width it checks (`expand_width`), the distance it keeps, how long it holds a stop |
| `control_validator.param.yaml` | control validator | the deviation that triggers an emergency stop |
| `lane_departure_checker.param.yaml` | lane departure checker | footprint margins |

Values are upstream's on purpose: nothing was pre-tuned. Re-copy after an Autoware
upgrade. `docs/guides/lab3-tuning.md` says what each parameter does.
