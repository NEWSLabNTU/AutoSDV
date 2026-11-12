// Where each part lives: repositories, packages, and what uses what.
// Spec: docs/superpowers/specs/2026-11-12-autoware-drift-integration-design.md ("Where things go")
// Render: typst compile --format svg repos.typ
#import "style.typ": *
#show: page-setup

#align(center, text(size: 13pt, weight: "bold")[Repositories and packages])
#v(4pt)

#let pk(pos, name, sub, fill, stroke, id, w: 60mm, new: false) = bx(pos, title(mono(name), sub), fill, stroke, id, w: w, new: new)
#let needs(t) = text(fill: s-hw.darken(25%), style: "italic")[needs #t]

#diagram(
  spacing: (26mm, 7mm),
  edge-stroke: 0.8pt + dim,
  mark-scale: 80%,

  // ── autoware-drift ─────────────────────────────────────────────────────
  tag((0, -1), [autoware-drift (generic Autoware) \ #mono[src/planning/external/autoware-drift], branch #mono[main]], s-race, <t-race>),
  pk((0, 0), [race_launch], [generic `race.launch.xml`: course, spec, scale, cap], c-race, s-race, <rl>, new: true),
  pk((0, 1), [race_planner_ros], [planner node around `live.Driver` \ (imports `race_planner`)], c-race, s-race, <node>, new: true),
  pk((0, 2), [race_controller], [C++ acados NMPC component, 50 Hz, \ checked against `tracker.py` · #needs[acados]], c-race, s-race, <ctl>, new: true),
  pk((0, 3), [race_estimator], [`/race/state` (Python or C++: builder's call)], c-race, s-race, <est>, new: true),
  group((<node>, <est>), c-race, s-race, name: <nodes>),
  pk((0, 4.3), [race_msgs], [`RacePlan`: s, pose, speed, sideslip, yaw rate, mode], c-race, s-race, <msgs>, new: true),
  pk((0, 5.5), [race_planner], [Python library (exists) · \+ scale profile, `v_max`, \ `Track.from_lanelet2`, open differential if measured \ #needs[CasADi, IPOPT]], c-race, s-race, <lib>),
  group((<t-race>, <rl>, <lib>), c-race, s-race),

  // ── AutoSDV ────────────────────────────────────────────────────────────
  tag((1, -1), [AutoSDV], s-sdv, <t-sdv>),
  pk((1, 0), [just/race.just], [`course`, `check`, `sim`, `twin`, `run`], c-sdv, s-sdv, <just>, new: true),
  pk((1, 1), [autosdv_launch], [race launch on `waypoint_follow`'s pattern, \ `rigloc` preset, `autosdv-race-main.yaml`], c-sdv, s-sdv, <sl>),
  pk((1, 2), [autosdv_vehicle_interface], [acceleration pass-through, accel→ESC plug-in, \ overspeed guard (off by default)], c-sdv, s-sdv, <vi>),
  pk((1, 3), [autosdv_sensor_kit_launch], [three-ZED sensor suites (new names)], c-sdv, s-sdv, <kit>),
  pk((1, 4.3), [individual_params], [per car (A, B, C, twin, sim), new files: \ `vehicle_spec.yaml`, rig overlay, three-ZED calibration], c-sdv, s-sdv, <ip>),
  pk((1, 5.5), [data, config], [COSS course file, surface classes], c-sdv, s-sdv, <data>, new: true),
  group((<t-sdv>, <just>, <data>), c-sdv, s-sdv),

  // ── rig_localization ───────────────────────────────────────────────────
  tag((2, -1), [rig_localization \ #mono[src/localization/external/rig_localization]], s-rig, <t-rig>),
  pk((2, 1), [rig_localization], [\+ no-lateral wheel row, stereo scale, \ small-car settings · #needs[cuVSLAM, GTSAM]], c-rig, s-rig, <rig>, w: 58mm),
  group((<t-rig>, <rig>), c-rig, s-rig),

  // ── uses ───────────────────────────────────────────────────────────────
  edge(<rl>, <nodes>, "-|>", topic[launches], label-side: right),
  edge(<nodes>, <msgs>, "-|>", topic[use], label-side: right),
  edge(<just>, <sl>, "-|>"),
  edge(<sl>, <rl>, "-|>", topic[includes \ course, spec, cap], label-side: left, label-pos: 0.5),
  edge(<sl>, <rig>, "-|>", topic[`rigloc` preset launches]),
  edge(<sl>, <vi>, "-|>", topic[selects accel plug-in], label-side: left),
)

#v(6pt)
#legend()
#v(2pt)
#text(size: 8.5pt, fill: dim)[Each package with a third-party dependency (#emph[needs ...]) skips itself when it is missing, so `just build` without them is today's build. \ `drift_controller` and `drift_launch` stay in autoware-drift unported; they build without new dependencies.]
