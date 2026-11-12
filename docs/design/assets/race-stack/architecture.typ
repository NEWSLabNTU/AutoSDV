// autoware-drift on the AutoSDV cars: the target architecture, on the car.
// Spec: docs/superpowers/specs/2026-11-12-autoware-drift-integration-design.md
// Render: typst compile --format svg architecture.typ   (or --format png --ppi 160)
#import "style.typ": *
#show: page-setup

#align(center, text(size: 13pt, weight: "bold")[autoware-drift on the AutoSDV cars: `just race run`])
#v(4pt)

#diagram(
  spacing: (13mm, 9mm),
  edge-stroke: 0.8pt + dim,
  mark-scale: 80%,

  // ── sensors ────────────────────────────────────────────────────────────
  tag((0, -0.7), [Sensors], s-hw, <t-sen>),
  bx((0, 0), title[LiDAR][VLP-16 (A) / Robin-W (B, C)], c-hw, s-hw, <lidar>, w: 32mm),
  bx((0, 1.6), title[ZED X Mini, front-left][stereo + IMU · topic `zed`], c-hw, s-hw, <zfl>, w: 32mm, new: true),
  bx((0, 2.8), title[ZED X Mini, front-right][stereo], c-hw, s-hw, <zfr>, w: 32mm, new: true),
  bx((0, 4.0), title[ZED X Mini, rear][stereo], c-hw, s-hw, <zr>, w: 32mm, new: true),
  group((<t-sen>, <lidar>, <zr>), c-hw, s-hw),

  // ── estimation ─────────────────────────────────────────────────────────
  bx((1.5, 0), title[cuda_ndt][scan matching on the COSS map], c-sdv, s-sdv, <ndt>, w: 34mm),
  bx((1.5, 2.8), title[rig_localization][cuVSLAM ×3 + GTSAM smoother \ + no-lateral wheel row, stereo scale], c-rig, s-rig, <rig>, w: 38mm, new: true),
  bx((3.0, 2.8), title[race_estimator][single-track filter \ x, y, yaw, vx, vy, r], c-race, s-race, <est>, w: 34mm, new: true),

  // ── race stack ─────────────────────────────────────────────────────────
  tag((4.5, -0.7), [autoware-drift race stack], s-race, <t-race>),
  bx((4.5, 0.6), title[race_planner node][L1 min-time re-plan, 1 s \ scale profile, `v_max`], c-race, s-race, <rp>, w: 38mm, new: true),
  bx((4.5, 4.0), title[race_controller][C++ acados NMPC, 50 Hz \ path, speed, sideslip, yaw rate], c-race, s-race, <rc>, w: 38mm, new: true),
  group((<t-race>, <rp>, <rc>), c-race, s-race),

  // ── inputs from AutoSDV's config ───────────────────────────────────────
  bx((3.0, -0.9), title[course + car][COSS course file (GLIM map, lanelet loop, grid clearance) \ per-car `vehicle_spec.yaml`], c-sdv, s-sdv, <cfg>, w: 52mm, new: true),
  bx((6.2, -0.9), title[operator cap][`speed_cap:=` → VelocityLimit], c-sdv, s-sdv, <cap>, w: 32mm, new: true),

  // ── Autoware (kept) ────────────────────────────────────────────────────
  tag((6.4, 0.0), [Autoware (kept)], s-aw, <t-aw>),
  bx((6.4, 0.75), title[mission planner, ADAPI][route into AUTONOMOUS], c-aw, s-aw, <mission>, w: 36mm),
  bx((6.4, 1.95), title[EKF][kinematic_state for Autoware], c-aw, s-aw, <ekf>, w: 36mm),
  bx((6.4, 3.15), title[op modes, MRM handler][diag graph `autosdv-race-main`], c-aw, s-aw, <modes>, w: 36mm),
  bx((6.4, 4.35), title[vehicle_cmd_gate][`vel_lim` = cap + margin], c-aw, s-aw, <gate>, w: 36mm),
  group((<t-aw>, <mission>, <gate>), c-aw, s-aw),

  // ── vehicle ────────────────────────────────────────────────────────────
  bx((4.5, 5.7), title[autosdv_vehicle_interface][steering: per-car ratio · *accel → ESC plug-in* (race only) \ *overspeed guard* · per-test `max_pwm` ceiling], c-sdv, s-sdv, <vi>, w: 66mm),
  bx((1.5, 5.7), title[PCA9685 → servo, ESC][hall sensor (rear wheel)], c-hw, s-hw, <hw>, w: 36mm),

  // ── edges ──────────────────────────────────────────────────────────────
  edge(<lidar>, <ndt>, "-|>", topic[points]),
  edge(<zfl>, <rig>, "-|>"),
  edge(<zfr>, <rig>, "-|>", topic[images]),
  edge(<zr>, <rig>, "-|>"),
  edge(<ndt>, (3.0, 0), <est>, "-|>", topic[pose 10 Hz], label-pos: 0.2),
  edge(<ndt>, (3.0, 0), (5.6, 0), (5.6, 1.95), <ekf>, "-|>", label-pos: 0.6),
  edge(<rig>, <est>, "-|>", topic[vx, vy, wz 100 Hz]),
  edge(<zfl>, (2.25, 1.6), (2.25, 2.25), (3.0, 2.25), <est>, "-|>", topic[IMU], label-pos: 0.15, label-side: left),
  edge(<rig>, (1.5, 3.4), (5.75, 3.4), (5.75, 1.95), <ekf>, "--|>", topic[twist (preset `rigloc`)], label-pos: 0.3, label-side: left),
  edge(<est>, <rp>, "-|>", topic[/race/state], label-side: left),
  edge(<est>, <rc>, "-|>", topic[/race/state], label-side: right),
  edge(<cfg>, <rp>, "-|>"),
  edge(<cap>, <rp>, "-|>", label-side: left),
  edge(<mission>, <rp>, "-|>", topic[route], label-side: right),
  edge(<rp>, <rc>, "-|>", topic[/race/plan], label-side: left),
  edge(<rp>, <modes>, "-|>", topic[trajectory], label-side: left, label-pos: 0.3),
  edge(<rc>, <gate>, "-|>", topic[follower control_cmd], label-side: right),
  edge(<gate>, (6.4, 5.7), <vi>, "-|>", topic[control_cmd], label-side: left, label-pos: 0.3),
  edge(<vi>, <hw>, "-|>", topic[PWM]),
  edge(<hw>, <vi>, "-|>", topic[hall edges], bend: -18deg, label-side: right),
  edge(<vi>, <est>, "-|>", topic[wheel speed, steer echo], label-pos: 0.45, label-side: left),
)

#v(6pt)
#legend()
#v(2pt)
#text(size: 8.5pt, fill: dim)[Not running in a race: Autoware's trajectory follower and scenario-planning output, lane-departure checker, AEB. Everything here is opt-in; `just launch` and the labs are unchanged.]
