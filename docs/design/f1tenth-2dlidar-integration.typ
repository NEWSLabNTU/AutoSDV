#import "@preview/fletcher:0.5.8" as fletcher: diagram, node, edge

// ─────────────────────────────────────────────────────────────
// Palette
#let c-auto   = rgb("#1f6feb")   // AutoSDV / Autoware — kept (blue)
#let c-auto-b = rgb("#dce7fb")
#let c-f1     = rgb("#e8590c")   // borrowed from f1tenth (orange)
#let c-f1-b   = rgb("#fbe3d2")
#let c-new    = rgb("#c9184a")   // new glue code (red)
#let c-new-b  = rgb("#f7d6de")
#let c-off    = rgb("#6c757d")   // offline / one-time (gray)
#let c-off-b  = rgb("#e4e6e9")
#let c-drop   = rgb("#adb5bd")   // dropped (faded)

#let legend(color, body) = box(baseline: 0.15em)[#box(width: 0.85em, height: 0.85em, fill: color, radius: 1pt) #body]

// ─────────────────────────────────────────────────────────────
// Document setup
#set document(title: "A Low-Cost 2D-LiDAR Build for AutoSDV")
#set page(
  paper: "a4",
  margin: (x: 2cm, y: 2cm),
  numbering: "1",
  footer: context [
    #set text(8pt, fill: c-off)
    AutoSDV · low-cost 2D-LiDAR build
    #h(1fr)
    #counter(page).display("1 / 1", both: true)
  ],
)
#set text(font: ("Liberation Sans", "DejaVu Sans"), size: 10pt)
#set par(justify: true, leading: 0.62em)
#show heading: set block(above: 1.3em, below: 0.7em)
#show heading.where(level: 1): set text(fill: c-auto, size: 16pt)
#show heading.where(level: 2): set text(fill: rgb("#0b3a7a"), size: 12.5pt)
#set table(stroke: 0.4pt + c-off, inset: 6pt)
#show raw.where(block: true): set block(fill: rgb("#f6f8fa"), inset: 8pt, radius: 3pt, width: 100%)

// ─────────────────────────────────────────────────────────────
// Title block
#align(center)[
  #text(20pt, weight: "bold", fill: c-auto)[
    A Low-Cost 2D-LiDAR Build for AutoSDV
  ]
  #v(-0.3em)
  #text(12pt, fill: c-off)[Cheaper sensing + localization, unchanged Autoware planning]
  #v(0.3em)
  #text(9pt, fill: c-off)[NEWSLabNTU AutoSDV · design report · 2026-07-21]
]

#v(0.5em)
#line(length: 100%, stroke: 0.6pt + c-off)

// ─────────────────────────────────────────────────────────────
= Executive Summary

*Goal: a cheaper build variant of AutoSDV* — not a different robot. AutoSDV is
a research platform built on Autoware, and the 3D LiDAR is its single largest
sensor cost. This variant swaps that 3D LiDAR for an inexpensive *2D LiDAR* and
keeps *everything that makes AutoSDV a research platform* — Autoware's planning,
control, behaviors, and vehicle interface stay exactly as they are.

A 2D LiDAR cannot feed Autoware's 3D localization (NDT) or 3D perception
(CenterPoint), so those two modules are replaced/dropped. For localization we
borrow a *proven, MIT-licensed* 2D stack from f1tenth — `particle_filter`
(Monte-Carlo localization on a 2D occupancy grid). Its pose is bridged into
Autoware's existing `ekf_localizer`, so *planning and control see the same
`kinematic_state` they always have*.

The change is deliberately *shallow*: it touches only the sensing and
localization layers. One small new node (a pose-covariance relay) is the entire
custom code footprint. The costs paid for the lower price are (a) authoring a
Lanelet2 map as usual plus a 2D occupancy grid, and (b) loss of 3D object
perception — acceptable for a low-speed, low-cost research build.

#align(center, box(inset: 8pt, radius: 4pt, fill: c-off-b, width: 95%)[
  #set align(left)
  #set text(9.5pt)
  *Legend* #h(1em)
  #legend(c-auto-b)[Autoware — unchanged] #h(1em)
  #legend(c-f1-b)[Borrowed (f1tenth)] #h(1em)
  #legend(c-new-b)[New glue] #h(1em)
  #legend(c-off-b)[Offline map] #h(1em)
  #legend(c-drop)[Dropped]
])

// ─────────────────────────────────────────────────────────────
= Motivation — Why a 2D Variant

The LiDAR dominates the AutoSDV sensor bill. A 2D LiDAR is roughly an order of
magnitude cheaper than the 3D units AutoSDV ships, while the rest of the sensor
suite (IMU, USB camera, GNSS) is untouched. Indicative list prices (vary by
vendor and region — treat as ballpark, not a quote):

#table(
  columns: (auto, 1fr, auto),
  table.header([*Class*], [*Example units*], [*Indicative cost*]),
  [3D LiDAR — mechanical], [Velodyne VLP-32C], [\$\~10–30 k],
  [3D LiDAR — solid state], [Robin-W, Blickfeld Cube1], [\$\~1–5 k],
  [*2D LiDAR* — industrial], [Hokuyo UST-10LX, SICK TiM], [\$\~1–2 k],
  [*2D LiDAR* — hobby], [Slamtec RPLIDAR], [\$\~0.1–0.6 k],
)

The design constraint is what separates this from a toy conversion: *keep the
research value*. AutoSDV's worth is its Autoware planning/control stack and the
experiments it enables. So the variant lowers cost *only at the sensor +
localization layer* and preserves the full Autoware driving pipeline above it.

// ─────────────────────────────────────────────────────────────
= What Changes vs Stock AutoSDV

#table(
  columns: (auto, 1fr, 1fr, auto),
  table.header([*Layer*], [*Stock AutoSDV*], [*2D-LiDAR variant*], [*Status*]),
  [LiDAR], [3D (VLP-32C / Robin-W / Cube1)], [2D (Hokuyo)], [swap],
  [Localization], [NDT scan matching (3D)], [`particle_filter` (2D MCL)], [replace],
  [Geometry map], [PCD point cloud], [2D occupancy grid], [replace],
  [Vector map], [Lanelet2], [Lanelet2], [*unchanged*],
  [3D perception], [CenterPoint], [dropped → optional 2D obstacle stop], [reduce],
  [Planning], [behavior + motion], [behavior + motion], [*unchanged*],
  [Control], [`trajectory_follower`], [`trajectory_follower`], [*unchanged*],
  [Vehicle interface], [AutoSDV interface], [AutoSDV interface], [*unchanged*],
  [Custom code], [—], [pose-covariance relay (1 node)], [new],
)

Four rows change; the entire planning/control half of the stack — the research
surface — is untouched.

// ─────────────────────────────────────────────────────────────
#pagebreak()
= Architecture

The 2D front-end (LiDAR + `particle_filter`) replaces the 3D front-end. Its
pose is relayed into Autoware's `ekf_localizer`, which fuses it with the
existing twist estimate and emits the same `/localization/kinematic_state` that
planning and control already consume. The output side is *native Autoware* —
no adapter, no message translation at the vehicle.

#v(0.4em)
#align(center)[
#diagram(
  spacing: (13mm, 11mm),
  node-stroke: 0.6pt,
  node-corner-radius: 3pt,
  node-inset: 4pt,
  {
    let t(b) = text(6.5pt, raw(b))
    let n(b) = text(7.5pt, b)

    // front-end
    node((0, 0), n[urg_node], fill: c-auto-b, name: <lidar>)
    node((0, 1), n[IMU +\ vehicle vel], fill: c-auto-b, name: <imu>)
    node((1, -1), n[map.pgm], fill: c-off-b, name: <grid>)
    node((1, 0), n[*particle\ \_filter*], fill: c-f1-b, name: <pf>)

    // bridge
    node((2, 0), n[pose relay], fill: c-new-b, name: <relay>)
    node((2, 1), n[gyro_odometer], fill: c-auto-b, name: <gyro>)
    node((3, 0.5), n[ekf_localizer], fill: c-auto-b, name: <ekf>)

    // autoware planning + control (unchanged)
    node((4, -1), n[Lanelet2\ map], fill: c-off-b, name: <ll>)
    node((4, 0.5), n[planning], fill: c-auto-b, name: <plan>)
    node((5, 0.5), n[trajectory\ \_follower], fill: c-auto-b, name: <ctrl>)
    node((6, 0.5), n[vehicle\ iface], fill: c-auto-b, name: <vif>)

    edge(<lidar>, <pf>, "->", label: t("/scan"))
    edge(<grid>, <pf>, "->")
    edge(<pf>, <relay>, "->", label: t("pose"))
    edge(<imu>, <gyro>, "->", label: t("imu+vel"))
    edge(<relay>, <ekf>, "->", label: t("pose_est"))
    edge(<gyro>, <ekf>, "->", label: t("twist_est"))
    edge(<ll>, <plan>, "->", label: t("lanes"))
    edge(<ekf>, <plan>, "->", label: t("kinematic_state"))
    edge(<ekf>, <ctrl>, "->", bend: -30deg, label: t("pose"))
    edge(<plan>, <ctrl>, "->", label: t("trajectory"))
    edge(<ctrl>, <vif>, "->", label: t("control_cmd"))
  }
)
]

#align(center, text(8pt, fill: c-off)[
  #legend(c-f1-b)[borrowed] #h(0.8em) #legend(c-new-b)[new] #h(0.8em)
  #legend(c-auto-b)[unchanged Autoware] #h(0.8em) #legend(c-off-b)[offline map] ·
  everything right of `ekf_localizer` is stock AutoSDV
])

== The Localization Bridge

The one seam that needs code. `particle_filter` produces a 2D pose; Autoware's
`ekf_localizer` expects a pose estimate on the exact topic NDT used to publish:

#table(
  columns: (auto, 1fr),
  table.header([*`ekf_localizer` input*], [*Fed by the variant*]),
  [`…/pose_estimator/pose_with_covariance`], [`particle_filter` pose → *pose relay* (stamps covariance, reframes to `map`)],
  [`…/twist_estimator/twist_with_covariance`], [`gyro_odometer` — IMU + vehicle velocity (*already in AutoSDV*)],
)

`ekf_localizer` fuses the two into `/localization/kinematic_state` and the
`map→base_link` TF. Downstream, *nothing else in Autoware knows the LiDAR
changed*. If `particle_filter` can be configured to publish the right topic and
covariance directly, the relay collapses to launch config.

// ─────────────────────────────────────────────────────────────
= Sensor & Message Shapes

The change is, concretely, a swap of ROS 2 message types at two points: the
range sensor (both are consumed differently) and the localization pose (the
bridge). Planning/control messages are unchanged.

== Sensor Interface

#table(
  columns: (auto, 1fr, 1fr),
  table.header([*Signal*], [*Stock AutoSDV*], [*2D-LiDAR variant*]),
  [Range], [`sensor_msgs/PointCloud2` — `/sensing/…/points` (3D `x y z intensity`)], [`sensor_msgs/LaserScan` — `/scan` (1 plane: `ranges[]`, `angle_*`)],
  [IMU], [`sensor_msgs/Imu`], [`sensor_msgs/Imu` (unchanged)],
  [Wheel speed], [`autoware_vehicle_msgs/VelocityReport` → twist], [same (unchanged)],
  [GNSS], [`sensor_msgs/NavSatFix` → pose], [same (optional, for init)],
)

Only the range sensor changes: `PointCloud2` (3D) → `LaserScan` (2D). Every
other sensor interface is identical.

== Localization

#table(
  columns: (auto, 1fr, 1fr),
  table.header([*Aspect*], [*Stock — NDT*], [*Variant — particle_filter*]),
  [Algorithm], [NDT scan matching (3D registration)], [Monte-Carlo localization (2D, RangeLibc raycast)],
  [Map input], [`PointCloud2` (PCD)], [`nav_msgs/OccupancyGrid` (map_server)],
  [Scan input], [`PointCloud2` (3D)], [`sensor_msgs/LaserScan` (2D)],
  [Motion input], [ekf twist (IMU + wheel)], [`nav_msgs/Odometry` `/odom`],
  [Pose output], [`PoseWithCovarianceStamped` → `ekf_localizer`], [`nav_msgs/Odometry` + `PoseStamped` → *relay* → `ekf_localizer`],
  [Fusion], [`ekf_localizer` → `kinematic_state`], [`ekf_localizer` → `kinematic_state` (*same*)],
  [Init], [`pose_initializer` (GNSS / NDT / manual)], [`/initialpose` (manual RViz) or GNSS],
)

== Message Shapes (the two conversions)

#table(
  columns: (auto, 1fr, 1fr),
  table.header([*Role*], [*Stock AutoSDV msg*], [*Variant msg*]),
  [Range\ (sensor swap)],
    [`sensor_msgs/PointCloud2`\ `{header, height, width,`\ `fields[x,y,z,intensity], data[]}`],
    [`sensor_msgs/LaserScan`\ `{header, angle_min, angle_max,`\ `angle_increment, range_min/max,`\ `ranges[], intensities[]}`],
  [Ego pose\ (bridge)],
    [`geometry_msgs/`\ `PoseWithCovarianceStamped`\ `{header, pose.pose{position,`\ `orientation}, pose.covariance[36]}`],
    [`nav_msgs/Odometry` (from PF)\ `{header, child_frame_id,`\ `pose.pose{…}, pose.covariance[36],`\ `twist.twist{…}}`\ → relay reshapes to the\ Autoware msg on the left],
)

The *pose relay* exists to reshape the ego-pose row: `particle_filter`
`Odometry` / `PoseStamped` → `PoseWithCovarianceStamped`, populating a
realistic covariance so `ekf_localizer` weights it correctly.

// ─────────────────────────────────────────────────────────────
#pagebreak()
= Maps

The variant needs *two* offline maps. The Lanelet2 vector map is *the same
artifact stock AutoSDV already requires* for planning — no new burden. The only
new map is the 2D occupancy grid that replaces the PCD point cloud (cheaper to
build: one teleop lap + `slam_toolbox`, versus a survey-grade 3D cloud).

#align(center)[
#diagram(
  spacing: (13mm, 7mm),
  node-stroke: 0.55pt,
  node-corner-radius: 3pt,
  node-inset: 5pt,
  {
    let t(b) = text(7pt, raw(b))
    let n(b) = text(8pt, b)

    node((0, 0), n[map.pgm + .yaml\ (occupancy)], fill: c-f1-b, name: <grid>)
    node((0, 1), n[lanelet2_map\ .osm], fill: c-auto-b, name: <ll>)
    node((2, 0), n[particle_filter], fill: c-f1-b, name: <pf>)
    node((2, 1), n[planning], fill: c-auto-b, name: <plan>)
    edge(<grid>, <pf>, "->", label: t("occ grid"))
    edge(<ll>, <plan>, "->", label: t("lanes"))
  }
)
]

#table(
  columns: (0.7fr, 1fr, 1fr),
  table.header([*Dimension*], [*Stock geometry map (PCD)*], [*Variant geometry map (occupancy)*]),
  [Format], [`.pcd` — PCD v0.7 binary, `x y z [rgb]`], [`.pgm` — grayscale occupancy image + `.yaml`],
  [Frame], [Georeferenced WGS84 / MGRS], [Local metric; grid origin in `.yaml`],
  [Built with], [3D LiDAR SLAM (survey-grade)], [2D SLAM (`slam_toolbox`), one teleop lap],
  [Typical size], [10s of MB (27 MB sample)], [\~100 KB],
  [Effort], [*high*], [*low*],
)

#box(inset: 7pt, radius: 3pt, fill: c-auto-b, width: 100%)[
  #set text(9.5pt)
  *Vector map (Lanelet2) is unchanged.* `.osm` OSM-XML with lanes, stop lines,
  and regulatory elements — authored in Vector Map Builder / JOSM exactly as for
  stock AutoSDV. Align its origin to the occupancy grid frame.
]

#v(0.4em)
#box(inset: 7pt, radius: 3pt, fill: c-f1-b, width: 100%)[
  #set text(9.5pt)
  *Preferred grid source for existing sites: slice the PCD.* Crop the existing
  3D point-cloud map to a z-band at the 2D LiDAR mounting height (relative to
  local ground), project to XY, rasterize at 0.05 m → `map.pgm` + `.yaml`
  (`pcd-to-pgm.py`). Because the PCD and the Lanelet2 map share one
  georeferenced frame, the grid is *aligned to the vector map by construction*
  — no re-mapping, no manual alignment. `slam_toolbox` remains the path for
  new sites without a PCD. Caveats: sloped sites need ground-relative slicing;
  survey-day clutter becomes phantom occupancy (erase manually if it disturbs
  MCL).
]

== Perception Trade-off

With no 3D LiDAR there is no CenterPoint object detection. Planning runs on an
*empty or scan-derived* object list. Options: (a) *static-environment* mode —
no dynamic objects, planner follows the lane; (b) a *2D obstacle stop* —
cluster `/scan` or the occupancy grid into blockages that trigger Autoware's
stop behavior. Dynamic overtaking/avoidance is *not* available with a 2D-only
sensor. This is the main capability cost of the cheaper build.

// ─────────────────────────────────────────────────────────────
= Phased Integration

Phases 0–2 are *offline preparation* with standalone checks — no vehicle, no
full launch. Phases 3–7 run on *rosbag replay* (Autoware's official replay
simulation assets or an AutoSDV recording), so the entire pipeline is
bench-tested before hardware. Phase 8 is the only on-vehicle step.

#table(
  columns: (auto, 1fr, 1fr),
  table.header([*Phase*], [*Work*], [*Verify (standalone where possible)*]),
  [0], [*Vendor & build* — add Roboracer `particle_filter` + `range_libc` as submodules (upstream URLs; fork to NEWSLabNTU only if patched); `colcon build`], [Build succeeds],
  [1], [*Map preparation* — `pcd-to-pgm.py`: z-band slice of the site PCD → `map.pgm` + `.yaml` (frame-aligned with Lanelet2 by construction). Alt for new sites: teleop + `slam_toolbox`], [*Standalone check:* `map_server` loads the grid; RViz overlay of grid vs Lanelet2 lanes aligns; origin/resolution sane],
  [2], [*Rosbag preparation* — fetch Autoware `sample-map-rosbag` + `sample-rosbag` (or COSS bag via `just download-data`); synthesize `/scan` from the 3D pointcloud with `pointcloud_to_laserscan` (z-band at virtual mount height)], [*Standalone check:* replay renders `/scan` in RViz at expected rate; scan geometry matches walls in the grid],
  [3], [*Port MCL* — `particle_filter` on the prepared grid, fed by replayed `/scan` + `/odom`; run stock NDT on the same bag in parallel as ground truth], [PF pose vs NDT pose error bounded (quantitative, not eyeball)],
  [4], [*Localization bridge* — pose relay → `pose_estimator/pose_with_covariance`; enable `ekf_localizer` + `gyro_odometer`], [`/localization/kinematic_state` + `map→base_link` TF valid; still tracks NDT reference],
  [5], [*Perception wiring* — `laserscan_to_pointcloud` → `obstacle_segmentation/pointcloud`; standalone `LaserscanBasedOccupancyGridMapNode` from real `/scan` → `occupancy_grid_map/map`], [*Standalone check:* both topics at rate; live grid renders; planted obstacle in replay appears in cloud + grid],
  [6], [*Planning/control port* — forked `autosdv_planning/control_component` launches (config root → `autosdv_launch`); `2dlidar` preset (module policy); param overrides (costmap height gate, collision_detector pointcloud, velocity-limiter grid source)], [Planner boots with `empty_objects_publisher`; no module blocks readiness; trajectory published on replay],
  [7], [*Replay end-to-end* — route/goal on replayed run; obstacle-stop drill — *no actuation*], [Trajectory follows lanes; `obstacle_stop` inserts stop at planted blockage],
  [8], [*Vehicle bring-up* — 2D LiDAR driver + static TF `base_link→laser`; native `control_cmd` → vehicle interface; low-speed closed loop], [Live `/scan` matches replay behavior; car follows the lane; stops at obstacle],
)

Only phase 8 needs the vehicle and the physical 2D LiDAR. NDT-as-reference in
phases 3–4 turns localization sign-off into a measured comparison rather than
an RViz impression.

// ─────────────────────────────────────────────────────────────
= Open Risks

#table(
  columns: (1.35fr, 1fr),
  table.header([*Risk*], [*Mitigation*]),
  [*Covariance realism* — `ekf_localizer` trusts the relay's reported covariance; bad values destabilize the fusion.], [Tune PF covariance; validate `kinematic_state` against RViz ground truth before enabling planning.],
  [*2D perception gap* — planner is blind to obstacles outside the scan plane.], [Ship a 2D obstacle-stop node; keep speeds low; document the limitation.],
  [*Pose initialization* — PF needs a 2D initial pose; no NDT auto-init.], [Manual RViz 2D-pose-estimate, or GNSS via `pose_initializer` when available.],
  [*Odom quality* — PF's motion model leans on `/odom`; poor wheel odometry degrades MCL.], [Validate `/odom` drift over a straight run in Phase 3 before trusting MCL.],
  [*Planner fit* — Autoware defaults tuned for full-size vehicles.], [Set vehicle footprint, min turning radius, and velocity limits for the platform.],
)

// ─────────────────────────────────────────────────────────────
= Rejected Alternative — Full f1tenth Racing Stack

An alternative would replace the *entire* driving loop with f1tenth's racing
stack (raceline planner + pure-pursuit control), using Autoware only as a
sensor/vehicle shell and a `/drive`→`Control` adapter at the output.

*Rejected.* AutoSDV is a research platform, not a race car. Swapping in the
f1tenth planner/controller would discard exactly what gives AutoSDV its value —
Autoware's behavior/motion planning, safety framework, and the experiments
built on them — turning it into a roboracer-class vehicle. The goal here is a
*cheaper AutoSDV*, so the change is kept shallow: only sensing and localization
move to the low-cost 2D stack; planning and control remain Autoware.

#v(1em)
#line(length: 100%, stroke: 0.4pt + c-off)
#align(center, text(8pt, fill: c-off)[
  Generated as a design report · sources: `~/repos/AutoSDV`, f1tenth `particle_filter`, Autoware localization launch
])
