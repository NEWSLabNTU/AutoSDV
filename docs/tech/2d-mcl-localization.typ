#import "@preview/fletcher:0.5.8" as fletcher: diagram, node, edge

// ─────────────────────────────────────────────────────────────
// Palette (consistent with docs/design/f1tenth-2dlidar-integration.typ)
#let c-auto   = rgb("#1f6feb")   // Autoware — unchanged
#let c-auto-b = rgb("#dce7fb")
#let c-f1     = rgb("#e8590c")   // borrowed (Roboracer / nav2)
#let c-f1-b   = rgb("#fbe3d2")
#let c-new    = rgb("#c9184a")   // AutoSDV-authored
#let c-new-b  = rgb("#f7d6de")
#let c-off    = rgb("#6c757d")   // offline / map artefact
#let c-off-b  = rgb("#e4e6e9")
#let c-fix    = rgb("#2f9e44")   // fixed / verified
#let c-fix-b  = rgb("#d3f9d8")
#let c-drop   = rgb("#868e96")   // deleted under mcl
#let c-drop-b = rgb("#e9ecef")
#let c-mod    = rgb("#f08c00")   // modified
#let c-mod-b  = rgb("#fff3bf")

#let legend(color, body) = box(baseline: 0.15em)[#box(width: 0.85em, height: 0.85em, fill: color, radius: 1pt) #body]
#let ok = text(fill: c-fix, weight: "bold")[✓]
#let no = text(fill: c-new, weight: "bold")[✗]

#set document(title: "AutoSDV 2D-MCL Localization — Internal Technical Report")
#set page(
  paper: "a4",
  margin: (x: 2cm, y: 2cm),
  numbering: "1",
  footer: context [
    #set text(8pt, fill: c-off)
    AutoSDV · 2D-MCL localization · internal
    #h(1fr)
    #counter(page).display("1 / 1", both: true)
  ],
)
#set text(font: ("Liberation Sans", "DejaVu Sans"), size: 10pt)
#set par(justify: true, leading: 0.62em)
#show heading: set block(above: 1.3em, below: 0.7em)
#show heading.where(level: 1): set text(fill: c-auto, size: 15pt)
#show heading.where(level: 2): set text(fill: rgb("#0b3a7a"), size: 12pt)
#show heading.where(level: 3): set text(fill: rgb("#0b3a7a"), size: 10.5pt)
#set table(stroke: 0.4pt + c-off, inset: 6pt)
#show raw.where(block: true): set block(fill: rgb("#f6f8fa"), inset: 8pt, radius: 3pt, width: 100%)
#show figure.caption: set text(8.5pt, fill: c-off)

#align(center)[
  #text(19pt, weight: "bold", fill: c-auto)[AutoSDV 2D-MCL Localization]
  #v(-0.35em)
  #text(11.5pt, fill: c-off)[Integration, method, and measured results]
  #v(0.3em)
  #text(9pt, fill: c-off)[NEWSLabNTU AutoSDV · internal technical report · 2026-07-27]
]
#v(0.4em)
#line(length: 100%, stroke: 0.6pt + c-off)

= Summary

AutoSDV can now localize from a single-plane 2-D LiDAR scan against a 2-D
occupancy grid, selected with `pose_source:=mcl`, feeding Autoware's
`ekf_localizer` through the same contract topic that NDT uses. The estimator is
a Monte-Carlo localization filter descended from Roboracer (F1TENTH)
`particle_filter`, carrying six AutoSDV revisions — three of which fix defects
in the inherited measurement model that made the upstream filter unusable on
this data.

Measured on the official Autoware sample site, against NDT as ground truth,
five seeds:

#table(
  columns: (auto, auto, auto, auto, auto),
  align: (left, right, right, right, center),
  table.header([*Metric*], [*Median*], [*Range*], [*Threshold*], [*Verdict*]),
  [p95 translational], [2.198 m], [2.10–2.27], [< 2.5 m], [#ok 5/5],
  [Mean \|yaw\| error], [0.054 rad], [0.032–0.058], [< 0.2 rad], [#ok 5/5],
  [Mean translational], [2.255 m], [0.824–2.659], [< 1.0 m], [#no 1/5],
)

*The accuracy gate is not met on the mean.* Section 6.3 shows why: the failing
metric is dominated by a single sub-second startup transient, not by
steady-state error — 95% of poses are inside 2.2 m and heading is within
0.054 rad on every seed. The cause is identified and its fix is known.

#align(center, box(inset: 7pt, radius: 4pt, fill: c-off-b, width: 100%)[
  #set align(left)
  #set text(9pt)
  *Legend* #h(0.8em)
  #legend(c-auto-b)[Autoware, unchanged] #h(0.8em)
  #legend(c-f1-b)[Borrowed: Roboracer / nav2] #h(0.8em)
  #legend(c-new-b)[AutoSDV-authored] #h(0.8em)
  #legend(c-off-b)[Offline map artefact]
])

= Node diagram after integration

Everything to the right of `ekf_localizer` is stock AutoSDV: planning, control
and the vehicle interface see the same `/localization/kinematic_state` they
always have. The 2-D front end replaces only the pose estimator.

#v(0.3em)
#align(center, scale(72%, reflow: true)[
#diagram(
  spacing: (13mm, 9mm),
  node-stroke: 0.6pt,
  node-corner-radius: 3pt,
  node-inset: 4.5pt,
  {
    let t(b) = text(6pt, raw(b))
    let n(b) = text(7.5pt, b)
    let d(b) = text(6pt, fill: rgb("#40484f"), b)

    // sensing
    node((0, 0), n[3-D LiDAR\ #d[PointCloud2]], fill: c-auto-b, name: <lidar>)
    node((0, 1.6), n[IMU + wheel\ velocity], fill: c-auto-b, name: <imu>)
    node((0, 2.6), n[GNSS\ #d[NavSatFix]], fill: c-auto-b, name: <gnss>)

    // scan synthesis
    node((1.25, -0.5), n[pointcloud_to\_laserscan\ #d[z-band @ sensor height,\ target_frame base_link]], fill: c-auto-b, name: <p2l>)
    node((1.25, 0.5), n[scan_qos_bridge\ #d[BEST_EFFORT→RELIABLE]], fill: c-new-b, name: <bridge>)
    node((1.25, 1.6), n[wheel_imu_odom\ #d[planar odometry]], fill: c-new-b, name: <odom>)

    // map
    node((1.25, -1.6), n[nav2_map_server\ #d[occupancy grid]], fill: c-f1-b, name: <mapsrv>)
    node((0, -1.6), n[map.pgm + .yaml], fill: c-off-b, name: <grid>)

    // filter
    node((2.7, 0), n[*particle_filter*\ #d[2-D MCL, CDDT raycast\ 6 AutoSDV revisions]], fill: c-f1-b, name: <pf>)
    node((2.7, 1.6), n[gyro_odometer], fill: c-auto-b, name: <gyro>)

    // init
    node((1.25, 2.6), n[gnss_poser +\ pose_initializer], fill: c-auto-b, name: <init>)

    // relay + fusion
    node((4.0, 0), n[*mcl_pose_relay*\ #d[6×6 covariance,\ scan stamp, base_link]], fill: c-new-b, name: <relay>)
    node((5.2, 0.8), n[ekf_localizer], fill: c-auto-b, name: <ekf>)

    // downstream
    node((6.4, -0.4), n[Lanelet2 map], fill: c-off-b, name: <ll>)
    node((6.4, 0.8), n[planning], fill: c-auto-b, name: <plan>)
    node((7.5, 0.8), n[control], fill: c-auto-b, name: <ctrl>)
    node((8.5, 0.8), n[vehicle\ iface], fill: c-auto-b, name: <vif>)

    edge(<lidar>, <p2l>, "->", label: t("points"), label-size: 6pt)
    edge(<p2l>, <bridge>, "->", label: t("/scan_raw"), label-size: 6pt)
    edge(<bridge>, <pf>, "->", label: t("/scan"), label-size: 6pt)
    edge(<grid>, <mapsrv>, "->")
    edge(<mapsrv>, <pf>, "->", label: t("GetMap"), label-size: 6pt)
    edge(<imu>, <odom>, "->")
    edge(<odom>, <pf>, "->", label: t("/odom"), label-size: 6pt)
    edge(<imu>, <gyro>, "->", bend: -14deg)
    edge(<gnss>, <init>, "->")
    edge(<init>, <pf>, "->", label: t("/initialpose"), label-size: 6pt)
    edge(<pf>, <relay>, "->", label: t("/pf/pose/odom"), label-size: 6pt)
    edge(<relay>, <ekf>, "->", label: t("pose_estimator/\npose_with_covariance"), label-size: 5.5pt)
    edge(<gyro>, <ekf>, "->", label: t("twist"), label-size: 6pt)
    edge(<ekf>, <plan>, "->", label: t("kinematic_state"), label-size: 6pt)
    edge(<ll>, <plan>, "->")
    edge(<plan>, <ctrl>, "->")
    edge(<ctrl>, <vif>, "->")
  }
)
])

== What the diagram implies about integration style

Autoware exposes a *pose-estimator plugin slot*
(`pose_twist_estimator.launch.xml`) that a source can fill by shipping
`launch/pose_estimator.launch.xml` and serving `ndt_align_srv` and
`trigger_node`. `cuda_ndt_matcher` uses that slot. 2-D MCL deliberately does
*not*: it follows the precedent set by the Isaac visual localizer and publishes
into the contract topic from outside.

The reason is structural rather than expedient. The plugin contract requires the
estimator to be a client of `/map/get_differential_pointcloud_map` — Autoware's
PCD map service. A 2-D MCL estimator consumes an `OccupancyGrid` served by
`nav2_map_server` instead, so that half of the contract can never be satisfied.
Being outside the slot also means the estimator is invisible to
`pose_initializer`'s align step and to the pose-estimator arbiter; both
consequences are accepted and documented.

= Theory

== The estimation problem

The state is a planar pose $x = (x, y, theta)$. Given scans $z$, odometry $u$
and a map $m$, the belief factors into the standard two-step recursion:

$ overline("bel")(x_t) = integral p(x_t | x_(t-1), u_t) "bel"(x_(t-1)) d x_(t-1) $
$ "bel"(x_t) = eta dot p(z_t | x_t, m) dot overline("bel")(x_t) $

Along a corridor many poses explain one scan equally well, so the posterior is
genuinely multi-modal and a Gaussian cannot represent it. MCL therefore carries
$N = 4000$ weighted samples and evaluates the prediction integral by importance
sampling: draw from the previous belief, push each sample through the motion
model, weight by the measurement likelihood, resample.

== Motion model

A unicycle step per update, with the odometry increment rotated into each
particle's own heading and additive Gaussian noise:

$ x' = x + v cos(theta) Delta t, quad y' = y + v sin(theta) Delta t, quad theta' = theta + omega Delta t $

Dispersion is fixed per update (0.05 m, 0.025 m, 0.1 rad), not scaled by
distance travelled — an inherited simplification, called out in §5.

== Measurement model, and why it is the crux

For a candidate pose, each of 82 decimated beams is raycast in the grid to a
predicted range $d_k$ and compared with the observed $r_k$. Beams are assumed
conditionally independent:

$ p(z | x, m) = product_(k=1)^(82) p(r_k | d_k) $

Each beam likelihood is a four-component mixture (Thrun et al.,
_Probabilistic Robotics_, ch. 6): a Gaussian hit term, a short-reading term for
unmapped obstacles, a max-range spike for no-return, and a uniform noise floor.
The textbook requires *each component to be a normalised density* before mixing.
The inherited implementation violated that for the short term, and §5.1 shows
what it cost.

Raycasting uses CDDT (Compressed Directional Distance Transform) over 112
discretized headings, which makes ~330 000 raycasts per update affordable on
CPU.

= Map handling

`map_path` keeps one meaning — a map directory — whose contents vary by method.
`pose_source` selects which geometry map is loaded:

#table(
  columns: (auto, auto, auto, auto),
  align: (left, center, center, center),
  table.header([*`pose_source`*], [*PCD loader*], [*Grid server*], [*Lanelet2 + projector*]),
  [`cuda_ndt`, `ndt`], [on], [off], [on],
  [`mcl`], [*off*], [*on*], [on],
  [`isaac`, `visual`], [on], [off], [on],
)

Two facts govern this. First, stock Autoware's `pointcloud_map_loader` is
unconditional and *fatal* — a missing PCD throws and kills `map_container` —
which is why AutoSDV owns its map component. Second, the projection file is
still required for `mcl`: `lanelet2_map_loader` publishes `/map/vector_map` only
inside its `/map/map_projector_info` callback, so without it planning never
starts, even though the grid itself never reads it.

The grid's `origin` is expressed in *`map`-frame metres*, and the `map` frame is
defined by the projector. A grid sliced from the site's PCD is therefore
frame-correct by construction; a grid built by SLAM is not, and must either be
georeferenced or declare `projector_type: Local`. A silent mismatch here
produces tens of metres of error with no error message, so `just map-check`
compares the grid's coverage against the Lanelet2 bounding box and refuses to
pass when it cannot verify the projection honestly.

= AutoSDV revisions to the MCL method

Six revisions separate this filter from upstream Roboracer `particle_filter`.
Three are measurement-model corrections without which the filter does not work
on this data; three are interface corrections required by Autoware.

== 5.1 Normalised short-reading component

*Upstream (Roboracer):* the short-reading term is a linear ramp
$2 z_"short" (d - r) \/ d$, which integrates to $z_"short" dot d$ — mass that
*grows with predicted range in pixels*. Because each table column is normalised
afterwards, the configured weights no longer mean what they say.

*Measured consequence:* with `z_hit` configured at 0.75, the hit term's actual
share of the probability mass collapses with range.

#figure(
  image("assets/sensor-model-mixture-mass.png", width: 92%),
  caption: [Effective mixture weights versus predicted range. Upstream, the configured 75% hit weight decays to 25.4% / 15.2% / 6.8% at 10 / 20 / 50 m; normalised, it stays flat.],
)

*AutoSDV revision:* replace the ramp with a properly normalised exponential,
$p_"short" (r|d) = eta lambda_"short" e^(-lambda_"short" r)$ with
$eta = 1 \/ (1 - e^(-lambda_"short" d))$, selected by
`sensor_model_variant: normalized_short`. The `upstream` variant is retained and
reproduces the original table bit-for-bit, which is what keeps earlier results
reproducible.

*Effect, measured offline over five frozen scans* — the model's own global
maximum versus the true pose:

#table(
  columns: (auto, auto, auto, auto),
  align: (left, right, right, right),
  table.header([*Configuration*], [*Gap (nats)*], [*Argmax distance*], [*GT percentile*]),
  [upstream], [51.92], [29.67 m], [0.83],
  [+ normalised short], [7.07], [0.20 m], [99.998],
  [+ skip non-finite (§5.2)], [6.24], [0.22 m], [99.9989],
)

The upstream model's most likely pose sat ~30 m from the truth, and the true
pose ranked below 99% of candidates. This is not a tuning deficiency; the
likelihood function was mis-specified.

#figure(
  image("assets/likelihood-field.png", width: 78%),
  caption: [Log-likelihood field over a 601×601 pose grid for one frozen scan, upstream model. The true pose (green) sits in a dark trough while the maximum (blue) lies ~29 m away — and this holds even while the filter is tracking to 0.4 m. Underflow was excluded: 0 of 361 201 poses underflow.],
)

== 5.2 Non-finite beams no longer outscore matches

*Upstream:* `inf` observations are clamped into the max-range bucket. Measured
on our data, a no-return beam then scored *1.87× a perfectly matching beam*,
and ~30% of beams are non-finite by construction of the synthesized scan.

*AutoSDV revision:* `skip_nonfinite_beams` drops non-finite beams and their
angles from the evaluation, with `min_finite_beams` (default 10) suppressing the
correction entirely when too few beams survive.

== 5.3 One correction per scan

*Upstream:* `update()` is invoked from the odometry callback. With odometry at
20 Hz and scans at 10 Hz, each scan is multiplied into roughly two consecutive
Bayes updates, squaring its likelihood contribution — textbook overconfidence,
and plausibly why an ad-hoc `squash_factor` exists at all.

*AutoSDV revision:* `update_on_new_scan_only` gates the correction on the scan
stamp and accumulates odometry between corrections by *exact* rotation
composition (unit-tested to 1e-9), not a small-angle approximation. Verified:
237 corrections against 237 unique scan stamps — zero double counting.

*Known consequence:* `publish_tf` and `visualize` live inside `update()`, so the
pose and TF publish rate halves to scan rate. Downstream consumers see ~10 Hz.

== 5.4 Initialization gating

*Upstream:* `initialize_global()` runs in the constructor, spreading particles
over the whole map, and the filter publishes the centroid of that cloud
immediately. Measured: the first published pose sat 11.47 m from truth and the
track began ~55 m away.

*AutoSDV revision:* `require_initialpose` skips global initialization and
suppresses updates and publication until a seed arrives — matching Autoware's
`UNINITIALIZED → INITIALIZED` semantics.

#table(
  columns: (auto, auto, auto),
  align: (left, right, right),
  table.header([*Quantity*], [*Ungated*], [*Gated*]),
  [First published pose, error], [11.47 m], [*0.85 m*],
  [Poses > 5 m from truth], [7], [*0*],
  [Worst pose in the run], [14.59 m], [*0.85 m*],
)

== 5.5 Covariance-aware seeding

*Upstream:* `initialize_particles_pose` receives a bare `Pose`, so the seed's
36-element covariance is unreachable, and particles are drawn from a hardcoded
isotropic 0.5 m / 0.4 rad Gaussian.

*Comparison with nav2 AMCL:* AMCL consumes only the covariance *diagonal*.

*AutoSDV revision:* the covariance reaches the sampler, which extracts the
planar $(x, y, theta)$ marginal — row-major indices 0, 1, 5 / 6, 7, 11 /
30, 31, 35 — symmetrises it, and draws from a multivariate normal so anisotropy
and correlation survive. It falls back to scalar sampling when the covariance is
absent, zero, or not positive semi-definite. This is slightly ahead of the nav2
reference rather than merely level with it.

== 5.6 Interface adaptation: `mcl_pose_relay`

#pagebreak(weak: true)
An AutoSDV-authored node republishes the filter's pose as
`PoseWithCovarianceStamped` on the contract topic, fixing four inherited
interface defects rather than propagating them:

#table(
  columns: (auto, 1fr),
  table.header([*Inherited defect*], [*Correction*]),
  [3×3 covariance written into `covariance[0:9]` of a row-major 6×6, so $sigma_(y y)$ lands in the $x$–$z$ slot], [planar terms placed at the correct indices; unused z/roll/pitch diagonals set to 10⁶ to mark them unobserved],
  [Pose stamped `now()` while the TF uses the scan stamp], [stamped with measurement time; a `stamp_source` parameter selects the recovery strategy],
  [Frame ids carry leading slashes (`/map`), which tf2 rejects], [normalised to `map`],
  [Pose expressed for the laser frame], [composed to `base_link` via tf2],
)

Publication is also not gated on subscriber count, unlike upstream, so a
late-joining `ekf_localizer` still receives poses.

= Changes to the Autoware stack

This section is the change record: what the localization stack looks like with
stock NDT, what it looks like under `pose_source:=mcl`, and every node that was
deleted, added or modified to get there. Nothing here alters the `cuda_ndt` or
`ndt` paths — those were verified byte-identical by topic and node list diff.

#align(center, box(inset: 6pt, radius: 4pt, fill: c-off-b, width: 100%)[
  #set align(left)
  #set text(9pt)
  *Change marks* #h(0.8em)
  #legend(c-auto-b)[unchanged] #h(0.8em)
  #legend(c-drop-b)[deleted for `mcl`] #h(0.8em)
  #legend(c-new-b)[added] #h(0.8em)
  #legend(c-mod-b)[modified]
])

== Before — stock NDT path

#v(0.2em)
#align(center, scale(76%, reflow: true)[
#diagram(
  spacing: (14mm, 9mm),
  node-stroke: 0.6pt,
  node-corner-radius: 3pt,
  node-inset: 4.5pt,
  {
    let n(b) = text(7.5pt, b)
    let d(b) = text(6pt, fill: rgb("#40484f"), b)
    let t(b) = text(6pt, raw(b))

    node((0, -1.4), n[pointcloud_map\_loader], fill: c-auto-b, name: <pcd>)
    node((0, -0.4), n[lanelet2_map\_loader], fill: c-auto-b, name: <ll>)
    node((0, 0.5), n[map_projection\_loader], fill: c-auto-b, name: <proj>)

    node((1.7, -1.4), n[util.launch\ #d[voxel downsample]], fill: c-auto-b, name: <util>)
    node((1.7, -0.3), n[*ndt_scan_matcher*\ #d[or cuda_ndt plugin\ serves ndt_align_srv,\ trigger_node]], fill: c-auto-b, name: <ndt>)

    node((1.7, 1.3), n[map_height_fitter\ #d[target: pointcloud_map]], fill: c-auto-b, name: <fit>)
    node((0, 1.3), n[pose_initializer\ #d[ndt_enabled: true]], fill: c-auto-b, name: <init>)

    node((3.3, 0.4), n[ekf_localizer], fill: c-auto-b, name: <ekf>)
    node((1.7, 2.3), n[voxel_based_compare\_map_filter], fill: c-auto-b, name: <cmp>)

    edge(<pcd>, <util>, "->", label: t("pointcloud_map"), label-size: 5.5pt)
    edge(<util>, <ndt>, "->", label: t("downsample/pointcloud"), label-size: 5.5pt)
    edge(<pcd>, <ndt>, "->", label: t("get_differential_map"), label-size: 5.5pt, bend: -22deg)
    edge(<ndt>, <ekf>, "->", label: t("pose_with_covariance"), label-size: 5.5pt)
    edge(<init>, <ndt>, "->", label: t("ndt_align"), label-size: 5.5pt)
    edge(<init>, <fit>, "->")
    edge(<pcd>, <fit>, "->", label: t("height"), label-size: 5.5pt)
    edge(<init>, <ekf>, "->", label: t("initialpose3d"), label-size: 5.5pt, bend: 20deg)
    edge(<ll>, <ekf>, "->", label: t("vector_map"), label-size: 5.5pt)
    edge(<proj>, <ll>, "->", label: t("projector_info"), label-size: 5.5pt)
    edge(<pcd>, <cmp>, "->", label: t("map"), label-size: 5.5pt)
  }
)
])

== After — `pose_source:=mcl`

#v(0.2em)
#align(center, scale(76%, reflow: true)[
#diagram(
  spacing: (14mm, 9mm),
  node-stroke: 0.6pt,
  node-corner-radius: 3pt,
  node-inset: 4.5pt,
  {
    let n(b) = text(7.5pt, b)
    let d(b) = text(6pt, fill: rgb("#40484f"), b)
    let t(b) = text(6pt, raw(b))

    node((0, -1.4), n[pointcloud_map\_loader], fill: c-drop-b, stroke: (dash: "dashed", paint: c-drop), name: <pcd>)
    node((0, -0.4), n[lanelet2_map\_loader], fill: c-auto-b, name: <ll>)
    node((0, 0.5), n[map_projection\_loader], fill: c-auto-b, name: <proj>)
    node((0, -2.4), n[*nav2_map_server*\ #d[occupancy grid,\ lifecycle-activated]], fill: c-new-b, name: <grid>)

    node((1.7, -1.4), n[util.launch\ #d[voxel downsample]], fill: c-drop-b, stroke: (dash: "dashed", paint: c-drop), name: <util>)
    node((1.7, -0.4), n[ndt_scan_matcher], fill: c-drop-b, stroke: (dash: "dashed", paint: c-drop), name: <ndt>)
    node((1.7, -2.4), n[*particle_filter*\ #d[with scan chain,\ QoS bridge, odometry]], fill: c-new-b, name: <pf>)
    node((3.2, -2.4), n[*mcl_pose_relay*], fill: c-new-b, name: <relay>)

    node((1.7, 1.3), n[map_height_fitter\ #d[target: *vector_map*]], fill: c-mod-b, name: <fit>)
    node((0, 1.3), n[pose_initializer\ #d[ndt_enabled: *false*]], fill: c-mod-b, name: <init>)

    node((3.3, 0.4), n[ekf_localizer\ #d[ungated — see §6.3]], fill: c-auto-b, name: <ekf>)
    node((1.7, 2.3), n[voxel_based_compare\_map_filter], fill: c-drop-b, stroke: (dash: "dashed", paint: c-drop), name: <cmp>)

    edge(<grid>, <pf>, "->", label: t("GetMap"), label-size: 5.5pt)
    edge(<pf>, <relay>, "->", label: t("/pf/pose/odom"), label-size: 5.5pt)
    edge(<relay>, <ekf>, "->", label: t("pose_with_covariance"), label-size: 5.5pt)
    edge(<init>, <pf>, "->", label: t("initialpose"), label-size: 5.5pt)
    edge(<init>, <fit>, "->")
    edge(<ll>, <fit>, "->", label: t("height"), label-size: 5.5pt)
    edge(<init>, <ekf>, "->", label: t("initialpose3d"), label-size: 5.5pt, bend: 20deg)
    edge(<ll>, <ekf>, "->", label: t("vector_map"), label-size: 5.5pt)
    edge(<proj>, <ll>, "->", label: t("projector_info"), label-size: 5.5pt)
  }
)
])

Dashed grey nodes are *not launched* under `mcl` — they are drawn to make the
deletion explicit rather than leaving the reader to spot an absence.

== Change table

#table(
  columns: (auto, auto, 1fr),
  align: (left, center, left),
  table.header([*Node / artefact*], [*Change*], [*Detail*]),

  table.cell(colspan: 3, fill: c-off-b)[*Map layer*],
  [`pointcloud_map_loader`], [deleted], [Not composed under `mcl`. Stock Autoware runs it unconditionally with no disable argument, and a missing PCD *throws* and kills `map_container` — hence an AutoSDV-owned map component.],
  [`nav2_map_server`], [added], [Serves the occupancy grid over `GetMap`. Node name must stay `map_server` (the filter hardcodes `/map_server/map`), and it is a lifecycle node, so bring-up configures then activates with retries.],
  [`autosdv_map_component`\ `.launch.xml`], [added], [Replaces stock `tier4_map_component`. Splits `map_container` into `with_pcd` / `no_pcd` variants selected by sibling `<group if>` — because `play_launch` ignores `if=` on `<composable_node>` (filed upstream, fixed in `play_launch_parser`).],
  [`lanelet2_map_loader`,\ `map_projection_loader`], [unchanged], [Both still required: the vector map is published only inside the projector-info callback, so `mcl` still needs `map_projector_info.yaml` even though the grid never reads it.],
  [`config/map/*.param.yaml`], [modified], [Previously dead — the stock include passed no arguments, so `/opt` configs loaded. Now actually wired.],

  table.cell(colspan: 3, fill: c-off-b)[*Pose estimator*],
  [`ndt_scan_matcher`], [deleted], [`'mcl'` is disjoint from `'ndt'` in `available_args`, so `use_ndt_pose` stays false and neither the built-in matcher nor the plugin slot is instantiated.],
  [`util.launch.xml`], [deleted], [The voxel-downsample chain is gated on `use_ndt_pose`; MCL consumes a `LaserScan`, not a downsampled cloud.],
  [`particle_filter`, scan chain,\ `wheel_imu_odom`], [added], [Via `autosdv_mcl_launch`, included from a new `use_mcl_pose` branch.],
  [`mcl_pose_relay`], [added], [Publishes the contract topic from outside the plugin slot, Isaac-style.],
  [`pose_twist_estimator`\ `.launch.xml`], [modified], [Fork edit: `'mcl'` added to `available_args`, plus a `use_mcl_pose` group. The pre-existing `pose_source_package` plugin slot is untouched.],

  table.cell(colspan: 3, fill: c-off-b)[*Pose initialization*],
  [`pose_initializer`], [modified], [`ndt_enabled` follows `use_ndt_pose`, so it is false for `mcl` — which is what stops the initializer blocking on an `ndt_align_srv` that MCL does not serve.],
  [`map_height_fitter`], [modified], [Target retargeted from `pointcloud_map` to `vector_map`. Measured before the change: its constructor blocked forever on an absent PCD, so `/localization/initialize` hung for its full timeout and `/initialpose` never even appeared.],
  [`gnss_poser`,\ `automatic_pose_initializer`], [unchanged], [GNSS auto-init works as it does for NDT. Heading remains the weak point — see §7.],

  table.cell(colspan: 3, fill: c-off-b)[*Perception and fusion*],
  [`voxel_based_compare`\ `_map_filter`], [deleted], [`use_pointcloud_map` is forced false for `mcl`. Left enabled it attempts to load and hangs mid-construction waiting for a map that never arrives.],
  [`gyro_odometer`], [unchanged], [Still the twist source.],
  [`ekf_localizer`], [unchanged], [Consumes the same contract topic. *Not* gated on initialization, which is the cause of the residual transient in §6.3 — the one identified item still outstanding.],
)

== Launch-layer changes outside the diagram

#table(
  columns: (auto, auto, 1fr),
  align: (left, center, left),
  table.header([*File*], [*Change*], [*Detail*]),
  [`autosdv.launch.yaml`,\ `logging_simulation.launch.yaml`], [modified], [`pose_source` default moved `ndt` → `cuda_ndt`; `pose_source_package` now defaults to the sentinel `auto`, resolved by a `let`. Previously `ndt` silently ran the CUDA plugin, and the documented workaround was impossible because `ros2 launch` rejects an empty argument value.],
  [`autosdv_autoware.launch.xml`], [modified], [Includes the new map component; forwards `use_pointcloud_map` and `occupancy_grid_file`; retargets the height fitter.],
  [`just map-check`], [added], [Validates a map directory against a `pose_source`, including a grid-versus-lanelet2 frame-extent comparison. It reports *cannot verify* rather than passing when the projection cannot be reproduced honestly.],
)

= Results

All figures and numbers come from the official Autoware sample site
(`sample-rosbag`), replayed against `occupancy_grid_scanaccum_mh1r05.yaml`
(0.05 m/cell, 363 295 occupied cells, no unknown cells), with NDT's
`/localization/kinematic_state` as ground truth. The MCL trace plotted is also
`/localization/kinematic_state` — i.e. the fused output after
`ekf_localizer`, not the raw filter pose — so these are end-to-end results.

== NDT path

#figure(
  image("assets/mcl-integrated-ndt.png", width: 58%),
  caption: [NDT (`pose_source:=cuda_ndt`) ground-truth trajectory, 1397 poses over the 58 s run. Markers every 5 s on a clock shared with the following two figures.],
)

== MCL path

#figure(
  image("assets/mcl-integrated-mcl.png", width: 58%),
  caption: [AutoSDV 2D-MCL (`pose_source:=mcl`, seed 61) as fused `kinematic_state`. Same extent, same shared clock.],
)

== Comparison

#figure(
  image("assets/mcl-integrated-overlay.png", width: 58%),
  caption: [Both paths on one axis. The 25 s–55 s markers pair between the two tracks, confirming agreement in time as well as in space. This seed's mean translational error is 0.824 m, p95 2.101 m, mean |yaw| 0.032 rad — the one seed of five meeting all three thresholds.],
)

== Numbers

Five seeds, fused `kinematic_state` versus NDT ground truth, aligned by nearest
timestamp within 100 ms:

#table(
  columns: (auto, auto, auto, auto, auto),
  align: (center, right, right, right, center),
  table.header([*Seed*], [*Mean (m)*], [*p95 (m)*], [*Mean \|yaw\| (rad)*], [*All three*]),
  [61], [0.824], [2.101], [0.032], [#ok],
  [62], [2.086], [2.265], [0.049], [#no],
  [63], [2.659], [2.198], [0.058], [#no],
  [64], [2.547], [2.189], [0.055], [#no],
  [65], [2.255], [2.223], [0.054], [#no],
  [*median*], [*2.255*], [*2.198*], [*0.054*], [1/5],
)

Reporting median and range rather than a single run is deliberate: an earlier
phase found three near-identical configurations spanning 17–27 m of mean error,
so single-run comparisons cannot support a claim about this filter.

=== 6.3 Why the mean fails while p95 and yaw pass

The distributions are not merely shifted — the failing metric is driven by a
handful of extreme outliers:

#table(
  columns: (auto, auto, auto, auto, auto),
  align: (center, right, right, right, left),
  table.header([*Seed*], [*Mean*], [*Max*], [*Poses > 20 m*], [*When*]),
  [61 (passes)], [0.82 m], [3.21 m], [*0* of 2239], [—],
  [62], [2.09 m], [116.36 m], [24 of 2238], [t+5.7 s to t+6.3 s],
)

Seeds 62–65 each contain one sub-second excursion reaching ~116 m. That is why
p95 sits at 2.20 m on every seed while the mean fails: 95% of poses are good,
and two dozen enormous outliers inside a 0.6 s window drag the average.

The cause is §5.4's lesson recurring one layer up. `require_initialpose` gates
*the filter* from publishing before it is seeded, but nothing gates
*`ekf_localizer`*, which holds and republishes the seed pose while the vehicle
drives away, until the filter's first real pose arrives. Autoware's native
mechanism for this is `ekf_trigger_node`, which `pose_initializer` uses to hold
the EKF until initialization completes; wiring it for `mcl` is the remaining
item between this work and the gate.

= What is not established

- *One site, one bag, one map.* Everything above is the Autoware sample site. No
  cross-validation on COSS or on live hardware.
- *Heading initialization is fragile by luck, not design.* `gnss_poser` derives
  heading from course-over-ground, meaningless at a standstill; the measured
  seed yaw error was 1.81 rad (104°) with a placeholder covariance of exactly
  1.0 rad². Recovery currently depends on a wide particle spread plus scan
  matching. The principled fix is an align step — Autoware calls `ndt_align_srv`
  for exactly this — and the offline field search already built for §5.1 would
  serve.
- *Two beam-model defects remain open.* Beams are treated as independent when
  corridor beams are strongly correlated, and returns beyond `max_range` are
  clamped into a pose-independent bucket, discarding the long down-corridor
  beams that would fix longitudinal position. nav2 AMCL was measured on the same
  data and also failed the gate, which suggests these are limits of beam-model
  MCL at this speed and site rather than defects of this port.
- *A residual 0.05–0.22 m offline offset* between the model's argmax and the true
  pose is unexplained.

= Reproducing

```bash
# validate a map directory for the method
just map-check data/sample-rosbag-replay/sample-map-rosbag mcl

# build a grid from an existing PCD map
just map-grid-from-pcd <map_dir> --z-min <ground+0.2> --z-max <ground+0.5>

# replay with 2D-MCL
just launch-sim-logging ARGS="pose_source:=mcl \
    occupancy_grid_file:=occupancy_grid_scanaccum_mh1r05.yaml"

# score a run against NDT ground truth
python3 scripts/2dlidar/compare_poses.py <gt_bag> <mcl_bag> \
    --no-motion-window --gt-time-source bag --out report.md

# regenerate this document's three figures
python3 scripts/2dlidar/plot_trajectories.py \
    --mcl-bag data/rosbags/phase5/mcl_e2e_s61 \
    --mcl-topic /localization/kinematic_state --mcl-type Odometry \
    --out-dir docs/tech/assets --prefix mcl-integrated
```

= References

#table(
  columns: (auto, 1fr),
  table.header([*Topic*], [*Document*]),
  [Algorithm, data formats, the five defects], [`docs/research/localization/2d_mcl_algorithm.md`],
  [Initialization and covariance], [`docs/research/localization/mcl_initialization_and_covariance.md`],
  [`pose_source` dispatch and the plugin contract], [`docs/design/localization-method-switching.md`],
  [Map handling per method], [`docs/design/map-handling-per-localization-method.md`],
  [Measured phase reports], [`docs/reports/2dlidar-phase3*.md` … `2dlidar-phase5-*.md`],
  [Upstream filter], [Roboracer/F1TENTH `particle_filter`; fork `NEWSLabNTU/particle_filter@autosdv`],
  [Raycaster], [`kctess5/range_libc`; fork `NEWSLabNTU/range_libc@autosdv`],
)
