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
// Compact legend rendered directly beneath a diagram, so each diagram is
// self-contained rather than depending on a key elsewhere in the document.
#let diagram-key(..entries) = align(center, box(inset: 5pt, radius: 3pt,
  fill: rgb("#fbfbfc"), stroke: 0.4pt + c-off)[
  #set text(7.5pt)
  #entries.pos().join(h(0.9em))
])
#let dashed-swatch(body) = box(baseline: 0.15em)[#box(width: 0.85em, height: 0.85em,
  fill: c-drop-b, radius: 1pt, stroke: (dash: "dashed", paint: c-drop, thickness: 0.5pt)) #body]

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
five seeds. Three terms recur in every results table below:

/ seed: the RNG seed given to the particle filter (`random_seed`). A particle
  filter is stochastic — particle placement and resampling draws differ run to
  run — so one run proves nothing. Each reported figure is a replay of the same
  bag against the same map, differing only in this seed; the spread across seeds
  is the run-to-run variability of the method.
/ mean: the arithmetic mean, over every published pose in a run, of the
  planar distance between the MCL pose and the NDT pose at the same timestamp.
  It is sensitive to outliers: one large excursion moves it far more than it
  moves the median, which is what made the harness defect described under
  #emph[Why an earlier matrix reported a failing mean] visible in the mean
  alone.
/ p95: the 95th percentile of that same per-pose distance — the value 95% of
  poses stay under. It reports typical behaviour with the worst 5% excluded, so
  a low p95 beside a high mean means "usually accurate, occasionally very wrong"
  rather than "uniformly mediocre".

Medians and ranges in the tables are taken across the five seeds, not across
poses.

#table(
  columns: (auto, auto, auto, auto, auto),
  align: (left, right, right, right, center),
  table.header([*Metric*], [*Median*], [*Range*], [*Threshold*], [*Verdict*]),
  [Mean translational], [0.849 m], [0.830–0.872], [< 1.0 m], [#ok 5/5],
  [p95 translational], [2.173 m], [2.11–2.24], [< 2.5 m], [#ok 5/5],
  [Mean \|yaw\| error], [0.034 rad], [0.032–0.034], [< 0.2 rad], [#ok 5/5],
)

*All three thresholds are met on all five seeds.* An earlier matrix reported
the mean failing at 2.26 m; that number was an artefact of its own harness,
which reused one launched stack across five replays and never applied the seed
it labelled each run with. #emph[Numbers], under Results, gives the corrected
figures, and #emph[Why an earlier matrix reported a failing mean] the
diagnosis.

= Target architecture

The production vehicle carries a *2-D LiDAR* whose driver publishes a
`sensor_msgs/LaserScan` directly, and a *2-D occupancy grid* map. Everything to
the right of `ekf_localizer` is stock AutoSDV: planning, control and the vehicle
interface consume the same `/localization/kinematic_state` they always have. The
2-D front end replaces only the pose estimator.

#v(0.3em)
#align(center, scale(74%, reflow: true)[
#diagram(
  spacing: (14mm, 9mm),
  node-stroke: 0.6pt,
  node-corner-radius: 3pt,
  node-inset: 4.5pt,
  {
    let t(b) = text(6pt, raw(b))
    let n(b) = text(7.5pt, b)
    let d(b) = text(6pt, fill: rgb("#40484f"), b)

    // map
    node((0, -1.5), n[grid.pgm + .yaml], fill: c-off-b, name: <grid>)
    node((1.35, -1.5), n[nav2_map_server], fill: c-f1-b, name: <mapsrv>)
    node((0, -0.55), n[lanelet2_map.osm\ + projector info], fill: c-off-b, name: <llfile>)
    node((1.35, -0.55), n[lanelet2 + projection\ loaders], fill: c-auto-b, name: <ll>)

    // sensing
    node((0, 0.5), n[*2-D LiDAR*\ #d[LaserScan]], fill: c-auto-b, name: <lidar>)
    node((1.35, 0.5), n[scan_qos_bridge\ #d[to RELIABLE]], fill: c-new-b, name: <bridge>)
    node((0, 1.5), n[IMU + wheel\ velocity], fill: c-auto-b, name: <imu>)
    node((1.35, 1.5), n[wheel_imu_odom], fill: c-new-b, name: <odom>)
    node((0, 2.4), n[GNSS], fill: c-auto-b, name: <gnss>)
    node((1.35, 2.4), n[gnss_poser +\ pose_initializer], fill: c-auto-b, name: <init>)

    // estimator
    node((2.9, 0.5), n[*particle_filter*\ #d[2-D MCL, CDDT\ 6 revisions, §5]], fill: c-f1-b, name: <pf>)
    node((2.9, 1.5), n[gyro_odometer], fill: c-auto-b, name: <gyro>)
    node((4.2, 0.5), n[*mcl_pose_relay*\ #d[6×6 cov, scan stamp,\ base_link]], fill: c-new-b, name: <relay>)

    // fusion + downstream
    node((5.5, 1.0), n[ekf_localizer], fill: c-auto-b, name: <ekf>)
    node((6.7, 1.0), n[planning], fill: c-auto-b, name: <plan>)
    node((7.7, 1.0), n[control], fill: c-auto-b, name: <ctrl>)
    node((8.7, 1.0), n[vehicle\ iface], fill: c-auto-b, name: <vif>)

    edge(<grid>, <mapsrv>, "->")
    edge(<mapsrv>, <pf>, "->", label: t("GetMap"), label-size: 5.5pt)
    edge(<llfile>, <ll>, "->")
    edge(<ll>, <ekf>, "->", label: t("vector_map"), label-size: 5.5pt)
    edge(<lidar>, <bridge>, "->", label: t("/scan_raw"), label-size: 5.5pt)
    edge(<bridge>, <pf>, "->", label: t("/scan"), label-size: 5.5pt)
    edge(<imu>, <odom>, "->")
    edge(<odom>, <pf>, "->", label: t("/odom"), label-size: 5.5pt)
    edge(<imu>, <gyro>, "->", bend: -12deg)
    edge(<gnss>, <init>, "->")
    edge(<init>, <pf>, "->", label: t("/initialpose"), label-size: 5.5pt)
    edge(<pf>, <relay>, "->", label: t("/pf/pose/odom"), label-size: 5.5pt)
    edge(<relay>, <ekf>, "->", label: t("pose_estimator/\npose_with_covariance"), label-size: 5.5pt)
    edge(<gyro>, <ekf>, "->", label: t("twist"), label-size: 5.5pt)
    edge(<ekf>, <plan>, "->", label: t("kinematic_state"), label-size: 5.5pt)
    edge(<plan>, <ctrl>, "->")
    edge(<ctrl>, <vif>, "->")
  }
)
])
#diagram-key(
  legend(c-auto-b)[Autoware, unchanged],
  legend(c-f1-b)[borrowed: Roboracer / nav2],
  legend(c-new-b)[AutoSDV-authored],
  legend(c-off-b)[offline map artefact],
)

== Testing configuration versus production

Every measurement in this report was taken on the official Autoware
`sample-rosbag`, which was recorded with a *3-D* LiDAR and ships a *PCD* map.
Two substitutions therefore stand in for hardware we did not have on the bench.
Both are test-only scaffolding; neither exists on the target vehicle.

#table(
  columns: (auto, 1fr, 1fr),
  table.header([*Input*], [*Production*], [*Testing (this report)*]),
  [Scan],
    [2-D LiDAR driver publishes `LaserScan` on `/scan_raw` directly],
    [`pointcloud_to_laserscan` synthesises a scan from the 3-D cloud — a z-band slab at the 2-D sensor's mounting height, `target_frame: base_link`],
  [Grid map],
    [Authored once for the site: `slam_toolbox` survey, or a one-time slice of an existing PCD map],
    [`pcd_to_pgm.py` slices the bag's PCD map offline, or `scan_accumulate_grid.py` accumulates scans at ground-truth poses],
  [Ground truth],
    [none — this is the estimate],
    [NDT `kinematic_state` from a parallel run over the same bag],
)

#align(center, box(inset: 7pt, radius: 3pt, fill: c-off-b, width: 100%)[
  #set align(left)
  #set text(9pt)
  *Insert the test scaffold into the target diagram like this:* the
  `2-D LiDAR` node is replaced by `3-D LiDAR → pointcloud_to_laserscan`, feeding
  the same `/scan_raw`; and `grid.pgm` is produced offline by `pcd_to_pgm.py`
  from `pointcloud_map.pcd` instead of being authored for the site. Every node
  to the right of `/scan_raw` — the filter, the relay, the fusion — is identical
  in both configurations, which is what makes the bench results transferable.
])

The scan substitution is not free, and §7 records the consequence: a synthesized
plane inherits the 3-D sensor's mounting and any vehicle pitch, so ~30% of beams
come back non-finite, where a rigidly-mounted 2-D unit would return a denser,
more stable plane.

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
  [\+ normalised short], [7.07], [0.20 m], [99.998],
  [\+ skip non-finite (§5.2)], [6.24], [0.22 m], [99.9989],
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
  columns: (1.1fr, 1fr),
  table.header([*Inherited defect*], [*Correction*]),
  [3×3 covariance written into `covariance[0:9]` of a row-major 6×6, so $sigma_(y y)$ lands in the $x$–$z$ slot], [planar terms placed at the correct indices; unused z/roll/pitch diagonals set to 10⁶ to mark them unobserved],
  [Pose stamped `now()` while the TF uses the scan stamp], [stamped with measurement time; a `stamp_source` parameter selects the recovery strategy],
  [Frame ids carry leading slashes (`/map`), which tf2 rejects], [normalised to `map`],
  [Pose expressed for the laser frame], [composed to `base_link` via tf2],
)

Publication is also not gated on subscriber count, unlike upstream, so a
late-joining `ekf_localizer` still receives poses.

= Changes to the Autoware stack

This section is the change record: every node that `pose_source:=mcl` deletes,
adds or reconfigures relative to the stock NDT stack. Nothing here alters the
`cuda_ndt` or `ndt` paths — those were verified byte-identical by topic and node
list diff, so the two configurations coexist in one tree.

== The diff, in one diagram

Same left-to-right layout as the target architecture in §2, with both paths
overlaid: the NDT nodes that `pose_source:=mcl` no longer launches are drawn
dashed, the nodes it adds are red, and the two it reconfigures are amber. Read
the dashed nodes as *present under `cuda_ndt`/`ndt`, absent under `mcl`*.

#v(0.2em)
#align(center, scale(74%, reflow: true)[
#diagram(
  spacing: (14mm, 9mm),
  node-stroke: 0.6pt,
  node-corner-radius: 3pt,
  node-inset: 4.5pt,
  {
    let t(b) = text(6pt, raw(b))
    let n(b) = text(7.5pt, b)
    let d(b) = text(6pt, fill: rgb("#40484f"), b)
    let dd(b) = text(6pt, fill: c-drop, b)
    let gone = (dash: "dashed", paint: c-drop)

    // ── map layer: PCD out, PGM in ──
    node((0, -2.3), n[pointcloud_map.pcd\ #dd[and metadata]], fill: c-drop-b, stroke: gone, name: <pcdf>)
    node((1.35, -2.3), n[pointcloud_map\_loader], fill: c-drop-b, stroke: gone, name: <pcd>)
    node((0, -1.35), n[*grid.pgm + .yaml*], fill: c-off-b, name: <grid>)
    node((1.35, -1.35), n[*nav2_map_server*], fill: c-new-b, name: <mapsrv>)
    node((0, -0.45), n[lanelet2 + projector], fill: c-off-b, name: <llfile>)
    node((1.35, -0.45), n[lanelet2 + projection\ loaders], fill: c-auto-b, name: <ll>)

    // ── sensing ──
    node((0, 0.5), n[2-D LiDAR\ #d[(3-D + converter\ in testing, §2.1)]], fill: c-auto-b, name: <lidar>)
    node((1.35, 0.5), n[*scan_qos_bridge*], fill: c-new-b, name: <bridge>)
    node((0, 1.45), n[IMU + wheel vel], fill: c-auto-b, name: <imu>)
    node((1.35, 1.45), n[*wheel_imu_odom*], fill: c-new-b, name: <odom>)

    // ── estimator: NDT out, MCL in ──
    node((2.9, -1.35), n[util.launch\ #dd[voxel downsample]], fill: c-drop-b, stroke: gone, name: <util>)
    node((2.9, -0.45), n[ndt_scan_matcher\ #dd[serves ndt_align_srv]], fill: c-drop-b, stroke: gone, name: <ndt>)
    node((2.9, 0.5), n[*particle_filter*], fill: c-new-b, name: <pf>)
    node((2.9, 1.45), n[gyro_odometer], fill: c-auto-b, name: <gyro>)
    node((4.25, 0.5), n[*mcl_pose_relay*], fill: c-new-b, name: <relay>)

    // ── init: both reconfigured ──
    node((1.35, 2.5), n[pose_initializer\ #d[ndt_enabled → *false*]], fill: c-mod-b, name: <init>)
    node((2.9, 2.5), n[map_height_fitter\ #d[pointcloud_map → *vector_map*]], fill: c-mod-b, name: <fit>)

    // ── fusion + perception ──
    node((5.6, 1.0), n[ekf_localizer\ #d[unchanged]], fill: c-auto-b, name: <ekf>)
    node((5.6, -0.6), n[voxel_based_compare\_map_filter\ #dd[use_pointcloud_map false]], fill: c-drop-b, stroke: gone, name: <cmp>)
    node((7.0, 1.0), n[planning → control\ → vehicle], fill: c-auto-b, name: <plan>)

    // NDT-path edges (dashed)
    edge(<pcdf>, <pcd>, "-->", stroke: gone)
    edge(<pcd>, <util>, "-->", stroke: gone)
    edge(<util>, <ndt>, "-->", stroke: gone)
    edge(<ndt>, <ekf>, "-->", stroke: gone, label: t("pose_with_cov"), label-size: 5.5pt)
    edge(<pcd>, <cmp>, "-->", stroke: gone)
    edge(<init>, <ndt>, "-->", stroke: gone, label: t("ndt_align"), label-size: 5.5pt)

    // MCL-path edges
    edge(<grid>, <mapsrv>, "->")
    edge(<mapsrv>, <pf>, "->", label: t("GetMap"), label-size: 5.5pt)
    edge(<llfile>, <ll>, "->")
    edge(<ll>, <ekf>, "->", label: t("vector_map"), label-size: 5.5pt)
    edge(<ll>, <fit>, "->", label: t("height"), label-size: 5.5pt)
    edge(<lidar>, <bridge>, "->")
    edge(<bridge>, <pf>, "->", label: t("/scan"), label-size: 5.5pt)
    edge(<imu>, <odom>, "->")
    edge(<odom>, <pf>, "->", label: t("/odom"), label-size: 5.5pt)
    edge(<init>, <pf>, "->", label: t("/initialpose"), label-size: 5.5pt)
    edge(<pf>, <relay>, "->")
    edge(<relay>, <ekf>, "->", label: t("pose_with_cov"), label-size: 5.5pt)
    edge(<gyro>, <ekf>, "->", label: t("twist"), label-size: 5.5pt)
    edge(<ekf>, <plan>, "->", label: t("kinematic_state"), label-size: 5.5pt)
    edge(<init>, <ekf>, "->", label: t("initialpose3d"), label-size: 5.5pt, bend: -18deg)
  }
)
])
#diagram-key(
  legend(c-auto-b)[unchanged],
  dashed-swatch[present under `ndt`, absent under `mcl`],
  legend(c-new-b)[added by `mcl`],
  legend(c-mod-b)[reconfigured],
  legend(c-off-b)[offline map artefact],
)

Read against §2: the map row swaps *PCD for PGM* — the file, its loader and both
its consumers (`ndt_scan_matcher`'s differential-map client and the perception
compare-map filter) all drop out, and a `nav2_map_server` serving the grid takes
their place. The lanelet2 and projection loaders are untouched in both, which is
why `mcl` still requires `map_projector_info.yaml`. The estimator row swaps
`ndt_scan_matcher` and its downsample chain for the filter plus relay. The
initialization row keeps the same two nodes but reconfigures both. Everything
from `ekf_localizer` rightwards is identical.


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
  [`ekf_localizer`], [unchanged], [Consumes the same contract topic, with no configuration change. Reached through `pose_initializer` by the ADAPI `/localization/initialize` call, so on a freshly launched stack its first published pose is already the seed.],
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
  caption: [AutoSDV 2D-MCL (`pose_source:=mcl`, seed 4) as fused `kinematic_state`. Same extent, same shared clock.],
)

== Comparison

#figure(
  image("assets/mcl-integrated-overlay.png", width: 58%),
  caption: [Both paths on one axis. The 25 s–55 s markers pair between the two tracks, confirming agreement in time as well as in space. This seed's mean translational error is 0.849 m, p95 2.156 m, mean |yaw| 0.033 rad, worst pose 3.44 m — the median seed of the five, all of which meet all three thresholds.],
)

== Numbers

Five seeds, fused `kinematic_state` versus NDT ground truth, aligned by nearest
timestamp within 100 ms. Each row is a separate launch of the whole stack with
`mcl_random_seed` set to that seed and the value read back off the running node
(2239 paired poses per run):

#table(
  columns: (auto, auto, auto, auto, auto, auto),
  align: (center, right, right, right, right, center),
  table.header([*Seed*], [*Mean (m)*], [*p95 (m)*], [*Max (m)*], [*Mean \|yaw\| (rad)*], [*All three*]),
  [1], [0.831], [2.225], [3.00], [0.034], [#ok],
  [2], [0.830], [2.112], [2.97], [0.034], [#ok],
  [3], [0.872], [2.173], [3.12], [0.032], [#ok],
  [4], [0.849], [2.156], [3.44], [0.033], [#ok],
  [5], [0.865], [2.235], [3.24], [0.034], [#ok],
  [*median*], [*0.849*], [*2.173*], [*3.12*], [*0.034*], [*5/5*],
)

Reporting median and range rather than a single run is deliberate: an earlier
phase found three near-identical configurations spanning 17–27 m of mean error,
so single-run comparisons cannot support a claim about this filter.

=== Why an earlier matrix reported a failing mean

The first end-to-end matrix put the median mean at 2.26 m and passed only one
of five seeds. The cause was in the measurement, not the estimator, and the
recorded bags show it directly. Two defects compounded:

*The seed was never applied.* The ad-hoc driver used its seed number only to
name output files. `random_seed` was never set on `particle_filter`, and no
launch argument existed that could set it, so every run used the shipped
`random_seed: -1` and seeded from entropy. The five rows were five
nondeterministic repeats of one configuration, and none of them is
reproducible.

*One launched stack served all five replays.* `particle_filter` and
`ekf_localizer` keep their converged pose between replays, so every run after
the first began believing the vehicle was where the previous replay had
finished:

#table(
  columns: (auto, auto, auto, auto),
  align: (center, right, right, right),
  table.header([*Run*], [*First EKF pose,\ from its own seed*],
               [*…from the previous\ run's finish*], [*Poses before\ the new seed*]),
  [1st on a fresh stack], [*0.00 m*], [117.86 m], [*0*],
  [2nd], [116.12 m], [1.75 m], [206],
  [3rd], [116.25 m], [1.64 m], [211],
  [4th], [115.97 m], [1.91 m], [223],
  [5th], [116.21 m], [1.68 m], [216],
)

Runs 2–5 start within 1.9 m of the previous run's finish line and 116 m from
where their own ground truth begins, holding there until the seed lands ~5 s
into replay. About two dozen of those pre-seed poses fall inside the
ground-truth track's time span and were scored — precisely the "24 of 2238
poses beyond 20 m in a 0.6 s window" that the earlier report presented as a
startup transient. The first run of each matrix was clean, which is why
exactly one seed passed.

That report attributed the transient to `ekf_localizer` not being gated on
initialization, and named wiring Autoware's `ekf_trigger_node` as the fix.
*That diagnosis was wrong.* On a fresh stack the EKF's first recorded pose is
already at the seed, to two decimals, with no earlier pose in the recording at
all; there was nothing for a trigger to gate. What the EKF did in the reused
stack was correct behaviour given a stack that had been told, in a previous
replay, that it was somewhere else.

Both defects are now structurally prevented rather than remembered:
`mcl_random_seed` is a real launch argument threaded down to the node, and
`scripts/2dlidar/run-mcl-e2e-matrix.sh` launches and tears down a fresh stack
per seed and asserts the seed readback, failing a cell instead of scoring it
when the two disagree. The table above was produced under both guards; every
result row records that they ran.

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
- *No `just` recipe builds a grid.* `scripts/map/pcd_to_pgm.py` works and is what
  the Reproducing section calls, but the operator must already know the height
  band to slice; nothing prints the z-distribution or refuses a bad band, and
  the `autosdv_map.yaml` sidecar that the map-handling design specifies is not
  emitted. `just map-survey` (a `slam_toolbox` path for sites with no PCD) is
  not implemented either.
- *Only the pose-estimator path is measured.* The gate scores
  `kinematic_state` against NDT on a replayed bag. Nothing here exercises
  planning or control on top of a 2-D MCL pose, and no run has been done on a
  vehicle carrying an actual 2-D LiDAR.

= Reproducing

```bash
# validate a map directory for the method
just map-check data/sample-rosbag-replay/sample-map-rosbag mcl

# build a grid from an existing PCD map (no just recipe yet, see sec. 8)
python3 scripts/map/pcd_to_pgm.py <map_dir>/pointcloud_map.pcd \
    <map_dir>/occupancy_grid --z-min <ground+0.2> --z-max <ground+0.5>

# replay with 2D-MCL
just launch-sim-logging ARGS="pose_source:=mcl \
    occupancy_grid_file:=occupancy_grid_scanaccum_mh1r05.yaml"

# the five-seed accuracy matrix: fresh stack per seed, seed readback asserted
SEEDS="1 2 3 4 5" scripts/2dlidar/run-mcl-e2e-matrix.sh

# score a run against NDT ground truth
python3 scripts/2dlidar/compare_poses.py <gt_bag> <mcl_bag> \
    --no-motion-window --gt-time-source bag --out report.md

# regenerate this document's three figures
python3 scripts/2dlidar/plot_trajectories.py \
    --mcl-bag data/rosbags/phase5-fresh/mcl_e2e_s4 \
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
