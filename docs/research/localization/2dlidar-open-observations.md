# 2-D MCL: unexplained observations

Things measured during the 2-D MCL work that are real, reproducible in the data,
and **not understood**. They are recorded here rather than left in commit
messages so they can be picked up deliberately, and so nobody re-derives them
from scratch.

Each entry states the evidence, why it matters, and the cheapest experiment that
would settle it.

---

## 1. Heading error halves with a 3-ring scan source

**Observation.** Mean |yaw| error is roughly half with three rings compared with
either a single ring or the thicker z-slab. Per seed, five seeds each:

| seed | slab | 1 ring | 3 rings |
|---|---|---|---|
| 1 | 0.0339 | 0.0290 | **0.0161** |
| 2 | 0.0334 | 0.0324 | **0.0159** |
| 3 | 0.0336 | 0.0321 | **0.0149** |
| 4 | 0.0340 | 0.0332 | **0.0151** |
| 5 | 0.0342 | 0.0270 | **0.0181** |

rad. The separation is clean: every 3-ring seed beats every seed of both other
sources, with no overlap.

**Why it is odd.** The translational improvement was predicted — three rings
carry more returns than one, so each update is better constrained. Heading was
not. And the slab, which has *more* returns than three rings, is the worst of
the three, so this is not simply "more points, better yaw".

**Hypothesis, untested.** The slab is a fixed 0.30 m window in `base_link`, so at
a given bearing it may admit returns from several elevations whose apparent
bearing differs slightly; flattening then smears the angular structure the filter
uses to resolve heading. Three adjacent rings span 0.22°, so their returns are
nearly co-planar and the angular structure survives. If that is right, yaw error
should degrade monotonically with group width.

**Experiment.** Sweep group width — 1, 3, 5, 9, 15 rings — through the existing
matrix and plot mean |yaw| against span in degrees. One command per width,
~20 minutes each:

```bash
SCAN_MODE=ring SCAN_RING_MIN=67 SCAN_RING_MAX=75 OUT_DIR=... \
    scripts/2dlidar/run-mcl-e2e-matrix.sh
```

If yaw is flat with width, the hypothesis is wrong and something specific to the
slab's fixed-window geometry is responsible.

---

## 2. MCL and NDT drive the same bag at different speeds

**Observation.** In the downstream probe, mean ego speed over the observation
window is ~1.8 m/s under `pose_source:=mcl` and ~3.8 m/s under `ndt`, on the
same re-stamped bag, same route, same replay rate.

**Why it matters.** It makes the cross-track columns of the MCL/NDT comparison
in `2dlidar-downstream-planning-control.md` **not like-for-like**: slower driving
tracks tighter, so MCL's better cross-track may be partly or wholly a speed
artefact. Any future comparison of those numbers is unsound until this is
explained.

**Candidate causes, none checked.**
- The MCL run engages autonomous mode and the NDT run does not (stock diagnostic
  graph, §5.7 of the downstream report). An engaged controller may be commanding
  a different velocity profile than the replayed one.
- The observation window may cover a different segment of the bag in each run,
  since it starts after initialisation and the two initialise at different times.
- `/localization/kinematic_state` twist may be derived differently along the two
  paths.

**Experiment.** Log ego speed against sim time for both runs and overlay them.
If the profiles differ in *shape*, the controller is acting; if they differ only
in *window*, it is a sampling artefact and the fix is to score a fixed sim-time
window in both.

---

## 3. One seed lost 19% of its poses

**Observation.** In the single-ring matrix, seed 1 produced **1813** paired poses
where every other run in every other configuration produced 2238–2239.

| configuration | pairs per seed |
|---|---|
| slab | 2238, 2239, 2239, 2239, 2239 |
| 1 ring | **1813**, 2239, 2239, 2239, 2239 |
| 3 rings | 2239 × 5 |

**Why it matters.** That seed's mean of 0.8895 m is computed over a fifth fewer
samples than its peers, so it is the least trustworthy figure in the scan-source
comparison — and it happens to be the *best* of the single-ring seeds, which
flatters the configuration that otherwise looked worst.

**Candidate causes, none checked.** A late-starting or early-dying estimator; a
replay hiccup; the pairing filter (100 ms max |dt|) rejecting more pairs because
the estimator ran at a lower rate that run.

**Experiment.** Re-run that single cell a few times and see whether the pair
count is stable. If it recurs, compare the recorded bag's pose timestamps
against the GT track to find where the pairs are lost. The matrix already records
every run's bag, so this is offline work.

---

## 4. The cross-track metric is partly self-referential

**Not a mystery, but a standing caveat** that belongs with the numbers it
qualifies. The downstream probe measures the vehicle's distance to *its own
planned trajectory*, and the trajectory is planned **from the pose being
measured**. It therefore reports how well control follows a plan, not how
accurate the pose is: a badly localized vehicle following a consistent plan can
score well.

The localization number is the five-seed matrix against NDT ground truth
(0.789 m for 3 rings), which uses an external reference. Do not quote cross-track
as a localization result.

**Would fix it.** Score the ego pose against the NDT ground-truth track over the
same window, alongside the plan-relative figure, so both are visible.

---

## 5. Diagnostic branches that stay STALE in passing runs

**Observation.** Even in runs that reach AUTONOMOUS, the aggregator has reported
`/autoware/localization/state` and `/autoware/planning/routing/state` as STALE at
various points, and `/adapi/mrm_request/delegate` STALE.

**Why it matters.** If those are genuinely never published, the availability
logic is passing for reasons other than the ones the graph appears to encode,
and a future change could silently depend on that. If they are merely slow to
appear, the diagnostics are noisy and train people to ignore them.

**Experiment.** Subscribe to the diagnostic graph during a passing run and record
each branch's state over time, rather than reading the aggregator's log at
teardown.

---

## Not in this list

Known and already explained elsewhere: the beam-independence and max-range
clamping defects (`2d_mcl_algorithm.md` §5.4, §5.5), the GNSS heading seed being
1.81 rad wrong with a placeholder covariance
(`mcl_initialization_and_covariance.md`), and the residual 0.05–0.22 m offline
offset between the sensor model's argmax and the true pose
(`docs/tech/2d-mcl-localization.typ`, "What is not established").
