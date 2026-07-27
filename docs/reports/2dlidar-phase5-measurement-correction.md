# Phase 5 follow-up: the accuracy matrix was measuring the harness

The Phase 5 report closed with one open item: the `mcl` accuracy gate failed
on the mean (median 2.26 m against a 1.0 m threshold) while p95 and yaw
passed on every seed, and
[`2dlidar-phase5-mcl-pose-source.md`](2dlidar-phase5-mcl-pose-source.md) §13
attributed the failure to a 116 m startup transient caused by
`ekf_localizer` not being gated on initialization. The stated fix was to wire
Autoware's `ekf_trigger_node` for `mcl`.

**That attribution was wrong, and the recorded bags disprove it.** The
transient was produced by the measurement harness, not by the localization
stack. Two defects in the driver are responsible, both invisible in the
numbers it emitted.

---

## 1. The seed was never applied

`tmp/t8_e2e_run.sh` used `$SEED` only to name output files and bags. It never
set `random_seed` on `particle_filter`, and until this follow-up there was no
launch argument that could: `mcl_random_seed` did not exist, and the shipped
`mcl_particle_filter.param.yaml` carries `random_seed: -1`, which the fork
reads as "seed from entropy".

Consequence: the five rows labelled *seed 61* through *seed 65* were five
nondeterministic repeats of one configuration. The spread across them was
run-to-run nondeterminism, not seed spread, and none of the runs is
reproducible. Every earlier phase did this correctly —
`scripts/2dlidar/run-seed-matrix.sh` exports `PF_RANDOM_SEED="$SEED"` — so
this is a regression introduced by the ad-hoc Phase 5 driver, not a
long-standing gap.

## 2. One stack was reused across all five replays

The driver assumed an already-running stack and replayed the bag into it five
times. `particle_filter` and `ekf_localizer` keep their converged state
between replays, so every run after the first began believing the vehicle was
where the *previous* replay ended.

Measured on the recorded bags, comparing each run's first EKF pose against
the ground-truth track's endpoints:

| run | first EKF pose, distance from the new run's seed | distance from the **previous** run's finish (GT track end) | EKF poses before the new seed landed |
|---|---|---|---|
| s61 | **0.00 m** | 117.86 m | **0** |
| s62 | 116.12 m | 1.75 m | 206 |
| s63 | 116.25 m | 1.64 m | 211 |
| s64 | 115.97 m | 1.91 m | 223 |
| s65 | 116.21 m | 1.68 m | 216 |

The pattern is unambiguous. Runs 2–5 start within 1.9 m of where the previous
run stopped, 116 m from where the new run's ground truth begins, and hold
there until the oracle seed arrives ~5 s into replay. Of those pre-seed
poses, about two dozen fall inside the ground-truth track's time span and
therefore got paired and scored — which is exactly the "24 of 2238 poses
> 20 m in a 0.6 s window" that the Phase 5 report presented as a startup
transient.

Run s61 is the control: it was the first replay on a freshly launched stack,
its very first EKF pose sits **0.00 m** from the seed, and it has zero
pre-seed poses. That is also why s61 was the one seed that passed.

## 3. Why the `ekf_trigger_node` diagnosis was wrong

The claim was that nothing gates `ekf_localizer`, so it publishes a stale
pose before the filter's first real pose arrives. s61 refutes it directly: on
a clean stack, the EKF's first pose in the recorded bag is already at the seed
(0.00 m) and there is no earlier pose in the recording at all. Whatever the
mechanism, the observable the report wanted — no EKF output before
initialization — already holds on a clean stack. The driver reaches the EKF
through the ADAPI `/localization/initialize` call rather than by publishing
`/initialpose3d` directly, precisely so `pose_initializer` runs its own flow;
that the EKF's first pose coincides with the seed is consistent with the
trigger path working, though this follow-up did not instrument the trigger
service itself to confirm it.

What the EKF did in runs s62–s65 was not ungated publication of garbage. It
was correct behaviour given a stack that had been told, in a previous
replay, that it was somewhere else. No amount of trigger wiring changes
that; only relaunching between runs does.

## 4. What changed

**Launch plumbing (`mcl_random_seed`).** A new argument, defaulting to `-1`
so operational behaviour is unchanged, threaded from both entry launches
(`autosdv.launch.yaml`, `logging_simulation.launch.yaml`) through
`autosdv_autoware.launch.xml` → `tier4_localization_component.launch.xml` →
`localization.launch.xml` → `pose_twist_estimator.launch.xml` →
`mcl_localization.launch.xml`, where it becomes
`-p random_seed:=` on the `particle_filter` process. Verified end to end: the
process cmdline carries `-p random_seed:=62` and the node logs `Seeded numpy
RNG with random_seed=62`.

**A committed driver (`scripts/2dlidar/run-mcl-e2e-matrix.sh`).** Replaces
the `tmp/` scripts, and structurally prevents both defects:

- launches and tears down a fresh stack per seed, waiting for
  `particle_filter`'s "Finished initializing" before proceeding, and killing
  by process group with a `pkill` fallback so orphaned component containers
  cannot hold node names the next seed needs;
- **asserts** the seed reached the node by grepping the run's own log for
  `Seeded numpy RNG with random_seed=$SEED`, failing the cell rather than
  scoring it if the readback disagrees;
- keeps Phase 5's dead-run gate (a frozen EKF pose still yields a
  real-looking mean), the ADAPI-plus-`/initialpose` seeding sequence, and the
  exclusion of the GT bag's own `kinematic_state` from replay.

The `random_seed_confirmed` and `fresh_stack` flags in each result row record
that both guards ran.

## 5. Corrected measurement

Five seeds, each a separate launch of the whole stack with `mcl_random_seed`
set and read back, scored on the fused `/localization/kinematic_state` against
the same NDT ground-truth bag, 2239 paired poses per run
(`data/rosbags/phase5-fresh/results.jsonl`):

| seed | mean (m) | p95 (m) | max (m) | mean \|yaw\| (rad) | all three |
|---|---|---|---|---|---|
| 1 | 0.8307 | 2.2248 | 2.9972 | 0.0338 | PASS |
| 2 | 0.8295 | 2.1116 | 2.9665 | 0.0341 | PASS |
| 3 | 0.8719 | 2.1732 | 3.1159 | 0.0324 | PASS |
| 4 | 0.8492 | 2.1561 | 3.4441 | 0.0325 | PASS |
| 5 | 0.8647 | 2.2350 | 3.2355 | 0.0343 | PASS |

| metric | median | range | limit | |
|---|---|---|---|---|
| mean translational | 0.849 m | 0.830–0.872 | < 1.0 | **PASS** |
| p95 translational | 2.173 m | 2.112–2.235 | < 2.5 | **PASS** |
| mean \|yaw\| | 0.034 rad | 0.032–0.034 | < 0.2 | **PASS** |

**5/5 seeds meet all three thresholds. The gate is met.**

The decisive detail is the max column: the worst pose in any of the five runs
is 3.44 m, against 116.36 m before. The 116 m excursions are simply absent
once each run starts on a stack that has not been told it is somewhere else.
Every other quantity is close to the old s61 row (0.824 / 2.101 / 0.032),
which is what the diagnosis predicts — s61 was already a clean first-run cell,
so it should not move, and it did not.

Median mean also lands within 0.03 m of the Phase 4 filter-only gate
(0.826 m). That the end-to-end number matches the filter-only number is the
expected outcome of a sound integration: the relay and EKF are passing the
estimate through rather than degrading it, which is what the Phase 5 report's
own chain measurement (filter 2.638 → relay 2.638 → EKF 2.642) indicated
before the harness noise obscured it.

Also worth stating plainly: the spread across these five rows is *genuine seed
spread*, for the first time in this series' end-to-end measurements. It is
narrow — 0.04 m of mean, 0.12 m of p95 — which is a stronger statement about
the method's stability than any previous end-to-end table could make, because
previous tables did not vary what they claimed to vary.

## 6. Reproduce

```bash
# five seeds, fresh stack each, sample site, Phase 4's fine grid
SEEDS="1 2 3 4 5" scripts/2dlidar/run-mcl-e2e-matrix.sh

# results, one row per seed
cat data/rosbags/phase5-fresh/results.jsonl
```
