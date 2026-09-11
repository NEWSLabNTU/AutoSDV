# Verifying the backport

**Goal**: turn `docs/roadmap/6-golfcart-backport.md`'s ten phases from "written
and compiled" into "seen working". Nothing in that campaign was launched.

**Status**: proposed

---

## What is already verified, and what that is worth

Worth distinguishing, because the campaign's commits say both:

| Verified | How |
|----------|-----|
| `cuda_pointcloud_filters` kernels | 12 unit tests against real device memory, passing on this desktop's GPU |
| The CUDA preprocessor accepts our new point layout | a synthetic `PointXYZIRCAEDT` cloud, processed at the full 10 Hz input rate |
| The layout itself is right | **a recorded Velodyne cloud has exactly it** — see below |
| Launch-logic branches | `launch_setup` driven with a stub context: cpu, cuda, and each refusal |
| The mode strip's rendering | 29 checks against a graph fixture captured from a live aggregator |
| The setup registry | `--list`, `--status`, `--run --dry-run` for two profiles |
| Every systemd unit | `systemd-analyze verify` on the rendered units |

None of that is a running stack. Everything below is.

## The bags change what needs a vehicle

`/home/aeon/nas/autoveh/dataset/2025-11-14 AutoSDV Localization RosBags in COSS`
holds four recordings, 9.3 GB:

| Bag | Duration | LiDAR topic | Layout |
|-----|----------|-------------|--------|
| `vlp32_1`, `vlp32_2` | 28.5 s each | `/sensing/lidar/velodyne_points`, 42,711 pts/frame | **`PointXYZIRCAEDT`, `point_step` 32** |
| `robin_1` | 153 s | `/sensing/lidar/iv_points`, 85,210 pts/frame | `PointXYZIRC`, `point_step` 16 |
| `robin_2` | 74.6 s | same | same |

All four carry `/sensing/imu/imu_data`, `/vehicle/status/*`, GNSS and
`/sensing/lidar/concatenated/pointcloud`, so they can drive
`just sim logging` without any sensor attached.

Two consequences that decide the order of everything below.

**The Velodyne bags already contain the layout this campaign taught the Seyond
driver to publish.** Read from `vlp32_1`, field for field: x0 y4 z8,
intensity12, return_type13, channel14, azimuth16, elevation20, distance24,
time_stamp28. That is what `point_xyzircaedt.h` declares. The layout is
therefore confirmed against a real recording, not only against the header file
it was copied from — and the whole CUDA sensing chain can be exercised on this
desktop, today, with no vehicle.

**The Robin-W bags cannot validate the new driver.** They were recorded with the
old one: 16 bytes per point, no `time_stamp` field. The per-point time cannot be
reconstructed from them — the information is not there. Anything about deskewing
the Robin-W needs a **new** bag off the rebuilt driver, and a *moving* one,
because deskew is invisible while stationary.

Note the bandwidth this makes concrete: 85,210 points per frame at 16 bytes is
1.36 MB; at 32 it is 2.73 MB, at 10 Hz. That is the cost of the new layout on
the sensor this vehicle actually runs, and it lands on the same deliver thread
the QoS fix unblocked.

---

## Running any of this: five traps

Found the hard way on 2026-09-11/12, each producing a silent non-measurement
rather than an error. Anyone repeating phases C or D will meet them.

1. **No `set -u` in a script that sources ROS.** `setup.bash` and the ament
   hooks read deliberately unset variables, so under nounset the source aborts
   *inside the sourced file* and the script dies before launching anything, with
   no output. This cost three separate debugging rounds.
2. **`--container-mode` decides whether you can measure at all.** play_launch's
   default, `isolated`, forks composable nodes behind its private control
   channel, and an outside `ros2 topic hz` sees none of the graph — it reports
   "does not appear to be published yet" while the stack runs perfectly. But
   `observable` costs **36 LoadNode service calls timing out at 30 s each** on
   this stack before deferring to ComponentEvent. Use `--container-mode stock`:
   ordinary ROS containers, visible, no waiting.
3. **Copy the bag to local disk.** Played off the NAS, `ros2 bag play` logs
   "Message queue starved" every second and delivers almost nothing, so the
   measurement window sees an idle graph. `data/rosbags/` is gitignored.
4. **Launch only the modules under test.** The full stack is 130 members and
   takes minutes to construct; two overlapping launches exhausted a 125 GB
   machine's memory and the OOM killer took the harness with it. For the sensing
   chain, `launch_perception:=false launch_planning:=false launch_control:=false
   launch_system:=false` is enough, and the vehicle and map stay because the
   chain ends in a transform to `base_link`.
5. **Discovery completes per-subscription long before the CLI inventory.**
   `ros2 topic hz` on a named topic returns data while `ros2 topic list` still
   shows two topics. Allow ~40 s, and run the probes one at a time: in parallel
   they compete for discovery and each reports a partial answer.

---

## Phase A — Build and test — PASS 2026-09-12

33 packages, exit 0, 2 min 8 s. `cuda_ndt_matcher` compiled for the first time
since the 22-commit bump (2 min 5 s of that total). `seyond` built with
`PointXYZIRCAEDT` as the default, and the built binary carries the `time_stamp`
field. `cuda_pointcloud_filters` built from its submodule path.

`colcon test`: the functional tests pass. Of 687 tests, the failures are
overwhelmingly lint — copyright, flake8, pep257, uncrustify, cpplint,
lint_cmake — in vendored packages (`zed_*`, Isaac, manual control).

**One package could not build at all, and it is now fixed.** `range_libc`'s
`RangeLibc.pyx` carried seven Python-2 `print` statements, which Cython 3
rejects as syntax errors:

```
print "Failed to construct PyOMap, check argument types."
      ^
RangeLibc.pyx:179:18: Syntax error in simple statement list
```

So `import range_libc` failed, `particle_filter`'s test suite errored during
collection rather than running, and `pose_source:=mcl` had no raycaster. An
older Cython compiled it, which is why the machine that measured MCL never saw
this. Fixed in the fork (`eee866b`), and it now has a setup step of its own —
it had none, and no documentation beyond a `compile.sh` reading
`sudo python setup.py install`. `particle_filter` now runs 57 tests, 54 passing,
the other 3 being lint.

---

## Phase B — Launch resolution — PASS 2026-09-12

**Parser parity**, the item phase 5 left open: 165 nodes, 113 topics, and
identical node key sets under the Rust and the Python parser. The Rust parser is
safe to rely on here.

**The `gnss_enabled` fix is confirmed at the graph level.** `use_gnss:=false`
now removes four nodes:

```
- /localization/util/default_adapi/helpers/autoware_automatic_pose_initializer_node-1
- /sensing/gnss/gnss_poser
- /sensing/gnss/ublox/sensing/gnss/ntrip/ntrip_client
- /sensing/gnss/ublox/ublox
```

The first is the point: before the fix it ran regardless of `use_gnss`, and
`pose_initializer` then waited on a GNSS pose that never came.

**Both CUDA switches resolve.** `pointcloud_backend:=cuda` adds
`/sensing/lidar/cuda_pointcloud_preprocessor_node`;
`localization_pointcloud_backend:=cuda` swaps the three localization filters to
`cuda_pointcloud_filters::CudaCropBoxFilterNode`, Autoware's
`CudaVoxelGridDownsampleFilterNode` and
`cuda_pointcloud_filters::CudaRandomDownsampleFilterNode` — same node names, so
the stage switches whole, which is the design.

---

## Phase C — The CUDA sensing chain — PASS 2026-09-12

Run on `vlp32_1`, copied to `data/rosbags/`, with the stack trimmed to sensing +
vehicle + map and `--container-mode stock`.

| | `pointcloud_backend:=cpu` | `pointcloud_backend:=cuda` |
|---|---|---|
| `/sensing/lidar/preprocessed/pointcloud` | absent | `point_step` 16, 33,353 pts, `base_link` |
| `/sensing/lidar/concatenated/pointcloud` | `point_step` **32**, 42,951 pts, `base_link` | `point_step` **16**, 33,444 pts, `base_link` |
| fields on the concatenated cloud | all ten (x…time_stamp) | six (x…channel) |

What this establishes:

- **The switch is whole-stage and it works.** In `cuda` the CUDA preprocessor
  runs, publishes, and its output is what reaches `concatenated/pointcloud` —
  six fields at 16 bytes, against the raw layout's ten at 32.
- **Both arms deliver in `base_link`.** This was the check worth making: the
  CUDA preprocessor does not transform frames, so had the passthrough been
  bypassed the cloud would silently have arrived in the sensor frame and
  everything downstream would have been wrong with no error anywhere.
- **The preprocessing does work**: 42,951 → 33,444 points, −22%, which is
  crop-self plus the ring outlier filter removing returns.
- One correction to the design doc: the CUDA preprocessor's own output is
  already in `base_link` here, not the sensor frame.

**A methodological trap, worth more than the numbers.** The bag *contains*
`/sensing/lidar/concatenated/pointcloud` — 284 messages of it, recorded by the
run that produced the bag. Replayed whole, that topic is published by the
player, not by the stack, and **both backends then measure identically**: 32
bytes, 42.7k points, ~10 Hz, whatever the switch is set to. The first CUDA arm
measured exactly that and looked like a null result. The replay is now an
allowlist — the raw cloud, IMU, velocity report and TF — so the only publisher
of the topic under test is the stack.

**Rate parity holds.** Once the DDS problem below was fixed:

| | `cpu` | `cuda` |
|---|---|---|
| `velodyne_points` | 10.204 Hz | 10.235 Hz |
| `concatenated/pointcloud` | 9.631 Hz, std 0.0075 s | 10.124 Hz, std 0.0084 s |
| GPU power while running | 7.39 W | **57.85 W** |

Both arms hold the bag's own 10 Hz with tight jitter: the CUDA chain drops no
frames. The GPU power difference is the clearest evidence the work actually
moved — a 50 W swing on an idle desktop card. Instantaneous `nvidia-smi`
utilization read 0% in both, having sampled between kernel bursts, and the
per-container CPU comparison did not survive this instrument either; the CPU
side of the trade needs `scripts/profiling/` on the Orin, not `top` on a
desktop.

**Everything above was unmeasurable until the DDS environment was fixed**, and
this is the finding with the longest reach. `install/setup.bash` alone leaves
`RMW_IMPLEMENTATION` unset, so a script that sources only the workspace overlay
runs on **Fast-DDS**, while this repo's kernel-buffer setup step, its
`cyclonedds.xml` and CLAUDE.md all assume CycloneDDS. Nothing errors. Discovery
half-works: `ros2 topic list` returns two topics, `ros2 topic echo` cannot
resolve a type that `ros2 topic hz` is already reading, and the same arm
measures 3.4, 9.1, 11.3 or 20.2 Hz depending on the run. Stale
`/dev/shm/fastrtps_*` segments from killed runs compound it
("open_and_lock_file failed"); 473 had accumulated here.

Sourcing `/opt/autoware/1.5.0/setup.bash` *before* the overlay fixes it — that
is where `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` comes from. Two related facts
worth knowing: `.envrc` sets the RMW only in its *fallback* branch, so on any
machine with Autoware installed it never runs; and Autoware's own
`/opt/autoware/1.5.0/config/cyclonedds.xml` wins, so the repo's tuned
`cyclonedds.xml` is not what any of this used.

---

## Phase D — Localization on the same bags

```bash
just sim logging ARGS="pose_source:=cuda_ndt"
python3 scripts/testing/localization/check_ndt_activated.py --timeout 30
python3 scripts/testing/localization/ndt_quality_report.py --seconds 60
python3 scripts/testing/localization/ndt_alignment_report.py --seconds 60
```

The COSS map is `data/COSS-map-planning`, which is what these bags were recorded
against, and what `ndt_alignment_report.py` now defaults to.

**Pass**: NDT activates; scan-to-map p50 within roughly one `ndt.resolution`
voxel; `localization_pointcloud_backend:=cuda` gives the same poses as `:=cpu`
within noise. The second is the real point — that switch has only ever been
checked for *correctness of wiring*, never for producing the same answer.

Then the one measurement phase 2 explicitly left undone: time the localization
chain in both arms. The CPU chain is about 19% of a core; the CUDA one has never
been timed and could be slower.

## Phase E — The Robin-W, which needs the vehicle

Everything above runs on a desktop. This does not.

1. **Rebuild and record.** With the rebased driver, record a short bag with the
   Robin-W **moving** — stationary cannot show deskew.
2. **Layout**: `point_step` 32 and ten fields at the offsets in phase C's table.
3. **The QoS fix**: the driver's own deliver-queue counters over a run of a few
   minutes, `added` against `dropped`. The golf cart saw 60% dropped before and
   the fix removes the cause; this is the confirmation.
4. **Deskew**: `pointcloud_backend:=cuda lidar_model:=robin-w` on that bag, and
   look at structure that the vehicle drove past — a wall, a pole — before and
   after. If the time offsets have the wrong origin, the cloud is plausible and
   sheared, which is why this needs eyes on real geometry rather than a rate
   check.
5. **Bandwidth**: 2.73 MB per frame at 10 Hz on this sensor. Watch the deliver
   thread and the concatenator, not just the topic rate.

## Phase F — The diagnostic graph, on the board

Settles the open entries in `docs/known-config-defects.md`.

```bash
just launch            # then open the monitor at :8080
```

1. **The mode availability strip** against a real `autoware_diagnostic_graph_aggregator`.
   It has only been driven from a fixture. Watch specifically for the
   index-join: chips labelled with the wrong mode look entirely plausible.
2. **How much of the graph is red.** The golf cart measured 31.1% of reports
   ERROR or STALE, nearly all configuration. Record ours.
3. **`system_monitor`**: confirm the four monitors that cannot pass on a Jetson,
   then decide whether to copy the golf cart's fix (name the interfaces, do not
   launch the three a parameter cannot fix).
4. **`topic_state_monitor_initialpose3d`**: whether it is ERROR on 100% of
   reports here too. If so, that is a report to make upstream — the all-zero
   thresholds are Autoware's own value, byte for byte.

## Phase G — The golf cart

The other vehicle now shares the driver, and one decision is waiting on a
measurement rather than on an edit: whether its Falcon moves from
`PASSTHROUGH_LIDARS` into `PREPROCESSED_LIDARS`. That changes what
`concatenate_and_time_sync_node.param.yaml` lists, and that concatenator is
known to be timing-sensitive — a silent second LiDAR halved its rate through
`timeout_sec` alone, and the timeout has since been brought under the scan
period. Do it against a bag with both sensors live, and compare the rate before
and after.

---

## Order, and what each phase costs

| Phase | Needs | Rough cost |
|-------|-------|-----------|
| A — build | this machine | one build |
| B — launch resolution | a built workspace | minutes |
| C — CUDA sensing | a built workspace + `vlp32_*` | an hour |
| D — localization | same | an hour |
| E — Robin-W | **the vehicle**, a new bag | a session |
| F — diagnostic graph | **the board** | a session |
| G — golf cart Falcon | **that vehicle**, a two-LiDAR bag | a session |

A through D need no hardware at all, which was not true before the bags turned
up. They should be done first, and in order: each one's failure would make the
next one's result meaningless.
