# Demos

Self-contained scenarios that run end to end from a single command, including
their own data preparation. Recipes live in a justfile module, so the
repository's top-level `just --list` stays short:

```bash
just demo               # list the demos
just demo check         # are the prerequisites in place?
just demo run           # run the COSS NDT replay, with RViz
just demo stop          # stop the stack it leaves running
just demo report        # metrics for the most recent run
```

## COSS NDT replay

Localizes against the COSS park map by replaying
`data/rosbags/outdoor_20251226_153115` (157 s: parked for the first 115.7 s,
then a 41 s drive at up to 1.58 m/s). Nothing about the vehicle is required --
the whole thing runs from the recording.

```bash
just demo run                # everything: fetch data, launch, seed, replay, summarise
```

That single recipe:

1. fetches the rosbag if it is missing (~2.8 GB) and checks the map is present,
2. stops any stack left over from a previous run, since two would fight for topics,
3. launches `logging_simulation.launch.yaml` with `pose_source:=cuda_ndt`,
   `use_gnss:=false` and RViz (when `DISPLAY` is set),
4. waits for `ndt_scan_matcher` and gives the 4.9 M-point map 25 s to load,
5. starts the wheel-speed scaler and remaps the bag's raw velocity topic through it,
6. records every localization diagnostic to `tmp/demo-runs/<label>_<stamp>/bag`,
7. replays the bag, seeding the initial pose 8 s in,
8. prints the metrics and leaves the stack up so the result can be inspected.

Stop it with `just demo stop` — that kills the whole process group, which
matters: killing the launcher by PID alone orphans the component containers.

### Variants

| recipe | difference |
|---|---|
| `just demo run` | the reference configuration |
| `just demo run-headless` | no RViz, stack stops at the end; for CI or a remote box |
| `just demo run-raw-speed` | the bag's uncorrected wheel speed, so you can see the lurching it causes |
| `just demo run-manual-init` | no pose seeding; set it yourself with RViz's *2D Pose Estimate* during the parked window |

### What a healthy run looks like

| metric | expected |
|---|---|
| published poses | every frame, ~1425 |
| worst publish gap | < 0.2 s |
| per-frame position scatter | ~0.010 m |
| init-to-result distance, moving | ~0.05 m |
| NVTL | ~2.76, against a 2.3 gate |
| heading vs direction of travel | within ~0.1 deg |
| `imu_corrector` "Please publish TF" errors | **0** |

The run prints the first six. Check the last one first if anything looks odd:

```bash
grep -c "Please publish TF" play_log/latest/node/imu_corrector_node/err
```

A nonzero count means the IMU transform is missing again, the EKF has stopped
propagating, and every other number is meaningless. That was the original bug;
see `docs/reports/cuda-ndt-coss-replay.md`.

### Analysing runs

```bash
just demo report                       # the most recent run
just demo report tmp/demo-runs/coss-ndt_20260802_120000
just demo yaw-bias                     # heading minus course, straight segments only
just demo compare a=<run_dir> b=<run_dir>
just demo list-runs
just demo clean                        # runs are ~2 GB each
```

### Two things the demo works around

Both are recorded in the bag and cannot be fixed by any parameter:

- **Wheel speed reads ~1.8x high.** `demo/scripts/velocity_scaler.py` republishes
  a corrected copy; disable it with `SCALE= just demo run` (or use
  `run-raw-speed`). The real fix is on the vehicle.
- **GNSS is single point** with ~20 m of scatter and disagrees with the direction
  of travel, so `use_gnss:=false` and the pose comes from
  `demo/scripts/seed_initialpose.py`, which replays the pose an operator set by
  hand and confirmed against the map.

See `docs/guides/ndt-tuning.md` before changing any NDT parameter on the basis
of what you see here.
