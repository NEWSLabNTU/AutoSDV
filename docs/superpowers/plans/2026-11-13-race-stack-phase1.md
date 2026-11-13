# Race stack phase 1: submodule, regression gate, Orin benchmark — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** autoware-drift sits in AutoSDV as a submodule without changing anyone's build, its
full-size results are guarded by a bit-for-bit regression gate, and L1, L2 and a first acados
NMPC are timed on a Jetson AGX Orin under AutoSDV's load.

**Architecture:**
- **autoware-drift** gains three generic things under `src/planning/race_planner`:
  - tests that skip, instead of failing, when swb's course data is absent;
  - a regression gate that runs a fixed set of deterministic runs at HEAD and at a base
    commit (through the harness's existing `RP_ROOT`) and compares their digests;
  - two benchmark studies: L1/L2 timing, and a first acados model of the tracker.
- **AutoSDV** gains:
  - the submodule;
  - a setup step that builds a private Python venv and acados (no system Python changes);
  - a benchmark wrapper;
  - the benchmark report.

**Tech Stack:** Python 3.10 (Humble's), CasADi 3.6+/IPOPT/HPIPM, acados (C, Python template),
uv or venv, colcon, just. Measured on newslab-server133 (x86 reference) and a car's AGX Orin.

**Spec:** `docs/superpowers/specs/2026-11-12-autoware-drift-integration-design.md`
(phase doc: `docs/roadmap/16-autoware-drift/1-submodule-gate-benchmark.md`).

## Status (2026-11-13): Tasks 1–6 done, Task 7 blocked on an Orin

| task | commits | result |
|---|---|---|
| 1 test skips | autoware-drift `b366069` | suite on py3.10: 0 failed |
| 2 regression gate | autoware-drift `d6c5b12`, `a20a0f7` | `just regress-check`: 6 runs, bit for bit; an error never passes; Ctrl-C kills the runs |
| 3 submodule, setup step | AutoSDV `a243db4` | acados v0.6.0; `just build`/`just test` unchanged (337 pre-existing failures, same by name) |
| 4 L1/L2 timing study | autoware-drift `21c796a`, `30471d5` | x86 smoke: L1 p95 0.227 s, L2 p95 2.6 ms |
| 5 acados tracker | autoware-drift `7d7afee` | x86: time_tot p95 0.30 ms, 488/488 status 0 |
| 6 bench wrapper | AutoSDV `01eb7ce`, `4ffc560` | `bench.sh` + `bench_report.py` |
| final-review fixes | autoware-drift `b9165dd`; AutoSDV `b196276`, `381ce5c` | 8 findings fixed; suite 229 passed, 7 skipped |
| 7 Orin runs, report | — | blocked: needs a car's Orin |

Deviations from the plan, as decided while executing:
- `race-env.sh` does not set `LD_LIBRARY_PATH`: acados' `libhpipm`/`libblasfeo` shadowed
  CasADi's copies and broke 6 tracker tests. acados is built with `RUNPATH=$ORIGIN` instead.
- The CasADi tracker and acados never share a process (two copies of HPIPM).
- `bench_live` passes `cpus[1:]` to the L1 seed workers; `cpus[0]` is the control loop's.
- `bench_live` takes `--length` and `--budget` (fields of `live.Driver`).
- The acados model omits the Tracker's delta tracking, jerk and grip rows: its solve time
  is a lower bound for phase 4's NMPC.
- Under a 1-minute load of ~53 (other users) server133's L1 p95 rose to ~0.87 s: L1 is
  load-sensitive.

Follow-ups from the reviews: `docs/roadmap/16-autoware-drift/1-follow-ups.md`.

## Global Constraints

- **Opt-in only.** `just build` and `just test` without the new dependencies give today's
  results plus the two autoware-drift packages (`drift_controller`, `drift_launch`);
  `autosdv.launch.yaml` is not touched.
- **No system-Python changes.** race_planner needs `numpy>=1.26`, `scipy>=1.11`; Jammy's apt
  has 1.21 / 1.8, and a user-site numpy breaks AutoSDV's build
  (`scripts/build/check-python-env.sh` refuses it). Everything race-side installs into its
  own venv: `${AUTOSDV_RACE_VENV:-<repo>/.venv-race}`, Python 3.10.
- **acados** installs without sudo into `${ACADOS_ROOT:-$HOME/.local/opt/acados}`, at a tag
  pinned in `versions.yaml` (`race.acados`).
- **Submodule:** `src/planning/external/autoware-drift`,
  `url = https://github.com/jerry73204/autoware-drift.git`, `branch = main`. `race_planner`
  keeps its `COLCON_IGNORE`.
- **autoware-drift pushes** go to its `main`, only after `just regress check` passes against
  `origin/main` (Task 2). Lockstep: autoware-drift push before the AutoSDV pin.
- **Where autoware-drift work happens:**
  - Tasks 1, 2, 4 and 5 commit in the clone `~/repos/autoware-drift` on `main`.
  - The AutoSDV submodule pins what was pushed.
  - Their venv is the package's own `src/planning/race_planner/.venv`, made with
    `uv venv --python /usr/bin/python3.10 && uv pip install -e '.[test]'` (Humble's
    interpreter, as on the cars; never `$HOME/.local` Pythons). Commands below write it as
    `.venv/bin/python` and always run under `env -u PYTHONPATH`.
- **swb:** tests, the gate and the benchmarks read swb through `$SWB`
  (`~/repos/scenario-world-bridge` on server133; the code's default
  `~/repos/carla-scenario-bridge` does not exist here). Never write into swb.
- **Determinism:** the gate runs with `RP_DET=1` (no clock in the solvers). Benchmarks run
  with the clock **on** (no `RP_DET`), because they measure wall time.
- **Temp output** in AutoSDV's `./tmp`; results that are kept go into the report.
- **No sudo.** Don't touch display :2. On server133 CUDA work exports
  `CUDA_MPS_PIPE_DIRECTORY`/`CUDA_MPS_LOG_DIRECTORY=~/repos/AutoSDV/tmp/no-mps`.
- Commit trailers per the session's attribution rule.

## Measured while planning (server133, 2026-11-13; for calibration, not acceptance)

- `RP_DET=1 ... mode_choice_m4.py run c:gravel:15:90:8 --arm sched --digest`: 49 s cold
  (JIT compile into `~/.cache/race_planner`), 17 s warm. Two runs gave the same
  `plans_digest` (`fec37b5e…`) and `traj_digest` (`81fa1c19…`), t 9.77 s, 11 plans.
- The race_planner suite on Python 3.14 + CasADi 3.8.2: 4 failed, 203 passed, 4 skipped, in
  18 min 26 s. All four failures are `FileNotFoundError` on Haruna's `centreline.npz` (no
  `$SWB/data/courses/haruna` on this host: Haruna and Tsukuba are built from open data by
  swb's course generator and never committed). Task 1.
- Each run's JSON already has the L1 timing (`solve_p50/p95/p99/max`, `overruns` over 1 s,
  `solve_all`, `adopt_p99`) and `load0`/`load1`. L2 (`Tracker.step`) times itself
  (`solve_s` in its output) but the harness does not aggregate it.

## Review Focus

1. **The gate run on a dirty tree or with HEAD == base.** Expected: a dirty race_planner tree is
   compared as it stands and the report says "working tree (dirty)"; HEAD == base prints
   "nothing to compare" and exits 0 without running. Pinned in Task 2's tests.
2. **A run that raises or ends `result != "done"` on one side only.** Expected: the gate fails
   and names the run and both outcomes; it never crashes or passes. Both sides failing the same
   way with equal digests passes. Pinned in Task 2.
3. **`$SWB` unset or missing a file the set needs.** Expected: the gate and the benchmarks stop
   before running anything, naming the missing path. Pinned in Tasks 2 and 4.
4. **The setup step on a machine with no `uv`, or rerun on an existing venv/acados.** Expected:
   it falls back to `python3 -m venv`, and a rerun is a no-op that verifies instead of
   rebuilding. Pinned in Task 3's verify command and its rerun step.
5. **The benchmark on a loaded or throttled Orin.** Expected: each result records the load
   averages, the Orin's power mode (`nvpmodel -q`) and clocks state, so a slow number can be
   told from a slow board. Pinned in Task 6's summary test.

---

### Task 1: Tests skip without swb's course data (autoware-drift)

The suite already skips four tests when Haruna's data is missing
(`@pytest.mark.skipif(not os.path.isdir(HARUNA), reason="Haruna's course data is not here")`,
`test_car_track.py:42`). Four more read it without the guard. Each of them fails on a host
without the data, with
`FileNotFoundError: …/data/courses/haruna/course/centreline.npz`:

- `test_m4_courses.py::test_a_one_class_course_is_that_class_everywhere[haruna@snow_packed]`
  (only the `haruna@…` case; `zones@gravel` still runs)
- `test_m4_courses.py::test_a_one_class_course_keeps_the_dry_geometry`
- `test_schedule.py::test_a_certain_map_gives_the_planner_the_track_060_plans_on`
- `test_surfaces.py::test_each_class_s_stiffness_and_peaks_are_swb_s`

(Measured on server133, Python 3.14: 4 failed, 203 passed, 4 skipped, in 18 min 26 s.)

**Files:**
- Modify: `test/test_m4_courses.py`, `test/test_schedule.py`, `test/test_surfaces.py`

- [ ] **Step 1: Confirm the failures.** Run the four tests by node id with
  `SWB=$HOME/repos/scenario-world-bridge`. Expected: 4 FAILED, `FileNotFoundError` on
  `centreline.npz`. This is the RED for a test-infrastructure fix.
- [ ] **Step 2: Add the existing guard** to each, with the same reason string. Use
  `pytest.param("haruna@snow_packed", marks=…)` for the parametrized case. If
  `test_surfaces.py` needs Haruna only for a part of the test, guard that part, not the whole
  test.
- [ ] **Step 3: Run the four again.** Expected: 3 skipped with the reason, and the
  `zones@gravel` case passes.
- [ ] **Step 4: Run the whole suite** with the package's Python 3.10 venv. Expected:
  0 failed and 8 skipped (the same reason each). Record the counts and wall time in the
  commit message.
- [ ] **Step 5: Commit** in autoware-drift on `main`. Don't push yet: it is pushed with Task 2
  after the gate passes.

### Task 2: Regression gate (autoware-drift)

**Files:**
- Create: `src/planning/race_planner/tools/regress.py`
- Create: `src/planning/race_planner/tools/regress_set.yaml`
- Create: `src/planning/race_planner/test/test_regress.py`
- Modify: `justfile` (repo root) — recipes `regress` and `regress-check`
- Modify: `README.md` — a "Before pushing to main" section

**Interfaces:**
- Produces, in `regress.py`:
  - `load_set(path) -> list[Run]`, where `Run = (key: str, argv: list[str])`. argv is
    `mode_choice_m4.py run`'s arguments.
  - `run_side(runs, rp_root: str | None, out_dir) -> dict[key, Outcome]`, where
    `Outcome = {"result", "t", "plans_digest", "traj_digest", "n_plans", "error"}`. It runs
    the harness in a subprocess with `RP_DET=1 M4_OUT=<out_dir>` and `RP_ROOT=<rp_root>` when
    given. A non-zero exit becomes `error` with the stderr's last line.
  - `compare(head: dict, base: dict) -> list[str]`. Each string is one difference naming the
    key and both values. Empty means pass.
  - CLI `regress.py check [--base origin/main] [--set FILE] [--jobs N]`. It exits 0 on pass,
    1 on a difference, 2 on a setup error (missing `$SWB` or file, unknown base).
  - CLI `regress.py run --base REV` runs one side and prints its outcomes, for inspection.
- The base side is a `git worktree add --detach <tmp>/base <rev>`, with
  `RP_ROOT=<tmp>/base/src/planning/race_planner`. Both sides use HEAD's `studies/` harness,
  so the comparison is of the race_planner package only. Results are cached under
  `~/.cache/race_planner/regress/<base sha>/` so a second check pays for HEAD only.

**The set** (`regress_set.yaml`; drift_mustang, full-size defaults; synthetic courses only, so
no swb course data is needed). Each entry gets `--digest`:

| key | argv |
|---|---|
| `asphalt_single` | `c:asphalt:25:90:8 --arm single` |
| `gravel_sched` | `c:gravel:15:90:8 --arm sched` |
| `gravel_hairpin_always` | `c:gravel:15:180:8:160 --arm always --force-drift` |
| `snow_packed_sched` | `c:snow_packed:25:90:15 --arm sched` |
| `mud_perc_est` | `c:mud_wet:25:90:8:100 --arm sched --perc --est` |
| `zones_err` | `zones --arm sched --err 10 --seed 1` |

If `zones` turns out to need course data, or takes over 5 min warm, drop it and ledger a
ruling.

- [ ] **Step 1: Write the failing tests** in `test/test_regress.py`. They are pure, with no
  solver runs:
  - `test_equal_outcomes_pass`: `compare(x, x) == []`.
  - `test_a_digest_difference_names_the_run`: changing `plans_digest` of `gravel_sched` gives
    exactly one string that contains `gravel_sched` and both digests' first 8 characters.
  - `test_one_side_raising_fails`: head has `error="Traceback…"` and base has
    `result="done"`. The result has one string containing the key, `error` and `done`.
  - `test_both_failing_alike_pass`: both sides have `result="off_road"` with equal digests,
    so the result is `[]`.
  - `test_missing_swb_is_a_setup_error`: run the CLI with `SWB=/nonexistent check`. It exits 2,
    and stderr names `/nonexistent`.
  - `test_same_rev_is_nothing_to_compare`: the CLI `check --base HEAD` on a clean tree prints
    `nothing to compare` and exits 0 within 5 s.
- [ ] **Step 2: Run them.** `... pytest -q test/test_regress.py`. Expected: FAIL (module
  missing).
- [ ] **Step 3: Implement** `regress.py`, `regress_set.yaml` and the justfile recipes:
  - `regress` runs `regress.py run --base HEAD`;
  - `regress-check base="origin/main"` runs `regress.py check --base {{base}}`.

  Both run `env -u PYTHONPATH` with the package's `.venv`. Runs go in parallel up to
  `--jobs` (default 2; each run spawns its own L1 processes). The report prints one line per
  run: key, result, t, the first 8 characters of both digests, and same/DIFF. When
  race_planner has uncommitted changes, the header says `working tree (dirty)`.
- [ ] **Step 4: Run the tests.** Expected: PASS.
- [ ] **Step 5: Prove the gate.**
  - Run `just regress-check base=HEAD~0`. Expected: `nothing to compare`.
  - On a scratch branch, change one constant on the gate's path, for example `axle.py`
    `SMOOTH` 50 → 51. Then run `just regress-check base=main`. Expected: exit 1, with
    `DIFF` on at least one run.
  - Revert the change and run the same check. Expected: exit 0, every run `same`.
  - Delete the scratch branch.

  Record the wall times, cold and warm, in the commit message.
- [ ] **Step 6: Commit; then push Tasks 1–2 to autoware-drift `main`.** Run `git pull --rebase`
  first. The README section says: before every push to `main`, run `just regress-check`
  (and `just test` for the suite). A change meant to alter results says so in its commit
  message and in the PR's description, naming the runs that moved.

### Task 3: Submodule and race setup step (AutoSDV)

**Files:**
- Modify: `.gitmodules`, add the submodule `src/planning/external/autoware-drift`
- Modify: `versions.yaml` — a new top-level `race:` block: `acados` (a release tag, the
  newest at the time of the task, older than two weeks), `python: "3.10"`. No existing key
  changes, and `export-versions.sh` does not read the block.
- Create: `setup/scripts/install-race-stack.sh`
- Modify: `setup/autosdv_setup/registry.py`, one `Step`
- Modify: `.gitignore`, add `.venv-race/`

**Interfaces:**
- Produces, for Tasks 4–6:
  - `$AUTOSDV_RACE_VENV/bin/python` (default `<repo>/.venv-race`) with race_planner
    installed editable from the submodule `[test]`, plus `acados_template`;
  - `$ACADOS_ROOT` with `lib/libacados.so`;
  - an env file `$AUTOSDV_RACE_VENV/race-env.sh` exporting `ACADOS_SOURCE_DIR`,
    `LD_LIBRARY_PATH` (acados' lib), `SWB` when it is set at install time, and
    `AUTOSDV_RACE_VENV`.
- Registry entry:
  - `id="race-stack"`, `label="Race stack (autoware-drift planner, acados)"`, `group="Libraries"`;
  - `why`: "The race planner's own Python (numpy ≥ 1.26, CasADi) and acados for the NMPC, in a
    private venv; nothing else needs them.";
  - `run=[_S("install-race-stack.sh")]`, `requires=Requires(sudo=False)`, `profiles=` none;
  - `verify`: `<venv>/bin/python -c 'import race_planner, casadi, acados_template'` and the
    presence of `$ACADOS_ROOT/lib/libacados.so`.

- [ ] **Step 1: Add the submodule** at the commit Task 2 pushed:
  `git submodule add -b main https://github.com/jerry73204/autoware-drift.git src/planning/external/autoware-drift`.
- [ ] **Step 2: Check the build is unchanged.**
  - Run `colcon list --base-paths src --names-only` before and after the submodule. Expected:
    the after-list equals the before-list plus `drift_controller` and `drift_launch`.
  - Run `just build`. Expected: success.
  - Run `just test`. Expected: the same failures as on `develop` before the change (record
    them first), plus `drift_controller`'s tests passing.
- [ ] **Step 3: Write `install-race-stack.sh`.**
  - `set -euo pipefail`, idempotent.
  - venv: `uv venv --python 3.10`, or `python3 -m venv` when uv is absent. Then
    `pip install -e <submodule>/src/planning/race_planner[test]`.
  - acados: clone at the pinned tag with submodules, then
    `cmake -DACADOS_WITH_QPOASES=OFF -DCMAKE_INSTALL_PREFIX=$ACADOS_ROOT`, `make install`, and
    `pip install -e <acados>/interfaces/acados_template`.
  - The `t_renderer` binary that acados_template fetches on first use goes into
    `$ACADOS_ROOT/bin` with a version check. On aarch64 it is built from source (Rust,
    `cargo`), because there is no prebuilt one.
  - A rerun with everything present only runs the verify checks.
- [ ] **Step 4: Run and rerun on server133.**
  `./setup.sh --run --only race-stack --yes`, twice. Expected: the first run installs; the
  second takes under 30 s and reports verified; `./setup.sh --status` shows the step as done.
- [ ] **Step 5: Run the AutoSDV checks again.** Expected: `scripts/build/check-python-env.sh`
  passes (no user-site numpy), and `just build` is unchanged.
- [ ] **Step 6: Commit and push** (AutoSDV `develop`; autoware-drift is already pushed).

### Task 4: L1/L2 timing study (autoware-drift)

**Files:**
- Create: `src/planning/race_planner/studies/bench_live.py`
- Create: `src/planning/race_planner/test/test_bench_live.py`

**Interfaces:**
- Consumes: `mode_choice_m4.run(name, arm, ..., cpus=...)` and its result JSON; `Tracker.step`.
- Produces:
  - `summarise(l1: list[float], l2: list[float], budget: float, tick: float) -> dict` with
    keys `l1_p50`, `l1_p95`, `l1_p99`, `l1_max`, `l1_overruns` (> budget), `l1_n`, `l2_p50`,
    `l2_p95`, `l2_p99`, `l2_max`, `l2_overruns` (> tick), `l2_n`;
  - `host() -> dict` with `machine`, `cpu_count`, `loadavg` (1/5/15 min), `nvpmodel`
    (`nvpmodel -q` output, or None), `jetson_clocks` (`jetson_clocks --show` first line, or
    None; both read-only), `python`, `casadi`;
  - CLI `bench_live.py run --courses c:asphalt:25:90:8,c:gravel:15:90:8,zones --arm sched --cpus LIST --label NAME --out DIR`.
    It writes `DIR/<label>.json` = `{host, label, per_course: {course: summarise(...) + {"result", "t"}}, all: summarise(over all)}`.
    It times L2 by wrapping `Tracker.step` around `time.perf_counter()`, and refuses to start
    if `RP_DET=1` is set.

- [ ] **Step 1: Write the failing tests.**
  - `test_summarise_counts_overruns`: `summarise([0.5, 1.2, 0.9], [0.001, 0.03], 1.0, 0.02)`
    has `l1_overruns == 1`, `l2_overruns == 1`, `l1_max == 1.2`, `l1_n == 3`.
  - `test_host_has_load_and_power_mode_keys`: `host()` has every key, and `nvpmodel` is None
    on x86.
  - `test_refuses_rp_det`: the CLI with `RP_DET=1` exits non-zero, and stderr names `RP_DET`.
  - `test_missing_swb_stops_before_running`: the CLI with `SWB=/nonexistent` exits 2, naming
    the path.
- [ ] **Step 2: Run them.** Expected: FAIL.
- [ ] **Step 3: Implement** `bench_live.py`.
- [ ] **Step 4: Run the tests.** Expected: PASS. Then run one real benchmark on server133:
  `bench_live.py run --courses c:gravel:15:90:8 --arm sched --cpus 8,9,10,11 --label x86-smoke --out ~/repos/AutoSDV/tmp/race-bench`.
  Expected: a JSON with `l1_n` ≥ 10, `l2_n` ≥ 400, and `l1_p95` below 1.0.
- [ ] **Step 5: Run `just regress-check`, commit, push** (autoware-drift `main`).

**Note on scale (the plan's call, ledger it):** the benchmark plans drift_mustang on
full-size courses. Under Froude scaling, the small car's window (length × 0.11 at ds × 0.11)
has the same node count, so L1 and L2 cost the same per solve, up to iteration counts. The
small car's own constants come in phase 3, which repeats one timing run with the small-car
profile to confirm.

### Task 5: First acados tracker model (autoware-drift)

**Files:**
- Create: `src/planning/race_planner/studies/acados_tracker.py`
- Create: `src/planning/race_planner/test/test_acados_tracker.py` (skipped when
  `acados_template` is not importable)

**Interfaces:**
- Consumes: `tracker._dynamics(car, grip, tyres=False) -> (f, acc, g, lbg, ubg)`, and
  `Tracker`'s horizon (1.0 s), stage dt (0.04 s) and tick (0.02 s), with its weights read from
  `Tracker`, not copied.
- Produces:
  - `build(car, n=25, dt=0.04, code_dir) -> AcadosOcpSolver`. States `(s, n, xi, vx, vy, r, delta)`
    and inputs `(delta_rate, a)` are `_dynamics`' f with `tyres=False`, as an explicit ERK model.
    The cost is NONLINEAR_LS on `n, xi, vx, vy, r` and the inputs, with references per stage.
    Hard bounds: steering lock and rate from the car. The QP solver is
    `PARTIAL_CONDENSING_HPIPM` with `SQP_RTI`.
  - `record(course, out_npz)` runs one `mode_choice_m4` run with `RP_DET=1` and saves, per
    tick, the state, the plan's reference arrays at the horizon's stations, and the
    CasADi Tracker's output (`delta_rate`, `a`, `solve_s`).
  - `bench(npz, solver) -> dict` gives the acados `time_tot` p50/p95/p99/max, the status
    counts, and |Δ| against the CasADi Tracker's first input (p95 for each input), reported
    and not gated: the models differ by the tyres (phase 4 makes them equal).
  - CLI: `acados_tracker.py record|bench --course c:gravel:15:90:8 --out DIR --label NAME`.

- [ ] **Step 1: Write the failing tests** (skipped without acados).
  - `test_straight_constant_speed_holds_still`: on a reference with κ 0, n 0 and vx 10 m/s
    everywhere, from that state, `solve()` returns status 0, |delta_rate| < 1e-3 and
    |a| < 0.5 m/s².
  - `test_offset_steers_back`: the same reference from n = +0.5 m gives a first delta_rate
    whose sign steers towards n = 0 (the same sign as the CasADi `Tracker` gives on that
    state).
- [ ] **Step 2: Run them** with the race venv and `race-env.sh` sourced. Expected: FAIL.
- [ ] **Step 3: Implement** `build`, `record` and `bench`. The generated code goes into
  `~/.cache/race_planner/acados/<hash of model + options>/`, reused when it is present.
- [ ] **Step 4: Run the tests** (expected: PASS), then on server133:
  - `acados_tracker.py record --course c:gravel:15:90:8 --out ~/repos/AutoSDV/tmp/race-bench`;
  - `acados_tracker.py bench --out ~/repos/AutoSDV/tmp/race-bench --label x86`.

  Expected: a JSON whose status counts are mostly 0. Record its `time_tot` p95.
- [ ] **Step 5: Run `just regress-check` (expected: same; no planner code changed), commit,
  push.**

### Task 6: Benchmark wrapper (AutoSDV)

**Files:**
- Create: `scripts/race/bench.sh`
- Create: `scripts/race/bench_report.py`
- Create: `scripts/race/test_bench_report.py` (pytest, system Python, no ROS)

**Interfaces:**
- Consumes: Task 3's `race-env.sh`, Task 4's `bench_live.py` JSON, Task 5's `bench` JSON,
  and `scripts/profiling/jetson_gpu_sampler.py` (on a Jetson only).
- Produces:
  - `bench.sh LABEL [--cpus LIST]`. It sources `race-env.sh`, records `host()`, starts the GPU
    sampler on a Jetson, then runs `bench_live.py run` on the three courses of Task 4 and
    `acados_tracker.py record` + `bench`. It writes everything to `tmp/race-bench/<LABEL>/`.
    It does not start any load itself: the operator brings the stack up first, and LABEL
    names that load.
  - `bench_report.py DIR... -o FILE.md` writes one Markdown table row per label: host
    (machine, nvpmodel), load (1-min average at start and end), L1 p50/p95/max and overruns,
    L2 p95/max, acados p95/max, GPU utilisation p95 (if sampled).
- [ ] **Step 1: Write the failing test.**
  `test_report_has_one_row_per_label_with_load_and_power_mode` builds two fake label dirs (one
  with an `nvpmodel` value and one without) and asserts two table rows. The row with the
  `nvpmodel` value shows it; the other shows `—`; both show the start and end loads.
- [ ] **Step 2: Run it** (`python3 -m pytest -q scripts/race/test_bench_report.py`).
  Expected: FAIL.
- [ ] **Step 3: Implement** both files.
- [ ] **Step 4: Run the test.** Expected: PASS. Then run `scripts/race/bench.sh x86-idle` on
  server133. Expected: a complete `tmp/race-bench/x86-idle/` and a one-row report.
- [ ] **Step 5: Commit and push** (AutoSDV `develop`).

### Task 7: Orin runs and the report (AutoSDV; needs a car's Orin)

This task needs hardware the planning host cannot reach: a car's AGX Orin, on the bench,
with network for setup. Before starting, the user names the car and how to reach its
Orin; until then the task is blocked and ledgered as such.

**Files:**
- Create: `docs/reports/race-stack-orin-benchmark.md`
- Modify: `docs/roadmap/16-autoware-drift/1-submodule-gate-benchmark.md` (status, outcome)
- Modify: `docs/roadmap/16-autoware-drift/README.md` (phase 1 status)

**Load configurations, in this order:**
1. `orin-idle`: nothing else running (the floor).
2. `orin-stack`: `just sim logging "vehicle_id:=<car> pose_source:=cuda_ndt"`, with the demo's
   COSS bag (the one `just demo run` fetches) replayed in a loop
   (`ros2 bag play --loop <bag> --clock`). Perception is on.
3. `orin-stack-noperc`: as 2, with `launch_perception:=false`.
4. `orin-stack-rig`: as 2, plus rig_localization on three replayed image streams, only if it
   already builds on the Orin. Otherwise the report says "not measured; phase 5".

- [ ] **Step 0: Two fixes carried from the final review** (each with a failing test first;
  autoware-drift behind `just regress-check`, then the AutoSDV pin):
  - `bench_live --budget` must scale the multistart seed budget (`e1.SEED_BUDGET`, 0.85 s)
    with it, and record `seed_budget` in the JSON. Until then a period sweep below 0.85 s
    reports overruns the configuration builds in.
  - `install-race-stack.sh` `make_venv` removes `$VENV` only if it holds `pyvenv.cfg` or is
    empty, never any directory whose python does not run.
- [ ] **Step 1: Set up the Orin.** Check out AutoSDV at the pushed `develop`. Before the
  setup step, clone scenario-world-bridge with the submodules the planner reads
  (`src/sim_vehicle_ecu`, `src/sim_vehicle_spec`, and `src/swb_isaac_world` for its
  `config/surface_classes.yaml`) and export `SWB`, so race-env.sh records it:
  ```bash
  git clone https://github.com/NEWSLabNTU/scenario-world-bridge.git ~/repos/scenario-world-bridge
  git -C ~/repos/scenario-world-bridge submodule update --init \
      src/sim_vehicle_ecu src/sim_vehicle_spec src/swb_isaac_world
  export SWB=~/repos/scenario-world-bridge
  ./setup.sh --run --only race-stack --yes
  ```
  Then run `just build`. Expected: the verify step passes and `just build` succeeds.
- [ ] **Step 2: Record the Orin's power mode.** Run `nvpmodel -q`; the clock state is
  recorded by each bench run from sysfs (`host.clocks`, the report's "Clocks pinned" column),
  since `jetson_clocks --show` needs root. Use MAXN if the car uses MAXN. The car's normal mode is what gets
  measured. Do not change the board's power mode for the benchmark.
- [ ] **Step 3: Run each load configuration.** For each, bring the load up, wait until NDT
  reports activated (`python3 scripts/testing/localization/check_ndt_activated.py`), then run
  `scripts/race/bench.sh <label> --cpus <four cores the stack uses least>`. Stop the load by
  its PGID.
- [ ] **Step 4: Write the report.**
  - Generate the tables with `bench_report.py tmp/race-bench/* -o …`.
  - State the decision the phase doc asks for:
    - the re-plan period and horizon for the small-car profile, so that L1 p95 stays under
      the period with ≥ 20 % margin under `orin-stack`;
    - whether perception must be off in a race (yes if only `orin-stack-noperc` meets that);
    - whether the acados tracker fits 50 Hz (p99 < 10 ms, half a tick).
  - If nothing fits, take the spec's fallback order: perception off, then a longer re-plan
    period and a shorter horizon. State where that leaves lap quality, as a phase 3 item.
- [ ] **Step 5: Update the phase doc and roadmap README** (status: done, the decision, a link
  to the report). Then commit and push.

## Not in this plan (phase 1's doc, deliberately deferred)

- The small car's own constants and spec: phases 2–3. Task 4's note covers why the cost
  measured now carries over.
- rig_localization on the Orin with three real cameras: phase 5. Load 4 above only if it
  already builds.
