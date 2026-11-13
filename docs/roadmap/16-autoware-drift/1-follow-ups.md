# Phase 1: follow-ups from the reviews

[← Phase 1](1-submodule-gate-benchmark.md)

These are open items from phase 1's task reviews and final review (2026-11-13). None blocks
the Orin runs, except the two under "Before the Orin runs". Fixed items are left out. "awd"
means autoware-drift, paths relative to `src/planning/race_planner`.

## Before the Orin runs (plan Task 7, Step 0)

- **`bench_live --budget` leaves the multistart seed budget at 0.85 s** (`e1.SEED_BUDGET`) under
  `--arm sched`. A period sweep below 0.85 s then reports overruns that the configuration
  itself causes. Scale the seed budget with `--budget` and record `seed_budget`.
- **`install-race-stack.sh` `make_venv` removes `$VENV` when its python does not run.** A
  mistyped `AUTOSDV_RACE_VENV` that points at a real directory would be deleted. Remove only a
  directory that holds `pyvenv.cfg` or is empty.

## To check on the Orin

- The sysfs clock reading (`bench_live.clocks`), the GPU sampler check and the arm64
  `t_renderer` download have never run on a Jetson. `nvpmodel -q` without root is unverified.
- acados prints "CasADi 3.8.2 not tested with acados" and deprecation warnings. Record the
  versions in the report.

## Regression gate (awd `tools/regress.py`)

- The cache key does not include the Python environment. After a numpy, scipy or casadi
  upgrade, a cached base can show a false DIFF. This fails safe.
- `_check_env` (`$SWB`) runs before the same-revision shortcut, so `check --base HEAD` exits 2
  without `$SWB` even when nothing would run.
- `run --base REV` runs the working tree when REV resolves to HEAD.
- The justfile strips a leading `base=` with a shell trick, so that
  `just regress-check base=X` works.
- The interrupt test sends a real SIGINT to pytest, so it could flake if pytest's SIGINT
  handling changes.
- Pyright: `dict[str, None]` inference at `regress.py` ~128–134 (annotate it), and
  `__doc__.split` fails under `python -OO`.

## Benchmarks (awd `studies/bench_live.py`, `studies/acados_tracker.py`; AutoSDV `scripts/race/`)

- `bench.sh` does not pass `--length` and `--budget` through; call `bench_live` directly to
  sweep them.
- The JSON records the requested `length`, but `live.Driver` raises it to at least the braking
  distance.
- No test covers the `Tracker.step` wrapper's call counting and restore.
- The subprocess boilerplate is duplicated in `host()`.
- `M4_OUT` takes effect only if `mode_choice_m4` has not been imported yet; add a comment.
- acados: the JSON key `casadi_solve_wall` holds the whole `Tracker.step` wall time; rename it
  `casadi_step_wall`.
- acados: `solver.cache_hit` reports that the files exist, not that they were reused.
- acados: `record` reads grip from kwargs only, and `car_box` keeps only the last Tracker
  built, so ticks mix under multistart arms.
- acados: `record` raises an opaque TypeError when `self._u` is None.
- acados: the warm start is not shifted by a tick, which biases |Δ| slightly.
- acados: the model has no delta tracking, jerk or grip rows, so phase 4 must add them.
- `bench.sh`: the end load is a 1-minute average that includes the benchmark's own load.
- `test_bench_report.py`: the GPU assertions are substring checks.
- `bench_report.py`: the nvpmodel fallback cell is the verbatim first line.
- `bench.sh`: the LD_LIBRARY_PATH warning misses spellings with a trailing slash or a symlink.

## Setup step (AutoSDV `setup/scripts/install-race-stack.sh`)

- A half-made venv whose python runs but has no pip is still reused. Probe it with
  `-m pip --version`.
- Deleting `ACADOS_SRC` while the install prefix is intact makes `install_tera` abort instead
  of re-cloning.
- The step does not check its prerequisites (git, cmake, a C compiler, curl, readelf). A
  missing `readelf` makes it rebuild on every run.
- An existing `.venv-race` is reused without checking its interpreter version.
- A verify-only rerun never rewrites an old-format `race-env.sh`, which keeps a stale
  unconditional `SWB`. Delete the file and rerun setup.

## Tests (awd `test/`)

- `test_m4_courses.py` ~38–40: the parametrize indentation is broken and the skipif line is
  over-long. Bind a `needs_haruna` mark.
- The SWB/HARUNA paths and the skip reason are repeated across five test files. A shared mark
  in `conftest.py` would remove that.
- The partial guard in `test_surfaces.py` ~142 passes silently, with no skip note.
