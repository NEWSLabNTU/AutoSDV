# Phase 1: versioned build

**Status:** done (2026-11-17), on the workstation.

## How a checkout selects its Autoware

- `versions.yaml` `autoware.versions.<v>`: install prefix, localrepo files and sha256,
  `workspace_suffix`, `data_path`, `models` (farm or download), `tensorrt_engine_abi`.
  `autoware.default` is 1.5.0.
- The selection, first hit wins: `AUTOSDV_AUTOWARE_VERSION` (this shell),
  `.autoware-version` (this checkout, gitignored), `autoware.default`
  (`scripts/version/autoware-version.sh`; an unknown version is an error naming the known
  ones). `get-version.sh autoware.<key>` reads the selected entry, so existing callers
  (`autoware.install_prefix`, `autoware.version`) follow it.
- `.envrc` and `scripts/env.sh` source `scripts/version/select-autoware.sh`: they refuse a
  shell whose `AUTOWARE_VERSION` (exported by every Autoware `setup.bash`) names another
  version, then source `<prefix>/setup.bash` and the workspace's `local_setup.bash`.
  Exports `AUTOSDV_AUTOWARE_VERSION`, `AUTOSDV_AUTOWARE_PREFIX`, `AUTOSDV_WS_INSTALL`,
  `AUTOSDV_DATA_PATH`.
- `just build|test|clean`: `build<suffix>`, `install<suffix>`, `log<suffix>`;
  `scripts/build/check-autoware-env.sh` refuses a shell carrying another version.
- `just setup-autoware-data`: 1.5.0 links `<prefix>/data` (never a glob over
  `/opt/autoware/*`); 2.1.0 runs `autoware-download-models-2-1-0 --dir data/autoware_data-2.1.0`.
- The installer and the setup steps read the selected entry (`AUTOSDV_AUTOWARE_VERSION=2.1.0
  ./setup.sh --only autoware-debian` installs 2.1.0).

## Rulings

- **1.5.0 keeps `build/`, `install/`, `log/`;** other versions get a suffix. Nothing built
  today rebuilds, and every script that still says `install/` keeps working on 1.5.0.
  Cost if wrong: one rename when 2.1.0 becomes the default (phase 6).
- **The selector is `AUTOSDV_AUTOWARE_VERSION`, not `AUTOWARE_VERSION`:** Autoware's own
  setup.bash exports the latter, so a `~/.bashrc` line would select silently; it is
  compared instead.
- **1.5.0's entry moves to localrepo 1.5.0-3,** the first that co-installs with 2.1.0 (this
  workstation runs it). The cars are unchanged until someone re-runs the step there.
- **TensorRT ABI per version;** 2.1.0's is unmeasured, so `trt-runtime-env.sh` leaves the
  host's TensorRT for it (phase 3 measures it).
- **cuda_pointcloud_filters:** our pin 1189ca8 (the MPI C-language fix) sat on
  `fix/enable-mpi-c-language`, never merged; cherry-picked onto `main` after d8be382 (the
  cuda_blackboard 0.5 stream) as 6a5687b. cuda_ndt_matcher to 083d587 (adds b0bc3c6, the CPU
  Hessian sign fix; the rest is its standalone scripts).

## Still hard-coded (left for later phases)

- `docker/desktop/*`: the image is built on 1.5.0.
- `scripts/testing/lab2/*`: Lab 2 runs on 1.5.0 from its release tags.
- `scripts/2dlidar/*` (14 files): research scripts.
- `setup/scripts/install-tensorrt*.sh`: install `nvidia_amd64.tensorrt_engine_abi` (1.5.0's).

**Exit:** `just build` and `just test` pass for 1.5.0 and 2.1.0 from one checkout, each in
its own workspace; switching versions needs no rebuild of the other.

## Result (2026-11-17, workstation, play_launch 0.38.0)

- `just build`: 48 packages on each version (1.5.0 in `build/`, 2 min 6 s incremental;
  2.1.0 in `build-2.1.0/`, 1 min 3 s). The 2.1.0 workspace resolves
  `cuda_blackboard_DIR` and `libcuda_blackboard.so` from `/opt/autoware/2.1.0` and holds no
  1.5.0 path.
- `just test`, 1748 tests each: 1.5.0 347 failures, 2.1.0 348. The 347 are the same files
  on both (lint and copyright in `autosdv_system_monitor`, `autoware_manual_control`,
  `autoware_zed_converter`, the ZED packages; `autosdv_vehicle_interface` 2,
  `particle_filter` 3) and none is in a package this phase changed: the baseline.
  `cuda_ndt_matcher`'s 3 are this workstation's foreign CUDA MPS daemon; with a private
  `CUDA_MPS_PIPE_DIRECTORY` it passes 91/91.
- The one 2.1.0-only failure, `autosdv_rc_sim` `test_copies_match_upstream_but_for_the_timeouts`:
  `rc_sim_diagnostics.yaml` is a copy of 1.5.0's `autoware-main.yaml`, which 2.1.0 changed.
  Phase 2.
