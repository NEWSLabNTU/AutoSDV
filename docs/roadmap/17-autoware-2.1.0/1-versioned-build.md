# Phase 1: versioned build

- `versions.yaml`: one `autoware` entry per version (install prefix, localrepo files,
  checksums) and a default; `export-versions.sh` exports the selected one
  (`AUTOSDV_AUTOWARE_VERSION`, default 1.5.0). Every key it reads exists for both.
- The environment (`.envrc`, `scripts/env.sh`) picks the version first, sources only
  `/opt/autoware/<ver>/setup.bash`, then the workspace's `install-<ver>/local_setup.bash`
  (not `setup.bash`: it replays the underlay baked in at build time).
- `just build`: `--build-base build-<ver> --install-base install-<ver>`; `just clean`
  per version. `data/autoware_data` becomes per version (`just setup-autoware-data`).
- Submodules: `cuda_pointcloud_filters` to d8be382 or later (picks the
  `cuda_blackboard` 0.1 / 0.5 API by `cuda_blackboard_VERSION`); `cuda_ndt_matcher` to
  083d587 or later (version selection; b0bc3c6, the CPU Hessian sign fix).
- Our fork `src/localization/tier4_localization_launch` builds on 1.5.0 only, or is
  replaced in phase 2.

**Exit:** `just build` and `just test` pass for 1.5.0 and 2.1.0 from one checkout, each in
its own workspace; switching versions needs no rebuild of the other.
