# Phase 4: simulation regression

Run on both versions and compare:

- `just demo run` (COSS NDT replay): NDT activation, pose quality, alignment
  (`scripts/testing/localization/*`).
- `just sim planning` and `just sim logging` (CUDA NDT, MCL).
- `just sim rc`, `just sim grade` (Lab 2 Part 1), `lab3_check_lanelet` / `lab3_sim_lap` on
  the Lab 3 example map, the coach sim, `just twin` on domain 47.
- CUDA pipeline (`pointcloud_backend:=cuda`, `localization_pointcloud_backend:=cuda`).

**Exit:** every run completes on 2.1.0 with numbers within 1.5.0's (or a named, accepted
difference recorded here).
