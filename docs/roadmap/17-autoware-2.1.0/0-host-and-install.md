# Phase 0: host and install

**Status:** done on the workstation, 2026-11-17, except the 1.5.0 library breakage below.

- `autoware-full-1-5-0` 1.5.0-2 and `autoware-full-2-1-0` 2.1.0-1 installed side by side
  (needs `autoware-localrepo-1-5-0` 1.5.0-3 or later; 1.5.0-1 ships unversioned files and
  cannot co-install).
- `/opt/autoware/2.1.0/share`: 499 packages against 1.5.0's 455; 40 removed, 84 added
  (lists: `tmp/aw21/removed.txt`, `added.txt` when regenerated with `comm` over the two
  `share/` listings).

## Open

- **1.5.0 broken by the ROS upgrade.** After `apt install --only-upgrade 'ros-humble-*'`
  (`diagnostic_updater` 4.0.7), four 1.5.0 libraries do not load: `undefined symbol:
  _ZTVN18diagnostic_updater7UpdaterE` in `libautoware_lidar_centerpoint_component.so`,
  `libconcatenate_data.so`, `libcuda_pointcloud_preprocessor{,_lib}.so`. A rebuilt
  1.5.0-humble-4 (amd64) is announced; until then pin `ros-humble-diagnostic-updater` back.
  Check the cars before any `ros-humble-*` upgrade there.
- **Models:** 2.1.0's are not in the Debian. `autoware-download-models-2-1-0` (per user,
  about 2.4 GB) or the `data-2.1.0-1` release; one data folder per version.
- **play_launch >= 0.37.0** (mock substitutions; 2.1.0's launch files parse). Until #0095
  is fixed, `--parser python` for 2.1.0 with perception on (the Rust parser drops
  `<autoware_node>`, three perception nodes).
- `autoware-env.bash` sets a loopback-only `CYCLONEDDS_URI`: source it before our DDS
  profile, never after.
