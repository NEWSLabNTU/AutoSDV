# Phase 5: on the cars

- A jetpack62 build of the 2.1.0 localrepo; install beside 1.5.0 on one Orin outside lab
  slots (cars B and C carry Lab 3 until 1 Dec).
- Engines on the Orin; `cuda_ndt` timing against 1.5.0 under load.
- Car A or B: sensors, localization on the COSS map, a supervised autonomous lap.

**Exit:** a lap on a real car on 2.1.0, and the car switched back to 1.5.0 in one command.

## Runbook (prepared 2026-11-18, not yet run)

Blocked on access, not on code: the cars carry Lab 3 until 1 Dec, they are not reachable
from the workstation, and installing the Debian needs sudo on the Orin. The jetpack62
build exists (`autoware-localrepo` release 2.1.0-1, checksum in `versions.yaml`).

On the Orin, branch `feature/autoware-2.1.0`, in one sitting:

```bash
cat /etc/nv_tegra_release                      # L4T 36.5 (JetPack 6.2.2+)
git fetch && git checkout feature/autoware-2.1.0 && git submodule update --init --recursive
AUTOSDV_AUTOWARE_VERSION=2.1.0 ./setup.sh --run --only autoware-debian --yes   # beside 1.5.0
echo 2.1.0 > .autoware-version                 # this checkout now selects 2.1.0
just build                                     # into build-2.1.0/ install-2.1.0/
just setup-autoware-data && just build-engines # engines on the board; second run reuses
just demo run                                  # cuda_ndt timing vs the 1.5.0 numbers
just launch vehicle_id:=car_b                  # sensors, NDT on COSS, supervised lap
rm .autoware-version                           # back to 1.5.0: the default, its own build/
```

Things to watch, each fixed on amd64 and not yet seen on the Orin:
- engine reuse: the arm64 TensorRT ABI is JetPack's by construction, so it should hold;
- `pointcloud_backend:=cuda` and `localization_pointcloud_backend:=cuda`, never replayed
  on 2.1.0 with the sensing half (cuda_blackboard 0.5);
- the ZED wrapper and Nebula 1.2 against the real sensors.
