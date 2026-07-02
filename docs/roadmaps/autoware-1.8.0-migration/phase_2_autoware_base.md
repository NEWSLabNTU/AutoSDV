# Phase 2: Autoware Base — `rosdebian/1.8.0-1` Localrepo Debs

**Objective**: Produce the Autoware 1.8.0 / jazzy `autoware-localrepo` Debian packages from the NEWSLabNTU/autoware fork, for amd64 and JetPack 7.2, and publish them as a GitHub release consumable by `setup/scripts/install-autoware-debian.sh`.

**Status**: ⬜ Not started

**Depends on**: Phase 1 (CUDA 13 cross-build spike 1.2.4)

## Background

- AutoSDV installs Autoware as a single `autoware-localrepo_<ver>_<arch>.deb` downloaded from `https://github.com/NEWSLabNTU/autoware/releases/download/rosdebian%2F<tag>/...` (see `setup/scripts/install-autoware-debian.sh:37-48`).
- Latest existing release: `rosdebian/2025.02-1` (amd64, arm64, jetpack6.0). No 1.8.0 and no jazzy release exists yet.
- Autoware 1.8.0 upstream already dual-supports humble/jazzy (`setup-dev-env.sh --ros-distro jazzy`; ansible maps Ubuntu 24.04 → jazzy). The official ROS buildfarm carries `ros-jazzy-autoware-core`, but **universe is not on the buildfarm** — hence the localrepo deb remains the delivery mechanism.
- Autoware 1.8.0 pins (from `ansible/roles/{cuda,tensorrt}/defaults/main.yaml`): CUDA 12.8; TRT `10.8.0.43-1+cuda12.8` (amd64) / `10.3.0.26-1+cuda12.5` (aarch64, **JetPack-6-era — does not match JP7.2**).

## Work Items

### 2.1 Fork sync

- [ ] **2.1.1** Sync NEWSLabNTU/autoware fork with upstream `autowarefoundation/autoware` tag **1.8.0**; create packaging branch (e.g. `rosdebian/1.8.0`).
- [ ] **2.1.2** Rebase the rosdebian packaging infrastructure (localrepo build scripts/CI) onto the 1.8.0 tree; audit packaging commits the same way as Phase 1 forks (KEEP / OBSOLETE).
- [ ] **2.1.3** Pin `autoware.repos` at the released 1.8.0 set (`autoware_core: 1.8.0` etc.); do **not** bump to 1.9.0 in this migration.

### 2.2 AMD64 build (jazzy, CUDA 12.8 / TRT 10.8)

- [ ] **2.2.1** Build the full autoware workspace on Ubuntu 24.04 / jazzy (amd64) — natively or in `ghcr.io/autowarefoundation/autoware:universe-devel-cuda-jazzy`.
- [ ] **2.2.2** Package as `autoware-localrepo_1.8.0-1_amd64.deb`; record SHA256.

### 2.3 JetPack 7.2 build (jazzy, CUDA 13.2.1 / TRT 10.16.2)

- [ ] **2.3.1** Set up arm64 noble build container on the ARM64 server with jazzy + CUDA 13.2 toolkit + TRT 10.16 (per Phase 1 spike).
- [ ] **2.3.2** Port autoware universe CUDA/TRT packages to CUDA 13 / TRT 10.16 as needed (expected: TRT plugin API drift, removed CUDA APIs, compute-capability flags — Orin is SM_87, supported by CUDA 13). Keep all porting patches as fork commits on the `rosdebian/1.8.0` branch, marked for upstreaming.
- [ ] **2.3.3** Build the full workspace for arm64; package as `autoware-localrepo_1.8.0-1_jetpack7.2.deb`; record SHA256.
- [ ] **2.3.4** 📟 *Device-only*: Install the jetpack7.2 deb on the Orin, source it, and launch a representative CUDA node (e.g. lidar centerpoint) to verify CUDA/TRT runtime linkage. (TRT engines are generated on-device at first run — that is Phase 6.)

### 2.4 Release

- [ ] **2.4.1** Publish GitHub release `rosdebian/1.8.0-1` on NEWSLabNTU/autoware with both debs (plus generic arm64 if desired).
- [ ] **2.4.2** Record SHA256 checksums for `versions.yaml` (Phase 3).
- [ ] **2.4.3** Document the CUDA 13 porting patches (list + upstream issue links) in the release notes.

## Goal (exit criteria)

- [ ] GitHub release `rosdebian/1.8.0-1` published with `autoware-localrepo_1.8.0-1_amd64.deb` and `autoware-localrepo_1.8.0-1_jetpack7.2.deb`.
- [ ] SHA256 checksums recorded and handed to Phase 3.
- [ ] amd64 deb verified: installs on clean Ubuntu 24.04, `source /opt/autoware/...` works, planning simulator launches.
- [ ] jetpack7.2 deb verified on-device: installs on JP7.2 Orin, a CUDA/TRT node starts without linkage errors.
- [ ] CUDA 13 porting patches documented for upstreaming.
