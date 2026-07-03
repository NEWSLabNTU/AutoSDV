# Stage 3: ARM64 Build-Server Track

**Objective**: On the 80-core ARM64 server, port Autoware universe CUDA/TRT packages to CUDA 13.2 / TRT 10.16, build and release the `jetpack7.2` localrepo deb, and build the full AutoSDV workspace for arm64 — staging artifacts so Stage 4 device time is install-and-run, not build-and-debug.

**Machine**: ARM64 server (80 cores / 512 GB, Ubuntu 22.04 host). **Always build inside arm64 Ubuntu 24.04 (noble/jazzy) containers — never on the bare 22.04 host** (glibc 2.35 vs 2.39, humble-only apt).

**Status**: ⬜ Not started

**Depends on**: Stage 2A (fork packaging branch `rosdebian/1.8.0` + published release), Stage 2C/2D (migrated workspace contents for the arm64 build). No Jetson required — device verification of everything built here happens in Stage 4.

## 3A Infra unblock + CUDA-13 spike (build-half)

Carried from Stage 1 §1.2.4 (blocked there; the block moves here instead of gating the whole roadmap):

- [ ] **3A.1** Enable a container runtime for the build account — rootless UID/GID mapping (subuid/subgid) *or* docker daemon/group access. **Owner: infra admin.**
- [ ] **3A.2** Provide a host GPU driver new enough for the CUDA-13 runtime (host driver currently predates CUDA 13 — CUDA-13 binaries cannot execute even if built). **Owner: infra admin.** *(Only needed to execute CUDA-13 binaries on the server; compiling does not require it.)*
- [ ] **3A.3** (old 1.2.4 build-half) CUDA 13 cross-build spike: compile a representative CUDA/TRT sample for SM_87 inside `nvidia/cuda:13.0.0-devel-ubuntu24.04` (arm64, image confirmed available). The definitive verify-half (run on Orin, sm_87) → Stage 4A.

## 3B CUDA-13 port

- [ ] **3B.1** (old 2.3.1 + old 6.2.1) Set up the arm64 noble build container: jazzy + CUDA 13.2 toolkit + TRT 10.16 (document the Dockerfile/compose in `docker/`; verifies the ⏳ Docker edits authored in Stage 2B).
- [ ] **3B.2** (old 2.3.2) Port autoware universe CUDA/TRT packages to CUDA 13 / TRT 10.16 as needed (expected: TRT plugin API drift, removed CUDA APIs, compute-capability flags — Orin is SM_87, supported by CUDA 13). Keep all porting patches as fork commits on the `rosdebian/1.8.0` branch, marked for upstreaming. *(Risk: Autoware 1.8.0's aarch64 pin is JetPack-6-era TRT `10.3.0.26-1+cuda12.5` — no official CUDA 13 support exists; effort unquantified, see README risk register.)*

## 3C jetpack7.2 deb + release update

- [ ] **3C.1** (old 2.3.3) Build the full autoware workspace for arm64; package as `autoware-localrepo_1.8.0-1_jetpack7.2.deb`; record SHA256.
- [ ] **3C.2** (old 2.4.1 remainder) **Attach the jetpack7.2 deb to the existing `rosdebian/1.8.0-1` release** (published with amd64 in Stage 2A; plus generic arm64 if desired).
- [ ] **3C.3** (old 2.4.2) Fill the jetpack72 SHA256 placeholder in `versions.yaml` (left open by Stage 2B.6).
- [ ] **3C.4** (old 2.4.3) Document the CUDA 13 porting patches (list + upstream issue links) in the release notes.
- 📟 (old 2.3.4) On-device install + CUDA node linkage check → Stage 4B.

## 3D AutoSDV arm64 workspace build

- [ ] **3D.1** (old 6.2.2) Full AutoSDV workspace build in the container (all submodules + in-tree packages, as migrated in Stage 2C/2D).
- [ ] **3D.2** (old 6.2.3) Package/stage artifacts for the device — deb via `scripts/build/make-deb.sh` or install-tree tarball; decide and document.

## Fallback

If the server infra (3A.1/3A.2) stays blocked: CUDA package builds move on-device to Stage 4 (build directly on the Orin). Cost: hours-long on-device builds vs the 80-core server; Stage 4 then starts with building instead of installing. The rest of this stage (3D via emulation or a borrowed arm64 host) should still be attempted before accepting that cost.

## Goal (exit criteria)

- [ ] CUDA-13 build spike compiles for SM_87 in the arm64 container.
- [ ] `autoware-localrepo_1.8.0-1_jetpack7.2.deb` attached to the `rosdebian/1.8.0-1` release; SHA256 recorded in `versions.yaml` (no placeholders left).
- [ ] CUDA 13 porting patches documented for upstreaming.
- [ ] Full AutoSDV arm64 workspace builds green in the container; deploy artifacts staged and the packaging method documented.

No item in this stage requires the Jetson; everything built here is verified on-device in Stage 4.
