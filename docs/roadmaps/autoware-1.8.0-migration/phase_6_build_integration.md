# Phase 6: Build & Integration Across Environments

**Objective**: Green full-workspace builds on all three environments; TensorRT engines generated on-device; system launches end-to-end.

**Status**: ⬜ Not started

**Depends on**: Phases 3–5

## Build Matrix

| Environment               | What builds here              | How                                                                                      |
|---------------------------|-------------------------------|------------------------------------------------------------------------------------------|
| AMD64 (Ubuntu 24.04)      | Everything                    | Native: `./setup.sh` → `make checkout` → `make build`                                    |
| ARM64 server (22.04 host) | All arm64 packages incl. CUDA | arm64 noble/jazzy container + CUDA 13.2 toolkit + TRT 10.16; artifacts staged for device |
| Jetson Orin (JP7.2)       | Final install; TRT engines    | `./setup.sh` on device; engines generated at first launch (SM_87)                        |

## Work Items

### 6.1 AMD64

- [ ] **6.1.1** Clean Ubuntu 24.04 machine: `./setup.sh` (jazzy + localrepo 1.8.0 + ZED SDK 5.4 + blickfeld lib), `make checkout`, `make build` — zero errors.
- [ ] **6.1.2** `make test` green.
- [ ] **6.1.3** `make launch` smoke test with `logging_simulation.launch.yaml` (no hardware).

### 6.2 ARM64 server

- [ ] **6.2.1** Stand up the build container: arm64 noble + jazzy + CUDA 13.2 + TRT 10.16 (document the Dockerfile/compose in `docker/`).
- [ ] **6.2.2** Full workspace build in the container (all submodules + in-tree packages).
- [ ] **6.2.3** Package/stage artifacts for the device (deb via `scripts/build/make-deb.sh` or install-tree tarball — decide and document).

### 6.3 Jetson Orin (📟 device-only)

- [ ] **6.3.1** `./setup.sh` on JP7.2 Orin: r39 detection path installs `jetpack7.2` localrepo deb; ZED SDK 5.4 Jetson variant; blickfeld lib.
- [ ] **6.3.2** Deploy AutoSDV workspace (server-built artifacts or on-device `make build` — record build time for both; the 80-core server should win).
- [ ] **6.3.3** First `make launch`: TensorRT engine generation for perception models (SM_87) completes; cache engines.
- [ ] **6.3.4** Verify CycloneDDS config (`cyclonedds.xml`) + sysctl tuning still applies on kernel 6.8.

## Goal (exit criteria)

- [ ] `make build` + `make test` green on AMD64 native and in the ARM64 server container.
- [ ] 📟 On-device: `make launch` reaches a running Autoware stack on the Orin (nodes up, no crash loops), TRT engines built and cached.
- [ ] Build/deploy procedure for each environment documented (README or `docs/`).
