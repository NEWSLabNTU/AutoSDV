# A container image students can run on any laptop

**Goal**: a class of 50 students, on whatever laptops they own, reaches a
running AutoSDV simulation in minutes — with no ROS 2, no Autoware and no
AutoSDV build on their own machine.

**Status**: in progress. Phase 1 (the gate) is done and passed; the amd64 image
is being built.

**Feeds**: [roadmap 9](9-workshop-laptop-onboarding.md), whose two-hour timetable
this changes — see *Consequences* below.

---

## Why a container rather than instructions

Roadmap 9 assumed every student installs AutoSDV natively during class, and
listed "a prepared VM image" as an undecided fallback. The room makes that
assumption unworkable: it contains Windows, macOS and several Linux flavours,
and Apple Silicon cannot run Autoware natively at all.

A container collapses that to one artefact and one command.

## What was established before building anything

Each of these was measured, not assumed. Together they are why the design looks
the way it does.

| Question | Answer | Evidence |
|---|---|---|
| Does AutoSDV need an NVIDIA driver? | **No.** 0 of 840 amd64 and 0 of 1053 arm64 Autoware libraries link `libcuda.so.1` | `readelf -d` over every shipped `.so` |
| Then what does it need? | CUDA *runtime* and TensorRT, and only for perception, which the workshop does not launch | 6 amd64 / 9 arm64 libs link `libcudart` |
| Can Apple Silicon run this natively? | **Yes.** JetPack's arm64 CUDA debs install on plain `ubuntu:22.04` arm64, pulling no Tegra dependency | probe against `repo.download.nvidia.com/jetson r36.4` |
| Is the ZED SDK required? | Only by `zed_components`. `zed_wrapper`, which the tutorial needs for TF, builds without it | `find_package(ZED REQUIRED)` in one CMakeLists |
| Is software rendering fast enough? | **Yes, with the right RViz config**: 31 fps, versus 2 fps with the stock one | [gpu-less-simulation-and-rviz.md](../reports/gpu-less-simulation-and-rviz.md) |

The last one is the load-bearing result. macOS can never be accelerated —
Hypervisor.framework exposes no vGPU — so the unaccelerated floor had to be
good enough on its own, and it is.

## Shape

One multi-arch manifest, so students never choose a variant:

```
docker.io/jerry73204/autosdv:desktop
   ├── linux/amd64   Windows, Intel Mac, Linux
   └── linux/arm64   Apple Silicon
```

**The two are different platforms sharing a Dockerfile, not one image built
twice.** The arm64 side is a *Jetson-flavoured* image: Autoware comes from the
`jetpack62` localrepo, because that is the only arm64 Autoware build that
exists, and its CUDA and TensorRT therefore come from the Jetson repository to
match. It runs off a Jetson because JetPack 6.2 is Ubuntu 22.04 underneath and
Apple Silicon is arm64.

| | amd64 | arm64 |
|---|---|---|
| Target | Windows, Intel Mac, Linux | Apple Silicon |
| Built on | any x86 machine | **the Orin** |
| Base | `nvidia/cuda:12.8.1-cudnn-devel-ubuntu22.04` | `ubuntu:22.04` |
| Autoware | `1.5.0-2ubuntu2204` | `1.5.0-2jetpack62` |
| CUDA / TensorRT | NVIDIA x86 CUDA repo | Jetson repo (`r36.4`) |

Not `nvidia/cuda`'s arm64 tag: that is SBSA, the server-ARM platform, and
pairing SBSA CUDA with JetPack Autoware packages mixes two platforms in one
image. **arm64 hosts other than the Orin are not a supported platform** -- this
exists so an Apple Silicon laptop can run the Jetson container natively instead
of through emulation.

Graphics are decided by the entrypoint, not by the instructions: the container
always runs its own X server (TurboVNC) reached over noVNC in a browser, and
picks the fastest renderer the host actually exposes.

| Host | GPU reachable in a container? | Path |
|---|---|---|
| Linux + NVIDIA | yes | VirtualGL |
| Linux + Intel/AMD | yes | VirtualGL on `/dev/dri` |
| Windows (WSL2) | yes | Mesa **d3d12** via `/dev/dxg` |
| **macOS** | **never** | software (llvmpipe) |

The workspace ships **prebuilt**. A student reaches a running simulation in
minutes rather than watching colcon for forty, and a laptop that would run out
of memory while linking never has to.

## Phases

| # | Phase | State |
|---|---|---|
| 0 | Repo prep: split `docker/` into `jetson/` and `desktop/`; Autoware `1.5.0-2`; `workshop.rviz` | done |
| 1 | **Gate:** GPU-less verification and RViz measurement | **done — passed** |
| 2 | amd64 image, headless | in progress |
| 3 | Graphics layer: TurboVNC + noVNC + renderer detection | in progress |
| 4 | Accelerated profiles (`nvidia`, `dri`, `wsl`) | pending |
| 5 | arm64, built natively on the Orin | **next -- see below** |
| 6 | Multi-arch manifest to Docker Hub; `docker save` tarball fallback | pending |
| 7 | Logging simulation + a 4-core laptop proxy, folded into the Phase 1 report | pending |
| 8 | Book page, EN + zh-TW | pending |
| 9 | Rewrite roadmap 9's timetable | pending |

## Consequences for roadmap 9

Its timetable is built around a 30-minute "ROS 2 concepts while it installs"
block. **That block no longer has an install to cover.** A `docker pull` done at
home the night before, plus `docker run`, is about five minutes.

This is a gain — the install was the session's biggest risk — but the two hours
have to be re-planned rather than trimmed. The ROS 2 material is still worth
teaching; it just needs to stand on its own rather than fill dead time.

## Phase 5: building the arm64 image on the Orin

The Orin is the **builder**, not the target. The image it produces runs on
Apple Silicon and is never run on the Jetson itself. Build it there because
arm64 under qemu on an x86 host compiles the whole workspace through emulation
and takes hours; `build.sh` warns when it detects that.

```bash
# on the Orin
cd ~/AutoSDV && git pull
PLATFORM=linux/arm64 TAG=jerry73204/autosdv:desktop-arm64 ./docker/desktop/build.sh
```

Everything the build needs is already wired:

- `versions.yaml` carries `container.desktop_base_arm64`, and `build.sh`
  selects it from `PLATFORM`
- `install-autoware-debian.sh` already picks the `jetpack62` deb on `aarch64`
- `install-tensorrt.sh` has an arm64 branch that installs from the Jetson
  repository (`JETSON_REPO` overrides the `r36.4` default)
- `blickfeld_driver` and `zed_components` now skip themselves when their SDKs
  are absent, so the build does not need `COLCON_IGNORE`

**What has not been proven**, and what the Orin run is actually testing:

1. that the Jetson repository serves TensorRT into a **plain** `ubuntu:22.04`
   arm64 container -- the apt resolution was probed and works, but no image has
   been built on it
2. that the `jetpack62` Autoware localrepo installs there, outside a Jetson
   root filesystem
3. that the workspace compiles, particularly `cuda_ndt_matcher`, whose
   `cuda_ffi/build.rs` needs `nvcc` and the CUB headers -- on arm64 those come
   from the Jetson CUDA packages rather than an NVIDIA CUDA base image
4. that TurboVNC and noVNC start

Expect (1)-(3) to surface defects of the same kind the amd64 build surfaced:
packages assuming something the host image happened to provide. When one does,
**fix the setup step, not the Dockerfile** -- see the note on workarounds in
`docker/desktop/README.md`.

Finally, publish one manifest over both tags so students never pick a variant:

```bash
docker manifest create jerry73204/autosdv:desktop \
    jerry73204/autosdv:desktop-amd64 jerry73204/autosdv:desktop-arm64
docker manifest push jerry73204/autosdv:desktop
```

## Known gaps

- **The WSL `/dev/dxg` path is unverified.** Written from Microsoft's documented
  requirements; nobody here has a Windows machine to run it on. It ships as a
  documented starting point, not a tested path.
- **`net.core.rmem_max` is not namespaced.** It must be set on the host or
  passed as `--sysctl`; below ~10 MB no ROS 2 node starts at all, with an error
  that mentions nothing about buffers. The entrypoint checks and says so.
- **Every measurement so far is from a 32-thread workstation**, which is four to
  eight times a student laptop. No hardware requirement should be published
  until the 4-core proxy runs.
