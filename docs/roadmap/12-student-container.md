# A container image students can run on any laptop

**Goal**: a class of 50 students, on whatever laptops they own, reaches a
running AutoSDV simulation in minutes — with no ROS 2, no Autoware and no
AutoSDV build on their own machine.

**Status**: in progress. Phase 1 (the gate) is done and passed; the amd64 image
is being built. The arm64 build has cleared four of its five unknowns and fixed
three real defects along the way, but has not yet produced an image — see
*Phase 5* for where it stopped and how to resume.

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
| 5 | arm64, built natively on the Orin | in progress -- 4 of 5 unknowns settled, 3 defects fixed; no image produced yet, see below |
| 6 | Multi-arch manifest to Docker Hub; `docker save` tarball fallback | pending |
| 7 | Logging simulation + a 4-core laptop proxy, folded into the Phase 1 report | **done — NDT holds 9.89 Hz on 4 cores; RViz is the limit, 1 fps** |
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

### Status as of 2026-09-14: four of the five unknowns settled

The prediction below was right -- (1)-(3) each surfaced a defect of exactly the
kind the amd64 build surfaced, and each was fixed in the step rather than
worked around in the Dockerfile. Six build attempts, each one getting further
than the last. The image has **not** been produced yet: the last attempt was
stopped partway through `setup.sh` because the Orin was being shut down, not
because anything failed.

| # | What the Orin run was testing | Outcome |
|---|---|---|
| 1 | Jetson repo serves TensorRT into plain `ubuntu:22.04` arm64 | **Proven.** `setup.sh`'s own `tensorrt` step does it; tier 3's aarch64 branch adds the Jetson repo and installs cleanly |
| 2 | `jetpack62` localrepo installs outside a Jetson rootfs | **Proven.** All 16 steps of `./setup.sh --run --profile dev --yes` reached "Setup complete." in 1872s |
| 3 | The workspace compiles, particularly `cuda_ndt_matcher` | **Fixed, then proven at the package level.** It did not compile; see below. `colcon build --packages-select cuda_ndt_matcher` now passes on this Orin in both CUDA and no-CUDA modes |
| 4 | TurboVNC and noVNC start | **Still unproven** -- no attempt has reached that layer |

**The three defects, and where each was fixed:**

- **The Dockerfile's own early TensorRT install was arch-broken.** It
  `apt-get install`ed `libnvinfer10` directly, which works only because the
  amd64 `nvidia/cuda` base image pre-configures NVIDIA's repo. On arm64's plain
  `ubuntu:22.04` there is no such repo yet, so the build died on
  `E: Unable to locate package libnvinfer10`. Removed entirely: `setup.sh`'s
  `tensorrt` step already installs the same three packages correctly on both
  architectures, later in the same image.
- **A stale `zed-ros2-wrapper` submodule checkout.** `zed_components` was still
  hard-failing on `find_package(ZED REQUIRED)` despite the fix being pinned,
  because the working tree sat at `458c725` while the superproject pinned
  `24e978f` ("Make the skip actually work: find ament_cmake before calling
  ament_package"). `git submodule update` was the whole fix. Worth knowing: the
  symptom looks identical to the bug being unfixed.
- **`cuda_ffi` hard-panicked with no CUDA toolkit** -- "CUDA installation not
  found" -- and `cuda_ndt_matcher` took eleven other packages down with it.
  This was the last package in the workspace that had not learned to skip
  itself, and Cargo has no equivalent of the CMake early-return the other three
  use, so the fix was to make the crate **compile and link** with no CUDA
  present: `build.rs` emits a `cuda_ffi_stub` cfg instead of panicking, and
  every public item that reaches a compiled `.cu` kernel has a stub twin
  returning `CudaError::NoToolkit`. `NEWSLabNTU/cuda_ndt_matcher` `326c303`,
  verified both ways on this Orin (real mode unchanged, stub mode clean).

**A fourth fix, not a defect in the container but in how it downloads.** The
Autoware deb is ~2 GB and the release host throttles per connection, so the
single-stream `wget` fallback in `install-autoware-debian.sh` is an order of
magnitude slower than the `aria2c` path the script prefers -- measured here,
383 KB/s against 2905 KB/s. A clean machine has no `aria2c`, so that fast path
was one nobody ever took: one attempt crawled at 182 KB/s for an hour. `aria2`
is now in the image's base tools, the step installs it when missing, and every
fallback warns loudly instead of degrading in silence.

**Watch the submodule pin.** It was rewound once mid-session by an unrelated
commit made from a working tree still checked out at the older submodule
commit, which silently removed the `cuda_ffi` fix from `develop` and would have
reproduced defect (3) on the next build. `git submodule status --recursive |
grep '^+'` before committing is what catches this.

### To resume

```bash
cd ~/AutoSDV && git pull                     # 4cce295 or later
PLATFORM=linux/arm64 TAG=jerry73204/autosdv:desktop-arm64 ./docker/desktop/build.sh
```

Expect `setup.sh` (step 6 of 12) to dominate, but with `aria2c` engaged the deb
should take minutes rather than hours -- confirm by grepping the log for
`Downloading with aria2c (parallel, 10 connections)` and NOT the
`WARNING: aria2c is not installed` path. Past that, the remaining unknown is
(4), the graphics layer.

One structural note for whoever iterates next: `COPY . ${AUTOSDV_HOME}` sits
**before** `RUN ./setup.sh`, so any source change invalidates the entire ROS 2 +
Autoware install layer and re-downloads the deb. That is why each of these six
attempts cost 35 minutes at best. Moving the COPY after the system-dependency
install, or splitting the system install from the workspace build, would make
every future iteration dramatically cheaper.

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
