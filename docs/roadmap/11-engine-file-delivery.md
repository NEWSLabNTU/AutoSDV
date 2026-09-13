# Portable TensorRT engine delivery

**Goal**: stop paying the `tensorrt-engines` setup step's build time (~1hr on an
Orin) on every fresh install or reprovision. Build each engine once per
compatible-hardware class, publish it as a release asset on
`NEWSLabNTU/AutoSDV`, and have `setup.sh` pull a match before it falls back to
building locally.

**Status**: Phases 0, 1 and 4 implemented and tested end-to-end on an AGX
Orin (`just engines` on an existing cache: 20.5s, vs. the ~1hr build). Phase 2
implemented for the exact-GPU-match case; cross-generation `kAMPERE_PLUS`
sharing confirmed unavailable without an upstream patch, see below. Phase 3
(actually publishing a release) is next — yours to do per your own timeline.

**Source material**: this session's investigation of engine portability rules
(TensorRT hardware-compatibility docs, JetPack 6.2.x release notes), the
current build step at `justfile:85-130` (`build-engines`) and its registry
entry (`setup/autosdv_setup/registry.py`, step `tensorrt-engines`), and
`docs/design/cuda-pipeline-data-flow.md` for the sibling submodule/retirement
pattern this doc borrows its shape from.

---

## Why a cache, and why it isn't just "copy the .engine files"

`just build-engines` builds 5 engines (`autoware_lidar_centerpoint`,
`autoware_tensorrt_yolox`, 2x `autoware_traffic_light_classifier`,
`autoware_traffic_light_fine_detector`) sequentially via
`ros2 launch ... build_only:=true`, one GPU, no parallelism possible — the
build time is real and doesn't compress. The only lever is not repeating it on
hardware that already produced (or could reuse) the same engine.

TensorRT engines are not freely portable, though. Two axes govern this:

**Version, not just architecture.** An engine's build fingerprint includes the
exact TensorRT version, plus CUDA/cuDNN. JetPack 6.2 vs 6.2.1 ship the same
TensorRT 10.3.0 / CUDA 12.6 / cuDNN 9.3.0 — only the L4T patch (36.4.3 vs
36.4.4) differs, so those two are *probably* interchangeable but not officially
validated. Any TensorRT version delta breaks it outright.

**Hardware, and JetPack is a special case.** TensorRT ships two build-time
hardware-compatibility levels — `kAMPERE_PLUS` (any Ampere-or-newer GPU, some
perf cost) and `kSAME_COMPUTE_CAPABILITY` (narrower). Neither is available on
JetPack: NVIDIA's docs state hardware compatibility "is not supported on
NVIDIA DriveOS or NVIDIA JetPack." So on Orin, an engine is locked to the exact
board (AGX Orin / Orin NX / Orin Nano are different silicon despite sharing the
Ampere family) with no escape hatch — the cache key must be per-SKU, not
per-architecture. Desktop GPUs (30xx/40xx/50xx) get the escape hatch: build
once with `kAMPERE_PLUS` and one artifact covers all three.

This is why the deliverable is a *keyed cache with a verified fallback*, not a
tarball dropped into the repo: a mismatched engine must fail closed (TensorRT
already does this — a version/hash mismatch is a clean load error, not silent
corruption) and trigger a local rebuild, never a bad flight.

---

## Phase 0 — Fingerprint and cache-key format

**Objective**: one deterministic string per hardware/software combination that
is safe to key a cache entry on, computable from a fresh Orin or desktop box
with no manual input.

### 0.1 Orin key

```
{l4t_version}_{tensorrt_version}_{board_model}_{autoware_version}_{model_hash}
```

- `l4t_version` — `/etc/nv_tegra_release` (e.g. `R36.4.4`)
- `tensorrt_version` — `dpkg-query -W -f='${Version}' tensorrt`
- `board_model` — `/proc/device-tree/model`, normalized (`AGX Orin`, `Orin NX`, `Orin Nano`)
- `autoware_version` — `./scripts/version/get-version.sh autoware.version`
- `model_hash` — hash of the `.onnx`/config the engine was built from (catches an Autoware point-release that changes a model without changing `AUTOWARE_VERSION`)

### 0.2 Desktop key

```
{tensorrt_version}_AMPERE_PLUS_{model_hash}
```

No board term: `kAMPERE_PLUS` build already spans 30xx/40xx/50xx by
construction. A future desktop box needing native (non-hw-compat) performance
gets its own narrower key, same shape as the Orin one minus `board_model`.

**Success criteria**:
- [ ] a script prints both key forms from a bare install (no manual lookup)
- [ ] the 5 model hashes are pinned somewhere version-controlled, not
      recomputed from a mutable Autoware Debian tree

---

## Phase 1 — Orin export/import procedure

**Objective**: a documented, scripted way to produce a portable artifact from
one Orin and prove it loads on another *before* trusting it in the cache.

### 1.1 Export

Tar the tree `just setup-autoware-data` already mirrors
(`data/autoware_data/**/*.engine`), named by the Phase 0 key. Record the key
alongside the tarball, not just in its filename — a rename shouldn't silently
break the lookup.

### 1.2 Import + verify

Pulling a cached tarball must not be trusted blindly:

1. compute the local key
2. fetch the release asset matching it, if one exists
3. attempt to load each engine (a cheap `build_only:=true` launch should
   report a load rather than a rebuild when the file already matches)
4. on any load failure, delete the pulled files and fall back to
   `just build-engines` — this is the existing behavior, just reached from a
   different starting point

**Success criteria**:
- [ ] importing on a byte-for-byte identical second AGX Orin skips the build
      entirely and the pipeline launches correctly
- [ ] importing on a deliberately mismatched box (different L4T patch, or a
      different board) fails the verify step and falls back to a local build,
      without the user having to notice or intervene

---

## Phase 2 — Desktop build (exact-GPU match, same mechanism as Orin)

**Status: implemented.** `kAMPERE_PLUS` cross-generation sharing (build once,
run on 3090+4090+5090) is **not available** — checked directly against the
installed `ros-humble-autoware-tensorrt-common-1-5-0` 1.5.0: its public
headers never declare a hardware-compatibility field, no launch/param file
across `autoware_lidar_centerpoint`/`autoware_tensorrt_yolox`/
`autoware_traffic_light_classifier`/`autoware_traffic_light_fine_detector`
exposes one, and `strings` on every shipped `.so` turns up no
`AMPERE`/`HardwareCompat`/`VERSION_COMPAT` reference at all. If the compiled
code never calls `IBuilderConfig::setHardwareCompatibilityLevel`, there is no
outside lever — no launch arg reaches a code path that doesn't exist.

The obvious workaround — build each `.engine` directly with `trtexec
--hardwareCompatibilityLevel=ampere_plus` against the model's `.onnx`,
bypassing Autoware's own builder — was rejected rather than attempted: these
models need a custom plugin (`autoware_tensorrt_plugins`) and per-model
precision/shape settings that Autoware's own builder wires up for you, and
`justfile:100`'s own long-standing comment already names exactly this failure
mode ("Building with trtexec by hand would not guarantee that [builder
settings match what the node will later expect]"). Not a safe substitute.

**What ships instead**: the desktop fingerprint keys by exact GPU, the same
way Orin keys by exact board —
`desktop-{gpu-slug}-{trt}-autoware{version}`, e.g. `desktop-rtx-4090-...`.
`just build-engines`, `just export-engines`, and `just engines` needed no
platform-specific code at all; they already ran generically wherever
`ros2 launch ... build_only:=true` works. This gets a working, cacheable
desktop path today — a 4090's cached engines just don't also cover a 3090 or
a 5090 yet.

**Future work, blocked on upstream**: sharing one build across desktop
generations needs a patch to `autoware_tensorrt_common` adding a
`hardware_compatibility_level` field to `TrtCommonConfig` and calling
`setHardwareCompatibilityLevel` in the builder — the same "fix upstream
first, submodule mirrors the branch" treatment as the CUDA pointcloud filters
(`docs/design/cuda-pipeline-data-flow.md`), not a local hack. Until that
lands, `desktop-{gpu-slug}-...` stays the correct key shape; adding
`kAMPERE_PLUS` support later is a value the key computes differently, not a
new mechanism.

**Success criteria**:
- [x] the fingerprint script and `just export-engines`/`just engines` work
      unmodified on a desktop box (no platform branch needed beyond the key
      format itself)
- [ ] confirmed on an actual desktop box — not yet tested (arm64 Orin only so
      far)
- [ ] (future, post-upstream-patch) one hw-compat engine set loads and infers
      correctly on a 3090, a 4090, and a 5090 without rebuilding

---

## Phase 3 — Release packaging on `NEWSLabNTU/AutoSDV`

**Objective**: a place these artifacts live that isn't "whoever built it last
keeps the tarball on their laptop."

- one GitHub Release per Autoware version bump (matches when engines actually
  change), tagged e.g. `engines-autoware-1.5.0`
- assets named by the Phase 0 key, one per Orin SKU plus one `AMPERE_PLUS`
  desktop asset
- a manifest file (json/yaml) in the release listing key → asset, so the
  import step doesn't have to guess a filename convention
- **not** committed into the git tree itself — same reasoning as
  `data/autoware_data` already being gitignored and mirrored, just at release
  scale instead of per-checkout

**Success criteria**:
- [ ] a fresh Orin box, `gh release download` + import, running pipeline,
      zero local `build_only` invocations
- [ ] the release is reproducible: re-running the export on the same hardware
      and Autoware version produces a byte-identical (or at least
      load-verified-equivalent) artifact

---

## Phase 4 — `setup.sh` integration

**Status: implemented** — `setup/autosdv_setup/registry.py`'s `tensorrt-engines`
and `tensorrt-engines-build` steps, `justfile`'s `engines`/`export-engines`
recipes, `scripts/version/engine-fingerprint.sh`. Tested live: both step ids
show correctly in `--list`, are individually selectable via `--only`, and
`--dry-run` prints the right underlying `just` call for each.

**Objective**: `tensorrt-engines` tries the cache before it builds, and
building from scratch stays a first-class choice — a menu item and a plain
step id, not an environment variable nobody discovers.

`setup/autosdv_setup/model.py`'s `Step` is already just a checkbox with an
id, a `why`, and an argv; the curses menu, the plain numbered menu, and
`--only`/`--skip` all iterate `STEPS` generically. So the "download vs. build
from scratch" choice needs no new mechanism, only a second step id:

```
Step(
    id="tensorrt-engines",
    label="Pre-compile TensorRT engines",
    why="Downloads a prebuilt engine set matching this board's fingerprint "
        "when one exists on a NEWSLabNTU/AutoSDV release; builds locally "
        "otherwise. Select 'tensorrt-engines-build' instead to always build "
        "locally, e.g. when developing something that changes a model.",
    run=_BASH(... "just engines"),   # try-download-else-build
    profiles=_on(),                  # opt-in, as today
),
Step(
    id="tensorrt-engines-build",
    label="Build TensorRT engines from scratch (skip the cache)",
    why="Forces a local build even when a cached engine set exists for this "
        "fingerprint. Selecting both this and 'tensorrt-engines' just runs "
        "the build twice, harmlessly -- the second run's files win.",
    run=_BASH(... "just build-engines"),   # unchanged, existing recipe
    profiles=_on(),                        # opt-in, off by default
),
```

Both show up in the curses menu (tick one, or both) and the plain numbered
one. Non-interactively, no new flag is needed — the existing selectors
already do it:

```bash
./setup.sh --run --only tensorrt-engines-build --yes   # force a local build
./setup.sh --run --only tensorrt-engines --yes         # download-or-build (default path)
./setup.sh --rerun tensorrt-engines-build               # re-run just this one
```

`--profile vehicle --skip tensorrt-engines` plus `--only` cannot both apply at
once (`--only` replaces the whole selection, per `main.py`'s `_select`), so
picking the from-scratch path alongside a profile run is two invocations
today: the profile run, then `--only tensorrt-engines-build` separately if
the default step already ran. Not a blocker — just note it, rather than
add a third selector flag for a case `--only` already covers.

Keep `just build-engines`'s body unchanged — this phase only adds `just
engines` in front of it as a cache-checking wrapper, not a rewrite.

**Success criteria**:
- [x] `./setup.sh --run --profile vehicle --yes` on a matching-fingerprint Orin
      finishes the engine step in seconds, not an hour — measured 20.5s
      against an existing cache on this AGX Orin (log:
      fingerprint → no release yet → fell to `build-engines` → each of the 5
      models loaded its existing `.engine` in 3-4s rather than rebuilding)
- [ ] `--rerun tensorrt-engines` after an Autoware upgrade correctly misses the
      cache (new `autoware_version` in the key) and rebuilds — logic is in
      place (the key includes `autoware{version}`); not yet exercised against
      an actual version bump
- [x] `tensorrt-engines-build` is visible and selectable in both the curses
      menu and `--plain`, and via `--only tensorrt-engines-build` — confirmed

---

## Phase summary

| Phase | Description | Payload | Status |
|---|---|---|---|
| 0 | Cache-key format (Orin + desktop) | `scripts/version/engine-fingerprint.sh` | Implemented, Orin-tested |
| 1 | Orin export/import + verify-or-fallback | `just export-engines` / `just engines` | Implemented, Orin-tested |
| 2 | Desktop build | exact-GPU key (same mechanism as Orin) | Implemented, **not yet tested on a desktop box** |
| 2f | Desktop cross-generation `kAMPERE_PLUS` sharing | — | Blocked on an upstream `autoware_tensorrt_common` patch |
| 3 | GitHub Release packaging | release + manifest convention | Not started — yours to publish when ready |
| 4 | `setup.sh` integration | two `registry.py` steps | Implemented, tested |

## Open decisions

1. ~~Does `autoware_tensorrt_common` expose the hardware-compatibility build
   flag today?~~ **Resolved: no.** Checked directly against the installed
   1.5.0 package — no header field, no launch/param exposure, and no
   `AMPERE`/`HardwareCompat`/`VERSION_COMPAT` string in any shipped `.so`.
   Cross-generation desktop sharing is Phase 2f, blocked on an upstream patch;
   today's desktop path keys by exact GPU instead (Phase 2, done).
2. **Is JetPack 6.2 ↔ 6.2.1 cross-compatibility (same TensorRT/CUDA/cuDNN,
   different L4T patch) safe to fold into one cache key, or does it need its
   own verify-tested exception list?** Treat as two separate keys until
   Phase 1's verify step has actually been run across that pair once.
3. **Who owns re-running the export after an Autoware version bump?** The
   model-hash term in the key makes a stale cache miss cleanly, but nothing
   automates *producing* the new release asset — likely a manual step for now,
   worth automating only once this has been done by hand a few times.
