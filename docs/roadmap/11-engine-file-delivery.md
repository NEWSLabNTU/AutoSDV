# Portable TensorRT engine delivery

**Goal**: stop paying the `tensorrt-engines` setup step's build time (~1hr on an
Orin) on every fresh install or reprovision. Build each engine once per
compatible-hardware class, publish it as a release asset on
`NEWSLabNTU/AutoSDV`, and have `setup.sh` pull a match before it falls back to
building locally.

**Status**: Not started.

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

## Phase 2 — Desktop hardware-compatible build

**Objective**: one engine set, built with `BuilderFlag::kAMPERE_PLUS`, that a
30xx/40xx/50xx desktop dev box can all pull instead of building natively.

This only matters if desktop boxes run these TensorRT nodes at all (dev/test
convenience) — it has no bearing on the Orin fleet; a desktop-built engine
cannot load on Orin regardless of flags (different compute capability, and
JetPack's hardware-compat support gap cuts both directions).

### 2.1 Where the flag is set

Need to confirm whether `autoware_tensorrt_common`'s build path exposes
`kAMPERE_PLUS` today or whether this needs a patch carried the same way the
CUDA pointcloud filters are (`docs/design/cuda-pipeline-data-flow.md`'s
retire-upstream-first pattern) — **open decision, see below.**

**Success criteria**:
- [ ] one hw-compat engine set loads and infers correctly on a 3090, a 4090,
      and a 5090 without rebuilding
- [ ] measured perf delta vs a native build is recorded (expected: small, per
      TensorRT's own docs, but unmeasured for these specific models)

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

**Objective**: `tensorrt-engines` tries the cache before it builds.

```
Step(
    id="tensorrt-engines",
    ...
    run=[... compute key, attempt fetch+verify, else `just build-engines` ...],
)
```

Keep the existing step as the fallback body unchanged — this phase only adds a
cheap short-circuit in front of it, not a rewrite of `build-engines` itself.

**Success criteria**:
- [ ] `./setup.sh --run --profile vehicle --yes` on a matching-fingerprint Orin
      finishes the engine step in seconds, not an hour
- [ ] `--rerun tensorrt-engines` after an Autoware upgrade correctly misses the
      cache (new `autoware_version` in the key) and rebuilds

---

## Phase summary

| Phase | Description | Payload | Status |
|---|---|---|---|
| 0 | Cache-key format (Orin + desktop) | a key-computing script | Not started |
| 1 | Orin export/import + verify-or-fallback | scripted procedure | Not started |
| 2 | Desktop `kAMPERE_PLUS` build | one cross-GPU engine set | Not started |
| 3 | GitHub Release packaging | release + manifest convention | Not started |
| 4 | `setup.sh` short-circuit | registry.py change | Not started |

## Open decisions

1. **Does `autoware_tensorrt_common` expose the hardware-compatibility build
   flag today, or does Phase 2 need an upstream patch first?** Unconfirmed —
   check before scheduling Phase 2; if it needs a patch, it follows the same
   "fix upstream first, submodule mirrors the branch" rule as the CUDA
   pointcloud filters, not a local hack.
2. **Is JetPack 6.2 ↔ 6.2.1 cross-compatibility (same TensorRT/CUDA/cuDNN,
   different L4T patch) safe to fold into one cache key, or does it need its
   own verify-tested exception list?** Treat as two separate keys until
   Phase 1's verify step has actually been run across that pair once.
3. **Who owns re-running the export after an Autoware version bump?** The
   model-hash term in the key makes a stale cache miss cleanly, but nothing
   automates *producing* the new release asset — likely a manual step for now,
   worth automating only once this has been done by hand a few times.
