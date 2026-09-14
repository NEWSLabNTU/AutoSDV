# Portable TensorRT engine delivery

**Goal**: stop paying the `tensorrt-engines` setup step's build time (~1hr on an
Orin) on every fresh install or reprovision. Build each engine once per
compatible-hardware class, publish it as a release asset on
`NEWSLabNTU/AutoSDV`, and have `setup.sh` pull a match before it falls back to
building locally.

**Status**: Phases 0, 1 and 4 implemented and tested end-to-end on an AGX
Orin (`just engines` on an existing cache: 20.5s, vs. the ~1hr build). Phase 2
now also exercised on a desktop box (RTX 3090) — which is where the
exact-TensorRT-patch requirement below was found, and it needed fixing before
a desktop cache could work at all. Cross-generation `kAMPERE_PLUS` sharing
remains unavailable without an upstream patch, see below. Phase 3 (actually
publishing a release) is next — yours to do per your own timeline.

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

### 2.1 What the first desktop run found: TensorRT must match to the PATCH

Building on amd64 worked first time. *Reusing* what it built did not, and the
reason is not hardware:

```
[W] [TRT] Validation failed for the existing engine file. Rebuilding
[I] [TRT] Plan was created with TensorRT 10.9.0
[W] [TRT] Plan was created with a different version of TensorRT! Current version: 10.8.0
```

`TrtCommon::validateEngine()` reads bytes 24-26 of the `.engine` — the plan's
own TensorRT version — and compares all three against the
`NV_TENSORRT_MAJOR/MINOR/PATCH` macros baked into `autoware_tensorrt_common`
when the Debian was built. Any difference discards the engine and rebuilds. The
package's `Depends: libnvinfer10` cannot express this: it is satisfied by 10.9
and by 10.16 alike, and `versions.yaml` said as much ("any patch version
satisfying those sonames works") because until now nothing had tested reuse on
amd64.

Measured on the 3090 workstation, whose loader resolved TensorRT **10.9.0**
from a tarball under `/usr/local/cuda-12.8/targets` while the Autoware Debians
were built against **10.8.0**:

| | build | second `just build-engines` |
|---|---|---|
| TensorRT 10.9.0 loaded (skew) | 9 min for 5 models | 9 min again, and again, every launch |
| TensorRT 10.8.0 loaded (match) | same 9 min once | **19 s** — 3-5 s per model, all loads |

Nothing about this is desktop-specific in principle — it is simply invisible on
a Jetson, where JetPack's TensorRT *is* the one the arm64 Debians were built
against, so the versions agree by construction. On amd64 they agree only by
accident. This is why the Orin cache appeared to work from the first attempt
and why Phase 0's "version, not just architecture" note was righter than it
knew: the version that matters is the one the *loader resolves*, not the one
`dpkg` lists (this box has three TensorRTs installed: 10.9.0 in the CUDA tree,
10.15.1.29 from apt, 8.6.1.6 left over).

**What ships for it**:

- `versions.yaml` gains `nvidia_amd64.tensorrt_engine_abi: "10.8.0"` — the
  exact version Autoware's own build log reports, with the command to re-read
  it after an Autoware upgrade.
- `setup/scripts/install-tensorrt-runtime.sh` + setup step `tensorrt-runtime`
  extract that runtime into `/opt/tensorrt/<version>`. A private prefix, not an
  apt downgrade: downgrading the system `libnvinfer` dragged apt into the whole
  TensorRT dev/python set (it wanted to install TensorRT **11** dev packages to
  satisfy a 10.8 runtime downgrade) and would have taken every other CUDA
  project on the workstation with it.
- `scripts/trt-runtime-env.sh`, sourced by `scripts/env.sh` and by
  `just build-engines`, puts that prefix first on `LD_LIBRARY_PATH`. It is a
  no-op on arm64 and a no-op when the prefix is absent.
- `just build-engines` now greps its children for the skew warning and, if it
  sees one, says outright that the engines it just built will never be reused
  and names the step that fixes it. Nine minutes of work silently producing a
  cache that misses is the failure mode this whole document exists to remove.
- `scripts/version/engine-fingerprint.sh` keys on the TensorRT the *loader*
  resolves under that same environment, so the key names what actually stamped
  the plans. On amd64 there is no `tensorrt` metapackage for `dpkg-query` to
  read at all, which is what made the script exit 1 on its first desktop run.

Two side findings from the same session, both small and both now fixed:

- `scripts/version/get-version.sh` preferred `yq` when present, in a form only
  one of the two `yq` implementations accepts. With mikefarah `yq` 4.16 it
  printed nothing and exited 0, so `AUTOWARE_VERSION` came out empty and the
  fingerprint script died on a box that looked fully provisioned — the exact
  silent-empty failure `versions.yaml`'s own header warns about. It now reads
  the file with PyYAML, which `export-versions.sh` already required.
- Engine builds are **not** byte-reproducible: the same model rebuilt on the
  same box in the same session produced 11 698 044 then 11 954 364 bytes. Phase
  3's "byte-identical" success criterion is therefore unattainable by
  construction; load-verified equivalence is the only testable form of it.

**Success criteria**:
- [x] the fingerprint script and `just export-engines`/`just engines` work
      unmodified on a desktop box (no platform branch needed beyond the key
      format itself)
- [x] confirmed on an actual desktop box — RTX 3090, Autoware 1.5.0, five
      models built in ~9 min (yolox 304s, fine detector 123s, 2x classifier
      39s each, centerpoint 33s), exported as
      `desktop-rtx-3090-<trt>-autoware1.5.0.tar.gz` (54 MB, 6 `.engine` files)
- [x] engine reuse verified on that box: a second `build_only` run loads every
      engine instead of rebuilding, once the TensorRT versions match (19 s for
      the set, against 9 min of rebuilds)
- [x] the exported asset verified as an artifact, not just as a tarball: every
      `.engine` deleted, the tarball extracted back over `data/autoware_data`,
      and `just build-engines` then loaded all six in 19.5 s with no rebuild
- [x] `just engines` exercised on the desktop key against the real release:
      fingerprints, reads the published `manifest.json`, reports
      `no cached engine set yet` for a key the release does not carry, and falls
      through to the local build
- [ ] (future, post-upstream-patch) one hw-compat engine set loads and infers
      correctly on a 3090, a 4090, and a 5090 without rebuilding

---

## Phase 3 — Release packaging on `NEWSLabNTU/AutoSDV`

**Objective**: a place these artifacts live that isn't "whoever built it last
keeps the tarball on their laptop."

**Published.** `engines-autoware-1.5.0` now carries both assets and a
`manifest.json` naming both keys:

| key | asset | size |
|---|---|---|
| `orin-agx-orin-R36.4.4-10.3.0.30-autoware1.5.0` | `…tar.gz` | 48.9 MB |
| `desktop-rtx-3090-10.8.0-autoware1.5.0` | `…tar.gz` | 56.2 MB |

Adding one is an upload plus one manifest entry, which `just export-engines`
prints with its sha256 already computed:

```bash
gh release upload engines-autoware-1.5.0 <asset>
# then add the printed entry to manifest.json and re-upload it with --clobber
```

The assets are gitignored (`/desktop-*.tar.gz`, `/orin-*.tar.gz`) so a 50 MB
release artifact cannot be committed by accident.

**The download path, measured against that release** (RTX 3090, engines deleted
first):

| run | time | what happened |
|---|---|---|
| cold | 30.7 s | manifest, 56 MB asset, extract, all 5 models load |
| again | 19.9 s | marker matches, network skipped entirely |
| local build | ~9 min | what both of the above replace |

`./setup.sh --run --only tensorrt-engines --yes` drives the same path and
reports `ok` on both a cold and a warm cache.

**Failure modes exercised, not assumed** — each of the first three used to end
in the full local build:

- a resumed download finishing with `curl` exit 0 and a corrupt file (the
  release URL redirects to a storage host, and `-C -` through that redirect was
  measured doing exactly this). A checksum failure now forces one clean whole
  re-fetch; a cleanly truncated partial still resumes.
- a marker sitting beside a deleted engine. `.engine-cache-files` records what
  the sync placed, and the marker is believed only while all of it is present.
- a `manifest.json` served stale by the CDN for ~30 s after upload, where a
  missing key is indistinguishable from an unpublished asset. Fetched with
  `Cache-Control: no-cache` and a query salt now.
- a kill mid-download: no marker, no engines moved, the partial kept as the
  resume point. Verified by killing the transfer at 16 MB of 56 MB.

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
- [x] a box with no engines pulls the release asset and reaches loaded engines
      with zero real builds — confirmed on the desktop key; the equivalent on a
      second Orin is still to do, and is the same code path
- [ ] a fresh Orin box, `gh release download` + import, running pipeline,
      zero local `build_only` invocations
- [x] the release is reproducible in the only sense available: re-running the
      export produces a load-verified-equivalent artifact, never a byte-identical
      one. The same model rebuilt on the same box minutes apart differed in size
      (11 698 044 vs 11 954 364 bytes), so byte-identity is not a property
      TensorRT offers and cannot be a release check

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
| 2 | Desktop build | exact-GPU key (same mechanism as Orin) | Implemented and tested on an RTX 3090 |
| 2a | amd64 TensorRT must match Autoware's build to the patch, or no engine is ever reused | `tensorrt_engine_abi`, setup step `tensorrt-runtime`, `scripts/trt-runtime-env.sh` | Implemented, measured |
| 2f | Desktop cross-generation `kAMPERE_PLUS` sharing | — | Blocked on an upstream `autoware_tensorrt_common` patch |
| 3 | GitHub Release packaging | release + manifest convention | Published: Orin + desktop assets, manifest with both keys; download path measured and its failure modes tested |
| 4 | `setup.sh` integration | two `registry.py` steps | Implemented, tested |

## Open decisions

1. ~~Does `autoware_tensorrt_common` expose the hardware-compatibility build
   flag today?~~ **Resolved: no.** Checked directly against the installed
   1.5.0 package — no header field, no launch/param exposure, and no
   `AMPERE`/`HardwareCompat`/`VERSION_COMPAT` string in any shipped `.so`.
   Cross-generation desktop sharing is Phase 2f, blocked on an upstream patch;
   today's desktop path keys by exact GPU instead (Phase 2, done).
2. ~~**Is JetPack 6.2 ↔ 6.2.1 cross-compatibility safe to fold into one cache
   key?**~~ **Moot, and the reason is worth keeping.** The project now pins
   JetPack **6.2.2 or newer**, and NVIDIA's patch numbering crosses an L4T minor
   inside the 6.2 series: 6.2 is L4T 36.4.3, 6.2.1 is 36.4.4, but 6.2.2 is
   **36.5.0** and 6.2.3 is 36.5.2. So the question is no longer about two patches
   of one minor.

   What it leaves behind is a real gap: **the published AGX Orin asset
   (`orin-agx-orin-R36.4.4-...`) was built on JetPack 6.2.1 and will miss on any
   board at the new floor.** The miss is clean — the key simply does not appear in
   the manifest, and `just engines` falls through to a local build — but it is an
   hour, on exactly the boards a fresh install is most likely to be flashed to.
   Someone should run `just build-engines` and `just export-engines` on a 6.2.2
   AGX Orin and publish the result; that also settles whether L4T 36.5 changes the
   TensorRT patch, which the fingerprint reads off the board rather than assuming. The
   amd64 finding in Phase 2.1 argues for keeping them separate: the check that
   rejects an engine is a patch-level comparison, and it is cheap to be wrong
   in the safe direction.
2b. **Should the desktop key carry the Autoware-side TensorRT as well as the
   loaded one?** Today they are equal by construction — `trt-runtime-env.sh`
   makes the loaded one match `tensorrt_engine_abi`, and a mismatch is reported
   rather than encoded. Worth revisiting only if a desktop is ever deliberately
   run with a non-matching TensorRT.
3. **Who owns re-running the export after an Autoware version bump?** The
   model-hash term in the key makes a stale cache miss cleanly, but nothing
   automates *producing* the new release asset — likely a manual step for now,
   worth automating only once this has been done by hand a few times.
