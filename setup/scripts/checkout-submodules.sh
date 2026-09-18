#!/usr/bin/env bash
# Check out every submodule at the commit this repository pins.
#
# The workspace is mostly submodules, and a fresh clone has none of them: the
# directories under src/ exist and are empty. Nothing says so until a later step
# reaches inside one, and then the failure is about a path rather than about a
# submodule:
#
#   bash: line 1: cd: .../src/localization/external/range_libc/pywrapper:
#   No such file or directory
#   FAILED  range-libc
#
# which names neither the submodule nor `git submodule update`. So this runs
# first and fills them in.
#
# It only ever CREATES. A submodule that is checked out at something other than
# the pin, or that has uncommitted work in it, stops the run instead of being
# fixed: `git submodule update` would move it without a word, and what it moved
# away from is somebody's unpushed afternoon. That is also the case this cannot
# guess at -- a moved pin is how a submodule change is made, so the checkout may
# be the newer truth and the superproject the thing that is behind.

set -euo pipefail

cd "$(git rev-parse --show-toplevel)"

if [[ ! -f .gitmodules ]]; then
    echo "No .gitmodules in this repository; nothing to check out."
    exit 0
fi

# The repository that OWNS a submodule path, and the path relative to it.
# Nested submodules (seyond_sdk inside seyond_ros_driver) are pinned by their
# parent, not by the superproject, so `git rev-parse HEAD:<full path>` would
# fail on exactly the ones worth reporting precisely.
pinned_sha() {
    local path="$1" owner rel
    owner="$(dirname "$path")"
    while [[ "$owner" != "." && ! -e "$owner/.git" ]]; do
        owner="$(dirname "$owner")"
    done
    if [[ "$owner" == "." ]]; then
        rel="$path"
    else
        rel="${path#"$owner"/}"
    fi
    git -C "$owner" rev-parse --short "HEAD:$rel" 2>/dev/null || echo "unknown"
}

missing=()      # never initialised: what this script is here to fix
moved=()        # initialised, but at a different commit
conflicted=()   # a merge left the gitlink unresolved
dirty=()        # at the pin, with uncommitted tracked changes
total=0

# `git submodule status --recursive` prints "<flag><sha> <path> (<describe>)",
# where the flag is ' ', '-', '+' or 'U', and paths are relative to the top
# level. It descends only into INITIALISED submodules, which is what we want:
# anything under a missing parent arrives fresh, at its parent's pin, once the
# parent is cloned.
while IFS= read -r line; do
    [[ -n "$line" ]] || continue
    flag="${line:0:1}"
    rest="${line:1}"
    sha="${rest%% *}"
    path="${rest#* }"
    path="${path%% (*}"
    total=$((total + 1))

    case "$flag" in
        -) missing+=("$path") ;;
        U) conflicted+=("$path") ;;
        +) moved+=("$path ${sha:0:7} $(pinned_sha "$path")") ;;
        *)
            # Untracked files do not count. A built Cython extension, a
            # colcon artefact or an editor swap file lives in these trees
            # routinely, and `git submodule update` does not touch them --
            # refusing over one would make this step unpassable on any machine
            # that has ever built the workspace.
            #
            # Neither does a moved gitlink: --recursive already reports the
            # nested submodule itself, and without --ignore-submodules the
            # parent is reported a second time, as " M <name>", for the same
            # fact in less useful words.
            if [[ -n "$(git -C "$path" status --porcelain --untracked-files=no --ignore-submodules=all 2>/dev/null)" ]]; then
                dirty+=("$path")
            fi
            ;;
    esac
done < <(git submodule status --recursive)

if ((${#moved[@]} || ${#dirty[@]} || ${#conflicted[@]})); then
    echo >&2
    echo "Refusing to check out submodules: some do not match the pinned commits." >&2
    echo >&2

    for entry in "${moved[@]}"; do
        read -r path have want <<<"$entry"
        echo "  at a different commit: $path" >&2
        echo "      pinned:       $want" >&2
        echo "      checked out:  $have" >&2
    done
    for path in "${dirty[@]}"; do
        echo "  uncommitted changes: $path" >&2
        git -C "$path" status --short --untracked-files=no --ignore-submodules=all 2>/dev/null \
            | sed 's/^/      /' >&2
    done
    for path in "${conflicted[@]}"; do
        echo "  unresolved merge conflict: $path" >&2
    done

    cat >&2 <<'HINT'

Nothing was checked out. Resolve each one, then re-run setup:

  * uncommitted changes you want to keep -- commit them on the submodule's
    tracking branch and push, then record the new pin in the superproject:

        cd <submodule> && git checkout <its branch> && git commit -a && git push
        cd - && git add <submodule> && git commit -m "Bump <submodule>"

    The submodule push comes FIRST: a pin nobody else can resolve fails every
    other checkout of this repository, and CI with it.

  * uncommitted changes you do not want -- discard them:

        git -C <submodule> checkout -- .

  * a submodule sitting at a different commit on purpose -- leave it there and
    let setup skip this step:

        ./setup.sh --run --skip submodules

  * a submodule sitting at a different commit by accident -- put it back:

        git submodule update --checkout -- <submodule>
HINT
    exit 1
fi

if ((${#missing[@]} == 0)); then
    echo "All $total submodule(s) are checked out at their pinned commits."
    exit 0
fi

echo "Checking out ${#missing[@]} of $total submodule(s):"
printf '  %s\n' "${missing[@]}"
echo

# A blanket update rather than a pathspec per missing entry. Everything already
# initialised has just been verified clean and at its pin, so this is a no-op
# for those -- and a pathspec cannot name a submodule nested inside another one
# from here anyway.
git submodule update --init --recursive

if git submodule status --recursive | grep -q '^-'; then
    echo >&2
    echo "Some submodules are still not checked out:" >&2
    git submodule status --recursive | grep '^-' | sed 's/^/  /' >&2
    exit 1
fi

echo
echo "All submodules are checked out at their pinned commits."
