#!/usr/bin/env bash
# Check out the submodules that are missing, and only those.
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
# which names neither the submodule nor `git submodule update`. Forgetting to
# init the submodules is the first thing a newcomer to this repository does, so
# this step runs before everything else and fills them in.
#
# It only ever CREATES. A submodule that is already checked out is left exactly
# as it is -- including one with uncommitted work in it, and one sitting at a
# commit other than the pin. Both are ordinary states to be working in, and
# neither has anything to do with the empty directories this is here to fill.
# They are reported at the end, because a build that then behaves oddly should
# have somewhere to look, but they do not stop anything.
#
# That is also why the missing paths are updated one owner at a time rather than
# with a blanket `git submodule update --init --recursive`: the blanket form
# would move a submodule that is deliberately parked at another commit, and the
# work it moved away from may be the only copy.

set -euo pipefail

cd "$(git rev-parse --show-toplevel)"

if [[ ! -f .gitmodules ]]; then
    echo "No .gitmodules in this repository; nothing to check out."
    exit 0
fi

# The repository that OWNS a submodule path. Nested submodules (seyond_sdk
# inside seyond_ros_driver) are pinned and updated by their parent, not by the
# superproject: `git submodule update -- <full path>` from here matches nothing.
owner_of() {
    local owner
    owner="$(dirname "$1")"
    while [[ "$owner" != "." && ! -e "$owner/.git" ]]; do
        owner="$(dirname "$owner")"
    done
    printf '%s' "$owner"
}

pinned_sha() {
    local path="$1" owner rel
    owner="$(owner_of "$path")"
    if [[ "$owner" == "." ]]; then rel="$path"; else rel="${path#"$owner"/}"; fi
    git -C "$owner" rev-parse --short "HEAD:$rel" 2>/dev/null || echo "unknown"
}

missing=()      # never initialised: what this script is here to fix
left=()         # initialised, and not ours to touch -- reported at the end
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
        U) left+=("$path|unresolved merge conflict on the pin") ;;
        +) left+=("$path|at ${sha:0:7}, pinned $(pinned_sha "$path")") ;;
        *)
            # Untracked files are not a local change worth mentioning: a built
            # Cython extension or a colcon artefact lives in these trees
            # routinely. Gitlink changes are not either -- --recursive already
            # reports the nested submodule in its own right, and reporting the
            # parent again as " M <name>" says the same thing less usefully.
            if [[ -n "$(git -C "$path" status --porcelain --untracked-files=no --ignore-submodules=all 2>/dev/null)" ]]; then
                left+=("$path|uncommitted changes")
            fi
            ;;
    esac
done < <(git submodule status --recursive)

report_left() {
    ((${#left[@]})) || return 0
    echo
    echo "Left as they are (already checked out; this step only fills in empty ones):"
    for entry in "${left[@]}"; do
        echo "  ${entry%%|*}  --  ${entry#*|}"
    done
    echo
    echo "That is normal while you are working on one. It does mean the build"
    echo "uses what is in that directory, not what this repository pins."
}

if ((${#missing[@]} == 0)); then
    echo "All $total submodule(s) are checked out."
    report_left
    exit 0
fi

echo "Checking out ${#missing[@]} of $total submodule(s):"
printf '  %s\n' "${missing[@]}"
echo

# Grouped by owning repository so the common case -- a fresh clone, where every
# missing submodule belongs to the superproject -- is one git invocation and can
# still use whatever submodule.fetchJobs is configured.
declare -A by_owner=()
for path in "${missing[@]}"; do
    owner="$(owner_of "$path")"
    if [[ "$owner" == "." ]]; then rel="$path"; else rel="${path#"$owner"/}"; fi
    by_owner["$owner"]+="$rel"$'\n'
done

for owner in "${!by_owner[@]}"; do
    mapfile -t rels < <(printf '%s' "${by_owner[$owner]}")
    git -C "$owner" submodule update --init --recursive -- "${rels[@]}"
done

still_missing="$(git submodule status --recursive | grep '^-' || true)"
if [[ -n "$still_missing" ]]; then
    echo >&2
    echo "Some submodules are still not checked out:" >&2
    echo "$still_missing" | sed 's/^/  /' >&2
    exit 1
fi

echo
echo "Done."
report_left
