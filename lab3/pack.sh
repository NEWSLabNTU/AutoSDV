#!/bin/bash
# Part 2: pack this clone for submission as ../autosdv.tgz (this folder must be AutoSDV).
# Refuses unless your branch fast-forwards from the newest lab3-v* tag it contains, the
# submodules are as that tag pins them, and everything is committed and pushed.
set -e
cd "$(git rev-parse --show-toplevel)"
fail() { echo "Refused: $*" >&2; exit 1; }
base=$(git describe --tags --abbrev=0 --match 'lab3-v*' HEAD 2>/dev/null) \
    || fail "no lab3-v* tag in your history: clone from lab3-v1"
git merge-base --is-ancestor $base HEAD || fail "$base is not an ancestor of HEAD"
diff <(git ls-tree -r $base | grep '^160000') <(git ls-tree -r HEAD | grep '^160000') >/dev/null \
    || fail "a submodule pin differs from $base"
git submodule status --recursive | grep -q '^+' && fail "a submodule is not at its pinned commit"
[ -z "$(git status --porcelain)" ] || fail "uncommitted changes; commit first"
git fetch -q && [ "$(git rev-parse HEAD)" = "$(git rev-parse @{u})" ] || fail "not pushed; push first"
{ echo "fork $(git remote get-url origin)"; echo "HEAD $(git rev-parse HEAD)"
  echo "date $(date -Iseconds)"; git submodule status --recursive; } > lab3/manifest.txt
cd .. && tar -czf autosdv.tgz -X AutoSDV/.packignore AutoSDV && ls -lh autosdv.tgz
