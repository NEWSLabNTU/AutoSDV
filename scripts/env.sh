# Source the environment every AutoSDV command needs, in the one order that
# works. Not executable: `source scripts/env.sh` from a bash shell.
#
# Why this file exists
# --------------------
# Two layers have to be present and `install/setup.bash` does not chain to the
# one underneath it, so a shell with only ROS and the workspace cannot see the
# Autoware packages AutoSDV is built against -- and, because Autoware's
# setup.bash is also what selects the middleware, cannot see a running stack
# either. Nodes start cleanly, report themselves ready, and publish into a void.
# There is no error message for that; topics are simply empty. It is defect 5 in
# docs/known-config-defects.md.
#
# Order matters: Autoware first, workspace last, so the workspace's overlay
# wins for packages it rebuilds.
#
# What this file deliberately does NOT do
# ---------------------------------------
# It sets no middleware of its own. Which RMW to run is the user's choice --
# CycloneDDS, Zenoh, something else -- and it belongs in `.envrc`, not in a
# helper every recipe sources. The one thing done here is to keep a choice the
# user has already made (in .envrc, which sets the default when the directory
# is entered): some Autoware builds' setup.bash export an RMW, so an explicit
# selection has to be restored after it runs or it is silently overwritten.

_autosdv_env_repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Empty unless the user picked one; restored below.
_autosdv_env_rmw="${RMW_IMPLEMENTATION:-}"

# Which Autoware, and so which workspace, is the checkout's selection
# (versions.yaml; scripts/version/select-autoware.sh). It refuses a shell that
# already holds another version's underlay; then nothing is sourced, and the
# caller's next command fails visibly rather than running on a mixed one.
set +u   # ROS setup hooks read variables that are unset
if ! source "${_autosdv_env_repo}/scripts/version/select-autoware.sh"; then
    echo "scripts/env.sh: nothing sourced." >&2
elif [ -f "${AUTOSDV_AUTOWARE_PREFIX}/setup.bash" ]; then
    source "${AUTOSDV_AUTOWARE_PREFIX}/setup.bash"
    # local_setup.bash, not setup.bash: setup.bash replays the underlay chain
    # recorded at build time.
    if [ -f "${AUTOSDV_WS_INSTALL}/local_setup.bash" ]; then
        source "${AUTOSDV_WS_INSTALL}/local_setup.bash"
    fi
else
    # No Autoware installed: ROS alone, and say so, because every launch that
    # needs an Autoware package is about to fail with a confusing message.
    echo "scripts/env.sh: ${AUTOSDV_AUTOWARE_PREFIX} not found; sourcing ROS only." >&2
    source /opt/ros/humble/setup.bash
fi
set -u

if [ -n "${_autosdv_env_rmw}" ]; then
    export RMW_IMPLEMENTATION="${_autosdv_env_rmw}"
fi

# After Autoware, so this wins: on amd64 the host's TensorRT is usually a
# different patch of 10.x than the Debians were built against, which silently
# voids every cached .engine. No-op on arm64 and when the prefix is absent.
source "$(dirname "${BASH_SOURCE[0]}")/trt-runtime-env.sh"

unset _autosdv_env_repo _autosdv_env_rmw
