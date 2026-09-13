# Source the environment every AutoSDV command needs, in the one order that
# works. Not executable: `source scripts/env.sh` from a bash shell.
#
# Why this file exists
# --------------------
# `install/setup.bash` does not chain to Autoware, and Autoware's own
# setup.bash is what sets RMW_IMPLEMENTATION and CYCLONEDDS_URI. Source only
# ROS and the workspace and you get rmw_fastrtps_cpp, which is a *different
# middleware* from the one the rest of the stack is using -- so nodes start
# cleanly, report themselves ready, and cannot see each other. There is no
# error message for this; topics are simply empty. It has cost this project
# days more than once, and it is defect 5 in docs/known-config-defects.md.
#
# Order matters: Autoware first, workspace last, so the workspace's overlay
# wins for packages it rebuilds.

_autosdv_env_repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# The prefix is recorded in versions.yaml so an Autoware upgrade is one edit.
_autosdv_env_prefix="$(
    "${_autosdv_env_repo}/scripts/version/get-version.sh" autoware.install_prefix 2>/dev/null
)"
: "${_autosdv_env_prefix:=/opt/autoware/1.5.0}"

set +u   # ROS setup hooks read variables that are unset
if [ -f "${_autosdv_env_prefix}/setup.bash" ]; then
    source "${_autosdv_env_prefix}/setup.bash"
else
    # No Autoware installed: ROS alone, and say so, because every launch that
    # needs an Autoware package is about to fail with a confusing message.
    echo "scripts/env.sh: ${_autosdv_env_prefix} not found; sourcing ROS only." >&2
    source /opt/ros/humble/setup.bash
    export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
fi

if [ -f "${_autosdv_env_repo}/install/setup.bash" ]; then
    source "${_autosdv_env_repo}/install/setup.bash"
fi
set -u

unset _autosdv_env_repo _autosdv_env_prefix
