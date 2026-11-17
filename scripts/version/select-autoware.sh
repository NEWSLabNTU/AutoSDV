# Resolve the selected Autoware version into the paths a shell needs. Sourced,
# not run, by .envrc and scripts/env.sh; bash.
#
# Sets, for the caller:
#   AUTOSDV_AUTOWARE_VERSION   exported: the selection (autoware-version.sh)
#   AUTOSDV_AUTOWARE_PREFIX    exported: its install prefix, /opt/autoware/<v>
#   AUTOSDV_WS_INSTALL         exported: this checkout's colcon install folder
#                              for that version (install, install-2.1.0, ...)
#   AUTOSDV_DATA_PATH          exported unless already set: the model folder the
#                              launches read (data_path in versions.yaml)
#
# Returns 1, saying why, when the selection is not in versions.yaml or when the
# shell already carries ANOTHER Autoware underlay (AUTOWARE_VERSION, which every
# Autoware setup.bash exports, names a different version). Sourcing a second
# underlay over the first is the mixed-environment failure: packages resolve
# from both, and libraries of one version load into nodes built for the other.
# The fix is a shell that has not sourced the other one (look in ~/.bashrc), or
# selecting the version it carries.

_autosdv_sel_repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
_autosdv_sel_out="$(
    "${_autosdv_sel_repo}/scripts/version/autoware-version.sh" | {
        read -r v
        python3 - "${_autosdv_sel_repo}/versions.yaml" "$v" <<'PY'
import sys, yaml
path, v = sys.argv[1:3]
e = yaml.safe_load(open(path))['autoware']['versions'][v]
print(v)
print(e['install_prefix'])
print('install' + (e.get('workspace_suffix') or ''))
print(e['data_path'])
PY
    }
)" || { unset _autosdv_sel_repo _autosdv_sel_out; return 1; }

{
    read -r _autosdv_sel_v
    read -r _autosdv_sel_prefix
    read -r _autosdv_sel_install
    read -r _autosdv_sel_data
} <<< "${_autosdv_sel_out}"

if [ -n "${AUTOWARE_VERSION:-}" ] && [ "${AUTOWARE_VERSION}" != "${_autosdv_sel_v}" ]; then
    echo "AutoSDV: this checkout selects Autoware ${_autosdv_sel_v}, but the shell already" >&2
    echo "  carries Autoware ${AUTOWARE_VERSION} (AUTOWARE_VERSION is set: something sourced" >&2
    echo "  ${AUTOWARE_HOME:-/opt/autoware/${AUTOWARE_VERSION}}/setup.bash, often ~/.bashrc)." >&2
    echo "  Two underlays do not mix. Open a shell without it, or select ${AUTOWARE_VERSION}:" >&2
    echo "      echo ${AUTOWARE_VERSION} > ${_autosdv_sel_repo}/.autoware-version" >&2
    unset _autosdv_sel_repo _autosdv_sel_out _autosdv_sel_v _autosdv_sel_prefix \
          _autosdv_sel_install _autosdv_sel_data
    return 1
fi

export AUTOSDV_AUTOWARE_VERSION="${_autosdv_sel_v}"
export AUTOSDV_AUTOWARE_PREFIX="${_autosdv_sel_prefix}"
export AUTOSDV_WS_INSTALL="${_autosdv_sel_repo}/${_autosdv_sel_install}"
: "${AUTOSDV_DATA_PATH:=${_autosdv_sel_repo}/${_autosdv_sel_data}}"
export AUTOSDV_DATA_PATH

unset _autosdv_sel_repo _autosdv_sel_out _autosdv_sel_v _autosdv_sel_prefix \
      _autosdv_sel_install _autosdv_sel_data
