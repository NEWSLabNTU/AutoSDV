#!/usr/bin/env bash
# Install the race stack's own Python and acados, opt-in, without sudo.
#
#   install-race-stack.sh            install what is missing, then verify
#   install-race-stack.sh --check    verify only; exit 0 when everything is there
#
# What it produces
# ----------------
#   ${AUTOSDV_RACE_VENV:-<repo>/.venv-race}
#       A Python 3.10 venv holding race_planner (editable, from the
#       autoware-drift submodule, with its [test] extra) and acados_template
#       (editable, from the acados source clone).
#   ${ACADOS_ROOT:-$HOME/.local/opt/acados}
#       acados installed at the tag pinned in versions.yaml (race.acados):
#       lib/libacados.so, include/, and bin/t_renderer, the template renderer
#       acados_template would otherwise download on first use.
#   <venv>/race-env.sh
#       Exports ACADOS_SOURCE_DIR, AUTOSDV_RACE_VENV, and SWB when SWB was set
#       while this script ran. Written when anything was
#       installed or the file is missing; a verify-only rerun leaves it alone.
#
# Why a private venv
# ------------------
# race_planner needs numpy >= 1.26 and scipy >= 1.11; Jammy's apt has 1.21 and
# 1.8. A user-site numpy breaks AutoSDV's own build (scripts/build/
# check-python-env.sh refuses it), so nothing here touches the system or user
# Python: everything goes into the venv, which nothing else imports.
#
# Why no LD_LIBRARY_PATH
# ----------------------
# acados' lib/ holds libhpipm.so and libblasfeo.so, the same sonames CasADi
# bundles beside its HPIPM plugin (found there through RUNPATH=$ORIGIN). On
# LD_LIBRARY_PATH, which outranks RUNPATH, acados' copies replace CasADi's and
# race_planner's tracker fails six of its own tests (test_tracker.py: the NMPC
# and LQR cases). So acados is built with RUNPATH=$ORIGIN as well:
# acados_template dlopens lib/libacados.so by absolute path, and its own
# dependencies then resolve next to it without any environment.
#
# Rerunning with everything present only runs the checks.
set -euo pipefail

REPO_DIR="$(cd "$(dirname "$(readlink -f "$0")")/../.." && pwd)"
GET_VERSION="${REPO_DIR}/scripts/version/get-version.sh"
DRIFT_DIR="${REPO_DIR}/src/planning/external/autoware-drift"
PLANNER_DIR="${DRIFT_DIR}/src/planning/race_planner"

ACADOS_TAG="$("$GET_VERSION" race.acados)"
PY_VERSION="$("$GET_VERSION" race.python)"
VENV="${AUTOSDV_RACE_VENV:-${REPO_DIR}/.venv-race}"
ACADOS_ROOT="${ACADOS_ROOT:-${HOME}/.local/opt/acados}"
ACADOS_SRC="${ACADOS_ROOT}/src/acados-${ACADOS_TAG}"
ACADOS_STAMP="${ACADOS_ROOT}/.autosdv-acados-tag"
ENV_FILE="${VENV}/race-env.sh"
VPY="${VENV}/bin/python"

# A ROS-sourced shell puts /opt/ros/... on PYTHONPATH; the venv must not see it.
unset PYTHONPATH
# uv and cargo are user installs, and the setup runner does not source a profile.
export PATH="${HOME}/.local/bin:${HOME}/.cargo/bin:${PATH}"

case "$(uname -m)" in
    x86_64)  TERA_ARCH=amd64 ;;
    aarch64) TERA_ARCH=arm64 ;;
    *) echo "install-race-stack.sh: unsupported architecture $(uname -m)" >&2; exit 1 ;;
esac

say() { echo "race-stack: $*"; }

# The t_renderer version acados_template asks for, read from the acados source
# being installed rather than pinned a second time here.
tera_required() {
    sed -n 's/^TERA_DEFAULT_VERSION *= *"\([0-9.]*\)".*/\1/p' \
        "${ACADOS_SRC}/interfaces/acados_template/acados_template/utils.py" 2>/dev/null || true
}

tera_ok() {
    local want have
    want="$(tera_required)"
    [[ -n "$want" && -x "${ACADOS_ROOT}/bin/t_renderer" ]] || return 1
    have="$("${ACADOS_ROOT}/bin/t_renderer" --version 2>/dev/null | awk '{print $NF}')"
    [[ "$have" == "$want" ]]
}

acados_ok() {
    [[ -f "${ACADOS_ROOT}/lib/libacados.so" ]] \
        && [[ -f "${ACADOS_ROOT}/lib/link_libs.json" ]] \
        && [[ "$(cat "$ACADOS_STAMP" 2>/dev/null)" == "$ACADOS_TAG" ]] \
        && [[ "$(readelf -d "${ACADOS_ROOT}/lib/libacados.so")" == *RUNPATH*\$ORIGIN* ]]
}

# race_planner must resolve to THIS checkout's submodule, so a venv shared
# through AUTOSDV_RACE_VENV with another clone does not pass for this one.
venv_ok() {
    [[ -x "$VPY" ]] || return 1
    "$VPY" - "$PLANNER_DIR" "$ACADOS_SRC" <<'EOF' >/dev/null 2>&1
import os, sys
import casadi, race_planner, acados_template
planner, acados = (os.path.realpath(p) for p in sys.argv[1:3])
assert os.path.realpath(race_planner.__file__).startswith(planner + os.sep)
assert os.path.realpath(acados_template.__file__).startswith(acados + os.sep)
EOF
}

# Each check, by name, so a failure says which part is missing.
check() {
    local missing=()
    venv_ok    || missing+=("venv (${VENV}: race_planner, casadi, acados_template)")
    acados_ok  || missing+=("acados ${ACADOS_TAG} (${ACADOS_ROOT}/lib: libacados.so with RUNPATH \$ORIGIN, link_libs.json)")
    tera_ok    || missing+=("t_renderer $(tera_required) (${ACADOS_ROOT}/bin/t_renderer)")
    [[ -f "$ENV_FILE" ]] || missing+=("env file (${ENV_FILE})")
    if ((${#missing[@]})); then
        printf 'race-stack: missing: %s\n' "${missing[@]}"
        return 1
    fi
    say "verified: venv ${VENV}, acados ${ACADOS_TAG} at ${ACADOS_ROOT}, t_renderer $(tera_required)"
}

if [[ "${1:-}" == "--check" ]]; then
    check
    exit
fi

if [[ ! -f "${PLANNER_DIR}/pyproject.toml" ]]; then
    echo "race-stack: ${PLANNER_DIR} is missing; check out the submodule first:" >&2
    echo "    git submodule update --init src/planning/external/autoware-drift" >&2
    exit 1
fi

write_env() {
    local tmp="${ENV_FILE}.tmp"
    {
        echo "# Written by setup/scripts/install-race-stack.sh. Source it before using the race stack."
        echo "export AUTOSDV_RACE_VENV=\"${VENV}\""
        echo "export ACADOS_SOURCE_DIR=\"${ACADOS_ROOT}\""
        if [[ -n "${SWB:-}" ]]; then
            echo "export SWB=\"${SWB}\""
        fi
    } > "$tmp"
    mv "$tmp" "$ENV_FILE"
}

# --- acados ------------------------------------------------------------------
install_acados() {
    if [[ ! -d "${ACADOS_SRC}/.git" ]]; then
        say "cloning acados ${ACADOS_TAG}"
        rm -rf "${ACADOS_SRC}.partial"
        mkdir -p "$(dirname "$ACADOS_SRC")"
        git clone --branch "$ACADOS_TAG" --depth 1 \
            https://github.com/acados/acados.git "${ACADOS_SRC}.partial"
        git -C "${ACADOS_SRC}.partial" submodule update --init --recursive
        mv "${ACADOS_SRC}.partial" "$ACADOS_SRC"
    fi
    say "building acados ${ACADOS_TAG} into ${ACADOS_ROOT}"
    cmake -S "$ACADOS_SRC" -B "${ACADOS_SRC}/build" \
        -DACADOS_WITH_QPOASES=OFF \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="$ACADOS_ROOT" \
        -DCMAKE_INSTALL_RPATH="\$ORIGIN"
    cmake --build "${ACADOS_SRC}/build" -j"$(nproc)"
    cmake --install "${ACADOS_SRC}/build"
    # acados' CMakeLists writes these into the SOURCE tree's lib/ and does not
    # install them, yet acados_template reads link_libs.json from
    # ${ACADOS_SOURCE_DIR}/lib at every solver generation.
    cp "${ACADOS_SRC}/lib/link_libs.json" "${ACADOS_ROOT}/lib/"
    if [[ -f "${ACADOS_SRC}/lib/git_commit_hash" ]]; then
        cp "${ACADOS_SRC}/lib/git_commit_hash" "${ACADOS_ROOT}/lib/"
    fi
    echo "$ACADOS_TAG" > "$ACADOS_STAMP"
}

# --- t_renderer ----------------------------------------------------------------
# acados_template downloads it on first use into ${ACADOS_SOURCE_DIR}/bin; doing
# it here keeps a solver build from needing the network. There is a prebuilt
# x86_64 binary; on aarch64 it is built from source with cargo.
install_tera() {
    local want tmp
    want="$(tera_required)"
    if [[ -z "$want" ]]; then
        echo "race-stack: cannot read TERA_DEFAULT_VERSION from acados_template" >&2
        return 1
    fi
    mkdir -p "${ACADOS_ROOT}/bin"
    tmp="${ACADOS_ROOT}/bin/t_renderer.partial"
    if [[ "$TERA_ARCH" == amd64 ]]; then
        say "downloading t_renderer ${want}"
        curl -fsSL -o "$tmp" \
            "https://github.com/acados/tera_renderer/releases/download/v${want}/t_renderer-v${want}-linux-${TERA_ARCH}"
    else
        if ! command -v cargo >/dev/null; then
            echo "race-stack: building t_renderer on aarch64 needs cargo (Rust):" >&2
            echo "    curl https://sh.rustup.rs -sSf | sh" >&2
            return 1
        fi
        say "building t_renderer ${want} with cargo"
        local root="${ACADOS_ROOT}/src/tera_renderer-${want}"
        cargo install --locked --git https://github.com/acados/tera_renderer.git \
            --tag "v${want}" --root "$root" t_renderer
        cp "${root}/bin/t_renderer" "$tmp"
    fi
    chmod +x "$tmp"
    mv "$tmp" "${ACADOS_ROOT}/bin/t_renderer"
    if ! tera_ok; then
        echo "race-stack: t_renderer at ${ACADOS_ROOT}/bin reports" \
             "'$("${ACADOS_ROOT}/bin/t_renderer" --version 2>&1)', expected ${want}" >&2
        return 1
    fi
}

# --- venv --------------------------------------------------------------------
install_venv() {
    local py="/usr/bin/python${PY_VERSION}"
    [[ -x "$py" ]] || py="$PY_VERSION"
    if [[ ! -x "$VPY" ]]; then
        say "creating venv ${VENV} (python ${PY_VERSION})"
        if command -v uv >/dev/null; then
            uv venv --python "$py" "$VENV"
        else
            "python${PY_VERSION}" -m venv "$VENV"
        fi
    fi
    say "installing race_planner and acados_template into ${VENV}"
    if command -v uv >/dev/null; then
        uv pip install --python "$VPY" \
            -e "${PLANNER_DIR}[test]" \
            -e "${ACADOS_SRC}/interfaces/acados_template"
    else
        "$VPY" -m pip install \
            -e "${PLANNER_DIR}[test]" \
            -e "${ACADOS_SRC}/interfaces/acados_template"
    fi
}

# A rerun with everything present writes nothing, so the env file keeps the SWB
# it was written with; delete it and rerun to write it again.
changed=0
acados_ok || { install_acados; changed=1; }
tera_ok   || { install_tera;   changed=1; }
venv_ok   || { install_venv;   changed=1; }
if ((changed)) || [[ ! -f "$ENV_FILE" ]]; then
    write_env
fi
check
