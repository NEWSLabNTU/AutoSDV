# Put the TensorRT the Autoware Debians were compiled against ahead of whatever
# else this host has. `source scripts/trt-runtime-env.sh` from a bash shell;
# scripts/env.sh already does it, and so does `just build-engines`.
#
# A different patch of TensorRT 10 loads perfectly and then throws away every
# cached .engine on startup -- `autoware_tensorrt_common` compares the plan's
# recorded version against the macros baked into its own build and rebuilds on
# any mismatch. The rebuild is silent apart from one warning line, so the only
# symptom is that perception takes minutes to come up, every single time. See
# `nvidia_amd64.tensorrt_engine_abi` in versions.yaml.
#
# No-op on arm64 (JetPack's TensorRT is the one the arm64 Debians were built
# against) and no-op when the prefix is absent, so sourcing this is always safe;
# `setup.sh` step `tensorrt-runtime` is what creates it.

if [ "$(uname -m)" = "x86_64" ]; then
    _autosdv_trt_repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
    _autosdv_trt_version="$(
        "${_autosdv_trt_repo}/scripts/version/get-version.sh" \
            nvidia_amd64.tensorrt_engine_abi 2>/dev/null
    )"
    if [ -n "${_autosdv_trt_version}" ] &&
        [ -f "/opt/tensorrt/${_autosdv_trt_version}/lib/libnvinfer.so.${_autosdv_trt_version}" ]; then
        export LD_LIBRARY_PATH="/opt/tensorrt/${_autosdv_trt_version}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
    fi
    unset _autosdv_trt_repo _autosdv_trt_version
fi
