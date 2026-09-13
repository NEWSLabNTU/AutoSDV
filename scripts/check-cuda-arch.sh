#!/usr/bin/env bash
# Can the selected CUDA toolkit compile for the GPU in this machine?
#
# cuda_ndt does not ship precompiled kernels. cubecl reads the device's compute
# capability at run time and hands NVRTC `--gpu-architecture=sm_<cc>`, so the
# toolkit that is *loaded* has to know that architecture. When it does not, the
# matcher panics on the first scan with
#
#     nvrtc: error: invalid value for --gpu-architecture (-arch)
#
# and nothing downstream publishes -- not the matcher, not even the EKF -- while
# the launcher reports every node ready. This check turns that into a sentence.
#
# Exit status: 0 usable or not applicable, 1 the toolkit is too old for the GPU.
set -uo pipefail

if ! command -v nvidia-smi >/dev/null 2>&1; then
    echo "no NVIDIA GPU detected; cuda_ndt is not available, use pose_source:=ndt"
    exit 0
fi

# Lowest compute capability among the installed GPUs: the kernel has to compile
# for whichever one CUDA hands the process, and cubecl does not choose.
cc="$(nvidia-smi --query-gpu=compute_cap --format=csv,noheader 2>/dev/null | tr -d ' .' | sort -n | head -1)"
if [[ -z "$cc" ]]; then
    echo "could not read the GPU compute capability from nvidia-smi"
    exit 0
fi

# The ecosystem convention, and the same order .envrc uses: an explicit choice
# first, the administrator's symlink last.
cuda_home="${CUDA_HOME:-${CUDA_PATH:-/usr/local/cuda}}"
nvcc="$cuda_home/bin/nvcc"
if [[ ! -x "$nvcc" ]]; then
    nvcc="$(command -v nvcc 2>/dev/null || true)"
fi
if [[ -z "$nvcc" ]]; then
    echo "no nvcc found under $cuda_home or on PATH; cannot tell whether the toolkit"
    echo "supports sm_$cc. Install a CUDA toolkit or set CUDA_HOME."
    exit 0
fi

ver="$("$nvcc" --version | sed -n 's/.*release \([0-9.]*\).*/\1/p')"
if "$nvcc" --list-gpu-arch 2>/dev/null | grep -qx "compute_$cc"; then
    echo "CUDA $ver at $(dirname "$(dirname "$nvcc")") supports sm_$cc (this GPU)"
    exit 0
fi

echo "CUDA $ver cannot compile for sm_$cc, which is what this GPU reports."
echo "cuda_ndt will panic on its first scan; pose_source:=ndt is unaffected."

# Name the way out, if the host already has one. Toolkits are conventionally
# /usr/local/cuda-<version>, and the symlink without a version is the current
# selection, so skip it to avoid naming the same tree twice.
found=""
for dir in /usr/local/cuda-*; do
    [[ -x "$dir/bin/nvcc" ]] || continue
    if "$dir/bin/nvcc" --list-gpu-arch 2>/dev/null | grep -qx "compute_$cc"; then
        found="$found $dir"
    fi
done

if [[ -n "$found" ]]; then
    echo
    echo "Installed toolkits that do support it:$found"
    echo
    echo "Select one for this workspace (direnv re-reads .envrc):"
    echo "    CUDA_HOME=${found## } direnv reload"
    echo
    echo "Or system-wide, if it is registered with the alternatives system:"
    echo "    sudo update-alternatives --config cuda"
else
    echo
    echo "No installed toolkit supports sm_$cc. sm_120 (Blackwell) needs CUDA 12.8"
    echo "or newer; sm_90 needs 11.8 or newer."
fi
exit 1
