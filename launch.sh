#!/usr/bin/env bash
set -e
script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd "$script_dir"

export RCUTILS_COLORIZED_OUTPUT=1  # 1 for forcing it


source install/setup.bash
ros2 launch autosdv_launch autosdv.launch.yaml
