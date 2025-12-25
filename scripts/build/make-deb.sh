#!/usr/bin/env bash
set -e

print_usage() {
    echo "Usage: $0 PKG_NAME PKG_DIR OUTPUT_DIR" >&2
    exit 1
}

pkg_name="$1"
shift || print_usage

pkg_dir="$1"
shift || print_usage

output_dir="$1"
shift || print_usage

pkg_name_dashed="${pkg_name//_/-}"
output_dir="$(realpath $output_dir)"

source /opt/ros/humble/setup.bash
cd "$pkg_dir"

rm -rf debian .obj-* ros-$ROS_DISTRO-"${pkg_name_dashed}"_*.deb
bloom-generate rosdebian --ros-distro "$ROS_DISTRO"
fakeroot debian/rules binary

deb_path=$(find .. -name ros-${ROS_DISTRO}-${pkg_name_dashed}_'*'.deb | head -n1)
if [ -n "$deb_path" ]; then
    mv "$deb_path" "$output_dir"
fi

ddeb_path=$(find .. -name ros-${ROS_DISTRO}-${pkg_name_dashed}-dbgsym_'*'.ddeb | head -n1)
if [ -n "$ddeb_path" ]; then
    mv "$ddeb_path" "$output_dir"
fi
