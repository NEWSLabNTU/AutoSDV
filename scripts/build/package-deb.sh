#!/usr/bin/env bash

source /opt/ros/humble/setup.bash
script_dir=$(cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)

top_work_dir=build_deb
success_file="$top_work_dir/success"
fail_file="$top_work_dir/fail"
make_deb_file="$script_dir/make-deb.sh"
# rosdep_file="$top_work_dir/rosdep.yaml"

mkdir -p "$top_work_dir"

# colcon list | cut -f1 | \
#     while read -r pkg_name; do
# 	pkg_name_dashed="${pkg_name//_/-}"
# 	echo "${pkg_name}:
#   ubuntu:
#     packages: [ros-$ROS_DISTRO-${pkg_name_dashed}]"
#     done > "$rosdep_file"

# sudo cp -v "$rosdep_file" /etc/ros/rosdep/sources.list.d/99-f1eighth.list

colcon list --topological-order | cut -f1-2 | \
    while read -r pkg_name pkg_dir; do

	pkg_name_dashed="${pkg_name//_/-}"
	pkg_work_dir="$(realpath $top_work_dir/$pkg_name)"
	out_file="$pkg_work_dir/out"
	err_file="$pkg_work_dir/err"
	status_file="$pkg_work_dir/status"
	
	mkdir -p "$pkg_work_dir"

	deb_path=$(find "$pkg_work_dir" -name ros-${ROS_DISTRO}-${pkg_name_dashed}_'*'.deb | head -n1)
	if [ -n "$deb_path" ]; then
	    echo "SKIP $pkg_name"
	    continue
	fi
	echo "BUILD $pkg_name"
	
	(
	    "$make_deb_file" "$pkg_name" "$pkg_dir" "$pkg_work_dir" && \
		sudo dpkg -i $pkg_work_dir/ros-$ROS_DISTRO-"${pkg_name_dashed}"_*.deb && \
		sudo apt-get install -f
	)  > "$out_file" 2> "$err_file"
	status=$?
	echo $status > "$status_file"
	
	if [ $status -eq 0 ]; then
	    echo "SUCCESS $pkg_name"
	    echo "$pkg_name $pkg_dir" >> "$success_file"
	else
	    echo "FAIL $pkg_name"
	    echo "$pkg_name $pkg_dir" >> "$fail_file"
	    exit 1
	fi
    done
