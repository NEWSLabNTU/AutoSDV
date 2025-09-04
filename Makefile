.PHONY: default setup prepare build launch launch_camera_calibration clean checkout
SHELL := /bin/bash

default:
	@echo 'make prepare'
	@echo '    Install required dependencies for this project.'
	@echo
	@echo 'make build'
	@echo '    Build this project.'
	@echo
	@echo 'make launch_camera_calibration'
	@echo '    Launch camera calibration with ZED camera and calibrator.'
	@echo
	@echo 'make clean'
	@echo '    Clean up built binaries.'

checkout:
	git submodule update --init --recursive --checkout

setup:
	./scripts/setup-dev-env/setup-dev-env.sh

prepare:
	source /opt/ros/humble/setup.sh && \
	rosdep update --rosdistro=humble && \
	rosdep install -y --from-paths src --ignore-src -r

build:
	source /opt/ros/humble/setup.bash && \
	colcon build \
		--base-paths src \
		--symlink-install \
		--cmake-args -DCMAKE_BUILD_TYPE=Release

launch:
	@./launch.sh

launch_camera_calibration:
	. install/setup.sh && \
	ros2 launch autosdv_launch camera_calibration.launch.xml

controller:
	. install/setup.sh && \
	ros2 run autoware_manual_control keyboard_control --ros-args --remap /external/selected/control_cmd:=/control/command/control_cmd

clean:
	@while true; do \
		read -p 'Are you sure to clean up? (yes/no) ' yn; \
		case $$yn in \
			yes ) rm -rf build install log; break;; \
			no ) break;; \
			* ) echo 'Please enter yes or no.';; \
		esac \
	done
