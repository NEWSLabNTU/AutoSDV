.PHONY: default setup prepare build launch stop restart status launch_camera_calibration clean checkout
SHELL := /bin/bash

default:
	@echo 'make prepare'
	@echo '    Install required dependencies for this project.'
	@echo
	@echo 'make build'
	@echo '    Build this project.'
	@echo
	@echo 'make launch'
	@echo '    Launch AutoSDV system using systemd service.'
	@echo
	@echo 'make stop'
	@echo '    Stop the running AutoSDV system.'
	@echo
	@echo 'make restart'
	@echo '    Restart the AutoSDV system.'
	@echo
	@echo 'make status'
	@echo '    Show AutoSDV system status and logs.'
	@echo
	@echo 'make controller'
	@echo '    Launch manual keyboard control.'
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
	@# Source the workspace to make autosdv available
	@source install/setup.bash 2>/dev/null || { echo "Error: Workspace not built. Please run 'make build' first"; exit 1; }; \
	if ! command -v autosdv &> /dev/null; then \
		echo "Error: autosdv command not found. Please build the project first with 'make build'"; \
		exit 1; \
	fi; \
	if ! systemctl --user list-unit-files | grep -q "^autosdv.service"; then \
		echo "AutoSDV service not installed. Installing..."; \
		autosdv install || { echo "Failed to install AutoSDV service"; exit 1; }; \
	fi; \
	if systemctl --user is-active autosdv &>/dev/null; then \
		echo "AutoSDV is already running."; \
		echo "Use 'make stop' to stop or 'make restart' to restart."; \
	else \
		echo "Starting AutoSDV service..."; \
		autosdv start || { echo "Failed to start AutoSDV service"; exit 1; }; \
		echo ""; \
		echo "AutoSDV started successfully!"; \
		echo "  • System monitor: http://localhost:8080/"; \
		echo "  • View logs: make status"; \
		echo "  • Stop system: make stop"; \
	fi

stop:
	@# Stop the AutoSDV service
	@source install/setup.bash 2>/dev/null || { echo "Error: Workspace not built. Please run 'make build' first"; exit 1; }; \
	if command -v autosdv &> /dev/null; then \
		autosdv stop && echo "AutoSDV service stopped."; \
	else \
		echo "Error: autosdv command not found."; \
		exit 1; \
	fi

restart:
	@# Restart the AutoSDV service
	@source install/setup.bash 2>/dev/null || { echo "Error: Workspace not built. Please run 'make build' first"; exit 1; }; \
	if command -v autosdv &> /dev/null; then \
		autosdv restart && echo "AutoSDV service restarted."; \
	else \
		echo "Error: autosdv command not found."; \
		exit 1; \
	fi

status:
	@# Show AutoSDV service status
	@source install/setup.bash 2>/dev/null || { echo "Error: Workspace not built. Please run 'make build' first"; exit 1; }; \
	if command -v autosdv &> /dev/null; then \
		autosdv status; \
	else \
		echo "Error: autosdv command not found."; \
		exit 1; \
	fi

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
