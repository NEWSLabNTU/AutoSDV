.PHONY: default setup prepare build test launch stop restart status launch_camera_calibration clean checkout
SHELL := /bin/bash

default:
	@echo 'make prepare'
	@echo '    Install required dependencies for this project.'
	@echo
	@echo 'make build'
	@echo '    Build this project.'
	@echo
	@echo 'make test'
	@echo '    Run tests for packages in src/ directory.'
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

#setup:
#	./scripts/setup-dev-env/setup-dev-env.sh

#prepare: setup #checkout
# 	sudo cp scripts/rosdep/*.yaml /etc/ros/rosdep/sources.list.d/
# 	rosdep update
# 	source /opt/ros/humble/setup.sh && 
# 	rosdep install -y --from-paths src --ignore-src -r --rosdistro=humble

build:
	@/bin/bash -c ' \
		set -e; \
		echo "--- Cleaning up old build directories ---"; \
		rm -rf build install log; \
		echo "--- Sourcing ROS environment ---"; \
		source /opt/ros/humble/setup.bash; \
		echo "--- Starting colcon build with ZED SDK path ---"; \
		if [ -d /usr/local/zed ]; then \
			echo "ZED SDK found at /usr/local/zed"; \
			echo "Building with ZED SDK 5.0 (compatible with zed-ros2-wrapper humble-v5.0.0)"; \
			echo "Skipping seyond package (x86-64 binary incompatible with ARM64)"; \
			colcon build \
				--base-paths src \
				--symlink-install \
				--packages-skip seyond \
				--cmake-args \
					-DCMAKE_BUILD_TYPE=Release \
					-DCMAKE_PREFIX_PATH="/usr/local/zed:$$CMAKE_PREFIX_PATH" \
					-DZED_DIR="/usr/local/zed" \
					-DCMAKE_MODULE_PATH="/usr/local/zed/cmake"; \
		else \
			echo "ZED SDK not found, skipping ZED-related packages"; \
			echo "Skipping seyond package (x86-64 binary incompatible with ARM64)"; \
			colcon build \
				--base-paths src \
				--symlink-install \
				--packages-skip \
					zed_components \
					zed_wrapper \
					zed_ros2 \
					autoware_zed_converter \
					autoware_zed_launch \
					zed_launch \
					seyond \
				--cmake-args \
					-DCMAKE_BUILD_TYPE=Release; \
		fi \
	'

test:
	@source /opt/ros/humble/setup.bash && \
	colcon test \
		--base-paths src \
		--return-code-on-test-failure; \
	TEST_EXIT_CODE=$$?; \
	echo "" && \
	colcon test-result --verbose; \
	exit $$TEST_EXIT_CODE

# Launch for testing without hardware (Docker on x86 PC)
launch-sim:
	@echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
	@echo "🐳 Launching in SIMULATION mode (no hardware required)"
	@echo "   launch_rviz:=false (GUI disabled)"
	@echo "   launch_vehicle:=false (no GPIO hardware)"
	@echo "   launch_sensing_driver:=false (no sensors)"
	@echo "   launch_perception:=false (no GPU/TensorRT)"
	@echo "   use_gnss:=false (no GPS hardware)"
	@echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
	@/bin/bash -c ' \
		set -e; \
		source install/setup.bash; \
		export RMW_IMPLEMENTATION=rmw_fastrtps_cpp; \
		ros2 launch autosdv_launch autosdv.launch.yaml \
			launch_rviz:=false \
			launch_vehicle:=false \
			launch_sensing_driver:=false \
			launch_perception:=false \
			use_gnss:=false \
	'

# Launch for real Jetson Orin with hardware
launch-hw:
	@echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
	@echo "🚗 Launching on HARDWARE (Jetson Orin with sensors)"
	@echo "   launch_rviz:=true (GUI enabled)"
	@echo "   launch_vehicle:=true (GPIO hardware available)"
	@echo "   launch_sensing_driver:=true (real sensors connected)"
	@echo "   launch_perception:=true (GPU/TensorRT available)"
	@echo "   use_gnss:=true (GPS hardware connected)"
	@echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
	@/bin/bash -c ' \
		set -e; \
		source install/setup.bash; \
		export RMW_IMPLEMENTATION=rmw_fastrtps_cpp; \
		ros2 launch autosdv_launch autosdv.launch.yaml \
			launch_rviz:=true \
			launch_vehicle:=true \
			launch_sensing_driver:=true \
			launch_perception:=true \
			use_gnss:=true \
	'

# Auto-detect: Check if running on REAL Jetson hardware (not Docker emulation)
launch:
	@echo "Detecting hardware environment..."; \
	if [ -f /proc/device-tree/model ] && grep -q "NVIDIA" /proc/device-tree/model 2>/dev/null; then \
		echo "🤖 Real Jetson hardware detected (device tree found)!"; \
		$(MAKE) launch-hw; \
	elif [ -d /sys/devices/platform ] && ls /sys/devices/platform/ 2>/dev/null | grep -q "tegra\|nvidia"; then \
		echo "🤖 Real Jetson hardware detected (Tegra platform devices found)!"; \
		$(MAKE) launch-hw; \
	elif [ -d /sys/class/gpio/gpiochip* ] 2>/dev/null && [ -f /etc/nv_tegra_release ]; then \
		echo "🤖 Potential Jetson hardware detected (GPIO + L4T found)"; \
		$(MAKE) launch-hw; \
	elif [ -f /etc/nv_tegra_release ]; then \
		echo "🐳 L4T Docker image detected (no real Jetson hardware) - using simulation mode"; \
		$(MAKE) launch-sim; \
	else \
		echo "🐳 Non-Jetson environment detected - using simulation mode"; \
		$(MAKE) launch-sim; \
	fi

start:
	ros2 systemd launch \
		--replace \
		--name autosdv \
		--source $(PWD)/setup.sh \
		--rmw cyclonedds \
		--env CYCLONEDDS_URI="file://$(PWD)/cyclonedds.xml" \
		autosdv_launch autosdv.launch.yaml

stop:
	ros2 systemd stop autosdv

restart: start

status:
	ros2 systemd status autosdv

logs:
	ros2 systemd logs autosdv

clean:
	@while true; do \
		read -p 'Are you sure to clean up? (yes/no) ' yn; \
		case $$yn in \
			yes ) rm -rf build install log; break;; \
			no ) break;; \
			* ) echo 'Please enter yes or no.';; \
		esac \
	done
