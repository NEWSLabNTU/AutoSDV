SHELL := /bin/bash

.PHONY: default
default:
	@echo 'make setup'
	@echo '    Run interactive setup (installs ROS 2, dependencies, etc.)'
	@echo
	@echo 'make build'
	@echo '    Build this project.'
	@echo
	@echo 'make test'
	@echo '    Run tests for packages in src/ directory.'
	@echo
	@echo 'make launch'
	@echo '    Launch AutoSDV system with web UI at http://localhost:8081'
	@echo '    Logs are saved to play_log/latest/ directory.'
	@echo '    Use Ctrl+C to stop the system.'
	@echo
	@echo 'make checkout'
	@echo '    Initialize and update all git submodules.'
	@echo
	@echo 'make run-controller'
	@echo '    Launch manual keyboard control.'
	@echo
	@echo 'make play-basic-control'
	@echo '    Launch vehicle control test (basic_control.launch.xml).'
	@echo
	@echo 'make run-straight-10m'
	@echo '    Run trajectory player with straight_10m.yaml (10m straight line).'
	@echo
	@echo 'make run-circle'
	@echo '    Run trajectory player with circle.yaml (circular path).'
	@echo
	@echo 'make run-rviz'
	@echo '    Launch RViz with AutoSDV configuration.'
	@echo
	@echo 'make run-plotjuggler'
	@echo '    Launch PlotJuggler for data visualization.'
	@echo
	@echo 'make record-outdoor'
	@echo '    Record outdoor sensor topics to rosbags/ directory.'
	@echo
	@echo 'make play-outdoor'
	@echo '    Play the most recent outdoor recording.'
	@echo
	@echo 'make run-drive'
	@echo '    Run autonomous driving with poses from scripts/testing/drive/poses.json.'
	@echo
	@echo 'make launch-planning-simulation'
	@echo '    Launch Autoware planning simulator with AutoSDV vehicle.'
	@echo
	@echo 'make launch-logging-simulation'
	@echo '    Launch logging simulation for rosbag replay testing.'
	@echo
	@echo 'make launch-zed-only'
	@echo '    Launch only ZED camera node for testing.'
	@echo
	@echo 'make test-logging-simulation'
	@echo '    Run full logging simulation test (launch + rosbag + drive + record).'
	@echo
	@echo 'make lio-sam-mapping'
	@echo '    Launch LIO-SAM mapping for point cloud map creation.'
	@echo '    Use ARGS="..." to pass launch arguments (e.g., ARGS="lidar_model:=vlp32c").'
	@echo
	@echo 'make record-mapping'
	@echo '    Record rosbag for offline LIO-SAM mapping.'
	@echo
	@echo 'make clean'
	@echo '    Clean up built binaries.'

.PHONY: checkout
checkout:
	git submodule update --init --recursive --checkout

.PHONY: setup
setup:
	./setup.sh

.PHONY: build
build:
	source /opt/ros/humble/setup.bash && \
	colcon build \
		--base-paths src \
		--symlink-install \
		--cmake-args -DCMAKE_BUILD_TYPE=Release

.PHONY: test
test:
	@source /opt/ros/humble/setup.bash && \
	colcon test \
		--base-paths src \
		--return-code-on-test-failure; \
	TEST_EXIT_CODE=$$?; \
	echo "" && \
	colcon test-result --verbose; \
	exit $$TEST_EXIT_CODE

.PHONY: launch
launch:
	@if [ -n "$$DISPLAY" ]; then \
		play_launch launch \
			--web-ui \
			--web-ui-addr 0.0.0.0 \
			--web-ui-port 8081 \
			autosdv_launch autosdv.launch.yaml; \
	else \
		play_launch launch \
			--web-ui \
			--web-ui-addr 0.0.0.0 \
			--web-ui-port 8081 \
			autosdv_launch autosdv.launch.yaml \
			rviz:=false; \
	fi

.PHONY: launch-planning-simulation
launch-planning-simulation:
	play_launch launch \
		--web-ui \
		--web-ui-addr 0.0.0.0 \
		--web-ui-port 8081 \
		autoware_launch planning_simulator.launch.xml \
		map_path:=$(PWD)/data/COSS-map-planning \
		vehicle_model:=autosdv_vehicle \
		sensor_model:=autosdv_sensor_kit


.PHONY: launch-zed-only
launch-zed-only:
	play_launch launch \
		--web-ui \
		--web-ui-addr 0.0.0.0 \
		--web-ui-port 8081 \
		zed_wrapper zed_camera.launch.py camera_model:=zedxm

.PHONY: launch-logging-simulation
launch-logging-simulation:
	@if [ -n "$$DISPLAY" ]; then \
		play_launch launch \
			--web-ui \
			--web-ui-addr 0.0.0.0 \
			--web-ui-port 8081 \
			autosdv_launch logging_simulation.launch.yaml; \
	else \
		play_launch launch \
			--web-ui \
			--web-ui-addr 0.0.0.0 \
			--web-ui-port 8081 \
			autosdv_launch logging_simulation.launch.yaml \
			rviz:=false; \
	fi


.PHONY: run-controller
run-controller:
	source install/setup.bash && \
	ros2 run control_test keyboard_control

.PHONY: run-plotjuggler
run-plotjuggler:
	source install/setup.bash && \
	ros2 run plotjuggler plotjuggler

.PHONY: play-basic-control
play-basic-control:
	play_launch launch control_test basic_control.launch.xml

.PHONY: run-straight-10m
run-straight-10m:
	source install/setup.bash && \
	ros2 run control_test trajectory_player --ros-args -p trajectory_file:=straight_10m.yaml

.PHONY: run-circle
run-circle:
	source install/setup.bash && \
	ros2 run control_test trajectory_player --ros-args -p trajectory_file:=circle.yaml

.PHONY: clean
clean:
	@while true; do \
		read -p 'Are you sure to clean up? (yes/no) ' yn; \
		case $$yn in \
			yes ) rm -rf build install log; break;; \
			no ) break;; \
			* ) echo 'Please enter yes or no.';; \
		esac \
	done

.PHONY: run-rviz
run-rviz:
	rviz2 -d ./src/launcher/autosdv_launch/rviz/autosdv.rviz

.PHONY: record-outdoor
record-outdoor:
	./scripts/record_outdoor.sh

.PHONY: play-outdoor
play-outdoor:
	@LATEST=$$(ls -td rosbags/outdoor_* 2>/dev/null | head -1); \
	if [ -z "$$LATEST" ]; then \
		echo "No outdoor recordings found in rosbags/"; \
		exit 1; \
	fi; \
	echo "Playing: $$LATEST"; \
	ros2 bag play "$$LATEST" --clock

.PHONY: test-logging-simulation
test-logging-simulation:
	source install/setup.bash && \
	parallel --line-buffer ::: \
		"$(MAKE) launch-logging-simulation" \
		"sleep 40 && ros2 bag play rosbags/outdoor_20251226_153115/ --clock -l -r 1.0" \
		"sleep 45 && ./scripts/record_localization.sh"

.PHONY: run-drive
run-drive:
	source install/setup.bash && \
	python3 ./scripts/testing/drive/run.py

# LIO-SAM Mapping Targets
.PHONY: lio-sam-mapping
lio-sam-mapping:
	@echo "Launching LIO-SAM mapping..."
	@echo "Use ARGS to pass launch arguments, e.g.:"
	@echo "  make lio-sam-mapping ARGS=\"lidar_model:=vlp32c imu_source:=mpu9250\""
	@echo ""
	source install/setup.bash && \
	ros2 launch autosdv_launch lio_sam_mapping.launch.xml $(ARGS)

.PHONY: record-mapping
record-mapping:
	@echo "Recording rosbag for LIO-SAM mapping..."
	@echo "Recording topics: LiDAR, IMU, GNSS, TF"
	@echo "Press Ctrl+C to stop recording."
	@echo ""
	@mkdir -p rosbags
	@TIMESTAMP=$$(date +%Y%m%d_%H%M%S); \
	source install/setup.bash && \
	ros2 bag record \
		/sensing/lidar/concatenated/pointcloud \
		/sensing/imu/imu_data \
		/sensing/gnss/garmin/fix \
		/sensing/gnss/ublox/nav_sat_fix \
		/sensing/gnss/septentrio/nav_sat_fix \
		/tf /tf_static \
		-o rosbags/mapping_$$TIMESTAMP
