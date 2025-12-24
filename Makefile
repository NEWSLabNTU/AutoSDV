SHELL := /bin/bash

.PHONY: default
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
	@echo '    Launch AutoSDV system with web UI at http://localhost:8081'
	@echo '    Logs are saved to play_log/latest/ directory.'
	@echo '    Use Ctrl+C to stop the system.'
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
	@echo 'make start-simulation'
	@echo '    Start Autoware logging simulator using systemd.'
	@echo
	@echo 'make stop-simulation'
	@echo '    Stop the running simulation.'
	@echo
	@echo 'make status-simulation'
	@echo '    Show simulation status.'
	@echo
	@echo 'make logs-simulation'
	@echo '    Follow simulation logs.'
	@echo
	@echo 'make run-rviz'
	@echo '    Launch RViz with AutoSDV configuration.'
	@echo
	@echo 'make clean'
	@echo '    Clean up built binaries.'

.PHONY: checkout
checkout:
	git submodule update --init --recursive --checkout

.PHONY: setup
setup:
	./scripts/setup-dev-env/setup-dev-env.sh

.PHONY: prepare
prepare:
	source /opt/ros/humble/setup.sh && \
	rosdep update --rosdistro=humble && \
	rosdep install -y --from-paths src --ignore-src -r

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
	play_launch launch \
		--web-ui \
		--web-ui-addr 0.0.0.0 \
		--web-ui-port 8081 \
		autosdv_launch autosdv.launch.yaml

.PHONY: launch-zed-only
launch-zed-only:
	play_launch launch \
		--web-ui \
		--web-ui-addr 0.0.0.0 \
		--web-ui-port 8081 \
		zed_wrapper zed_camera.launch.py camera_model:=zedxm

.PHONY: run-controller
run-controller:
	source install/setup.bash && \
	ros2 run control_test keyboard_control

.PHONY: run-plogjuggler
run-plogjuggler:
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

.PHONY: start-simulation
start-simulation:
	systemd-run --user \
		--unit=autosdv-simulation \
		--working-directory=$(PWD) \
		--setenv=CYCLONEDDS_URI="file://$(PWD)/cyclonedds.xml" \
		--setenv=RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
		/bin/bash -c "source /opt/ros/humble/setup.bash && source $(PWD)/install/setup.bash && \
		play_launch launch autoware_launch logging_simulator.launch.xml \
			map_path:=data/COSS-map-planning/ \
			vehicle_model:=autosdv_vehicle \
			sensor_model:=autosdv_sensor_kit"

.PHONY: stop-simulation
stop-simulation:
	systemctl --user stop autosdv-simulation

.PHONY: status-simulation
status-simulation:
	systemctl --user status autosdv-simulation

.PHONY: logs-simulation
logs-simulation:
	journalctl --user -u autosdv-simulation -f

.PHONY: run-rviz
run-rviz:
	rviz2 -d ./src/launcher/autosdv_launch/rviz/autosdv.rviz
