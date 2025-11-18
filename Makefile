.PHONY: default setup prepare build test launch stop restart status controller test-control launch_camera_calibration clean checkout start-simulation stop-simulation status-simulation logs-simulation
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
	@echo 'make play-basic-control'
	@echo '    Launch vehicle control test with tmux (system + controller + monitor).'
	@echo
	@echo 'make launch_camera_calibration'
	@echo '    Launch camera calibration with ZED camera and calibrator.'
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

test:
	@source /opt/ros/humble/setup.bash && \
	colcon test \
		--base-paths src \
		--return-code-on-test-failure; \
	TEST_EXIT_CODE=$$?; \
	echo "" && \
	colcon test-result --verbose; \
	exit $$TEST_EXIT_CODE

launch:
	ros2 launch autosdv_launch autosdv.launch.yaml

play:
	play_launch launch autosdv_launch autosdv.launch.yaml

stop:
	ros2 systemd stop autosdv

restart: start

status:
	ros2 systemd status autosdv

logs:
	ros2 systemd logs autosdv

run-controller:
	source install/setup.bash && \
	ros2 run control_test keyboard_control

run-plogjuggler:
	source install/setup.bash && \
	ros2 run plotjuggler plotjuggler

play-basic-control:
	play_launch launch control_test basic_control.launch.xml

clean:
	@while true; do \
		read -p 'Are you sure to clean up? (yes/no) ' yn; \
		case $$yn in \
			yes ) rm -rf build install log; break;; \
			no ) break;; \
			* ) echo 'Please enter yes or no.';; \
		esac \
	done

start-simulation:
	systemd-run --user \
		--unit=autosdv-simulation \
		--working-directory=$(PWD) \
		--setenv=CYCLONEDDS_URI="file://$(PWD)/cyclonedds.xml" \
		--setenv=RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
		/bin/bash -c "source /opt/ros/humble/setup.bash && source $(PWD)/install/setup.bash && \
		ros2 launch autoware_launch logging_simulator.launch.xml \
			map_path:=data/COSS-map-planning/ \
			vehicle_model:=autosdv_vehicle \
			sensor_model:=autosdv_sensor_kit"

stop-simulation:
	systemctl --user stop autosdv-simulation

status-simulation:
	systemctl --user status autosdv-simulation

logs-simulation:
	journalctl --user -u autosdv-simulation -f
