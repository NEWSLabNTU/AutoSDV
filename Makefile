.PHONY: default setup prepare build test launch stop restart status controller test-control launch_camera_calibration clean checkout
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
	@echo 'make test-control'
	@echo '    Launch vehicle control test with tmux (system + controller + monitor).'
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

controller:
	source install/setup.bash && \
	ros2 run autoware_manual_control keyboard_control

test-control:
	@if ! command -v tmux &> /dev/null; then \
		echo "Error: tmux is not installed. Please install it with: sudo apt install tmux"; \
		exit 1; \
	fi; \
	if tmux has-session -t autosdv-control-test 2>/dev/null; then \
		echo "Session 'autosdv-control-test' already exists. Attaching..."; \
		tmux attach-session -t autosdv-control-test; \
	else \
		echo "Creating new tmux session 'autosdv-control-test'..."; \
		tmux new-session -d -s autosdv-control-test -n launch "cd $(PWD) && make launch"; \
		tmux new-window -t autosdv-control-test:1 -n controller "cd $(PWD) && source install/setup.bash && ros2 run autoware_manual_control keyboard_control"; \
		tmux new-window -t autosdv-control-test:2 -n monitor "cd $(PWD) && source install/setup.bash && ros2 topic echo /control/command/control_cmd"; \
		tmux select-window -t autosdv-control-test:1; \
		tmux attach-session -t autosdv-control-test; \
	fi

clean:
	@while true; do \
		read -p 'Are you sure to clean up? (yes/no) ' yn; \
		case $$yn in \
			yes ) rm -rf build install log; break;; \
			no ) break;; \
			* ) echo 'Please enter yes or no.';; \
		esac \
	done
