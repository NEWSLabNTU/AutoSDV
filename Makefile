.PHONY: default setup prepare build launch clean checkout
SHELL := /bin/bash

default:
	@echo 'make prepare'
	@echo '    Install required dependencies for this project.'
	@echo
	@echo 'make build'
	@echo '    Build this project.'
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
		--symlink-install \
		--cmake-args -DCMAKE_BUILD_TYPE=Release

launch:
	source install/setup.bash && \
	ros2 launch f1eighth_launch f1eighth.launch.yaml

clean:
	@while true; do \
		read -p 'Are you sure to clean up? (yes/no) ' yn; \
		case $$yn in \
			yes ) rm -rf build install log; break;; \
			no ) break;; \
			* ) echo 'Please enter yes or no.';; \
		esac \
	done
