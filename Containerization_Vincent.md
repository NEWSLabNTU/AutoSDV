### Issue : X11 Authorization for RViz
## Root Cause : Docker container lacks permission to access X11 display server
## Solution : 
@if command -v xhost >/dev/null 2>&1; then \
	echo "Enabling X11 access for Docker containers..."; \
	xhost +local:docker >/dev/null 2>&1 || true; \
fi

========================================================================
========================================================================

### Issue : ROS 2 Packages Not Found During Build (Inside Docker build process)
## Root Cause: ROS 2 packages installed before ROS repository was configured by `setup-dev-env.sh`
## Solution : Modify the dockerfile ,then rebuild the docker image.

RUN apt-get install -y opencv-contrib-python i2c-tools wget
RUN cd /AutoSDV && ./setup-dev-env.sh
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    apt-get install -y ros-humble-cv-bridge ros-humble-vision-opencv"

========================================================================
========================================================================

### Issue : ZED SDK Installation for JetPack 6.0 (Docker container during build)
## Root Cause : 
## Solution : Modify the dockerfile ,then rebuild the docker image.
RUN apt-get update && apt-get install -y wget lsb-release zstd
RUN wget -q -O ZED_SDK_Linux.run \
    https://download.stereolabs.com/zedsdk/4.0/l4t36.3/jetsons && \
    chmod +x ZED_SDK_Linux.run && \
    ./ZED_SDK_Linux.run -- silent skip_tools skip_cuda && \
    rm ZED_SDK_Linux.run

========================================================================
========================================================================

### Issue : Seyond LiDAR SDK Architecture Mismatch
## Error : 
```
/usr/bin/ld: libinnolidarsdkclient.a: error adding symbols: file in wrong format
Relocations in generic ELF (EM: 62)
```
## Root Cause: Precompiled x86-64 static libraries (`.a` files) in seyond SDK cannot link to ARM64 build
## Solution (Added seyond SDK compilation step):
```
# Force rebuild SDK from source for ARM64
RUN cd /AutoSDV/src/drivers/seyond/robin_ros2/inno-lidar-sdk && \
    rm -f lib/*.a && \
    cd build && \
    bash build_unix.sh
```

========================================================================
========================================================================

### Issue : ZED SDK Not Found During CMake Configuration
## Error :
```
CMake Error: Could not find a package configuration file provided by "ZED"
```

## Root Cause: ZED SDK installed but `CMAKE_PREFIX_PATH` not set
## Solution (Added ZED SDK path to CMake arguments) :
```
build:
	colcon build --symlink-install \
		--cmake-args -DCMAKE_PREFIX_PATH="/usr/local/zed"
```

========================================================================
========================================================================

### Issue : ROS 2 DDS Communication Failure - Topics Not Discovered
## Error :
```
# After launching AutoSDV with make launch-sim:
ros2 topic list
# Output: Only /parameter_events and /rosout
```
## Root Cause #1: Conflicting CYCLONEDDS Environment Variable (Fast-RTPS vs CycloneDDS)
## Root Cause #2: Missing ROS_LOCALHOST_ONLY Setting
## Solution:
# Before (BROKEN):
```
launch-sim:
	@/bin/bash -c ' \
		set -e; \
		source install/setup.bash; \
		export RMW_IMPLEMENTATION=rmw_fastrtps_cpp; \
		ros2 launch autosdv_launch autosdv.launch.yaml \
			launch_rviz:=true \
			launch_vehicle:=false \
			launch_sensing_driver:=false \
			launch_perception:=false \
			use_gnss:=false \
	'
```
# After (WORKING):
```
launch-sim:
	@/bin/bash -c ' \
		set -e; \
		source install/setup.bash; \
		unset CYCLONEDDS_URI; \
		export ROS_LOCALHOST_ONLY=1; \
		export ROS_DOMAIN_ID=0; \
		export RMW_IMPLEMENTATION=rmw_fastrtps_cpp; \
		ros2 launch autosdv_launch autosdv.launch.yaml \
			launch_rviz:=true \
			launch_vehicle:=false \
			launch_sensing_driver:=false \
			launch_perception:=false \
			use_gnss:=false \
	'
```
# Changes
1. **`unset CYCLONEDDS_URI`** - Removes conflicting environment variable
2. **`export ROS_LOCALHOST_ONLY=1`** - Forces all DDS communication through localhost (127.0.0.1)
   - Bypasses multicast issues in Docker
   - Ensures all nodes can discover each other
3. **`export ROS_DOMAIN_ID=0`** - Explicitly sets ROS domain (default is 0)
4. **`export RMW_IMPLEMENTATION=rmw_fastrtps_cpp`** - Uses Fast-RTPS (now actually works)


========================================================================
========================================================================


