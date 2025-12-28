cmake_minimum_required(VERSION 3.8)
project(autosdv_isaac_slam_launch)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Build bridge executables
add_executable(odometry_to_twist_bridge
  src/odometry_to_twist_bridge.cpp
)

add_executable(odometry_to_pose_bridge
  src/odometry_to_pose_bridge.cpp
)

ament_target_dependencies(odometry_to_twist_bridge
  rclcpp
  rclcpp_components
  nav_msgs
  geometry_msgs
)

ament_target_dependencies(odometry_to_pose_bridge
  rclcpp
  rclcpp_components
  nav_msgs
  geometry_msgs
)

# Install executables
install(TARGETS
  odometry_to_twist_bridge
  odometry_to_pose_bridge
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install RViz config
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Install test directory
  install(DIRECTORY
    test
    DESTINATION share/${PROJECT_NAME}/
  )

  # Add Python integration tests
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_ar_tag_fusion test/test_ar_tag_fusion.py
    TIMEOUT 30
  )
endif()

ament_package()
