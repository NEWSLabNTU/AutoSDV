cmake_minimum_required(VERSION 3.14)
project(visual_pose_initializer_bridge)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(autoware_internal_localization_msgs REQUIRED)

# Build the node
add_executable(visual_pose_initializer_bridge_node
  src/visual_pose_initializer_bridge.cpp
)

ament_target_dependencies(visual_pose_initializer_bridge_node
  rclcpp
  rclcpp_components
  geometry_msgs
  std_srvs
  autoware_internal_localization_msgs
)

# Install executable
install(TARGETS
  visual_pose_initializer_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
