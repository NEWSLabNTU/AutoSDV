cmake_minimum_required(VERSION 3.8)
project(map_check_projector)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autoware_map_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(autoware_geography_utils REQUIRED)
find_package(autoware_map_projection_loader REQUIRED)

add_executable(project_points project_points.cpp)
ament_target_dependencies(project_points
  rclcpp
  autoware_map_msgs
  geographic_msgs
  autoware_geography_utils
  autoware_map_projection_loader
)

install(TARGETS project_points DESTINATION lib/${PROJECT_NAME})
