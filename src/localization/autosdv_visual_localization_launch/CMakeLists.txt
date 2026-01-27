cmake_minimum_required(VERSION 3.14)
project(autosdv_visual_localization_launch)

find_package(ament_cmake REQUIRED)

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
