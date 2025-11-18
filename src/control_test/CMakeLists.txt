cmake_minimum_required(VERSION 3.14)
project(control_test)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
# Note: mock_controller.py is installed via setup.py entry_points
# install(PROGRAMS
#   control_test/mock_controller.py
#   DESTINATION lib/${PROJECT_NAME}
#   RENAME mock_controller
# )

# Install launch files (if they exist)
# install(DIRECTORY
#   src/control_test/launch
#   DESTINATION share/${PROJECT_NAME}/launch
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
