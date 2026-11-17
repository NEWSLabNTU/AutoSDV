cmake_minimum_required(VERSION 3.5)
project(autosdv_cmake NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY cmake DESTINATION share/${PROJECT_NAME})
ament_package(CONFIG_EXTRAS autosdv_cmake-extras.cmake)
