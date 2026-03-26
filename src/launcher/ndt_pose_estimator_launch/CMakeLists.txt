cmake_minimum_required(VERSION 3.14)
project(ndt_pose_estimator_launch)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_package(INSTALL_TO_SHARE launch)
