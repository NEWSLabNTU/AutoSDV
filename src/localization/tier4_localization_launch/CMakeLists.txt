cmake_minimum_required(VERSION 3.14)
project(tier4_localization_launch)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)

# Version-specific files: autoware/<version>/ (autosdv_cmake).
autosdv_install_autoware_overlay()
