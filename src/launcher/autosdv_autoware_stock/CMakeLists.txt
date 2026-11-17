cmake_minimum_required(VERSION 3.5)
project(autosdv_autoware_stock NONE)

find_package(ament_cmake REQUIRED)

# share/autosdv_autoware_stock/<area> -> that area's stock parameter tree in the
# Autoware this workspace is built against. Autoware 1.5.0 keeps them in
# autoware_launch/config/<area>; 2.1.0 moved control, localization, map, perception
# and planning into autoware_<area>_config/config, with the same layout beneath. A
# launch file writes $(find-pkg-share autosdv_autoware_stock)/control/... and gets
# the right file on either. The target is whichever exists, so no version table.
set(_home "$ENV{AUTOWARE_HOME}")
if(_home STREQUAL "" OR NOT IS_DIRECTORY "${_home}/share")
  message(FATAL_ERROR "AUTOWARE_HOME is not set: source the selected Autoware "
    "(direnv, or scripts/env.sh) before building ${PROJECT_NAME}")
endif()
set(_share "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}")
foreach(_area control localization map perception planning system)
  set(_t "${_home}/share/autoware_${_area}_config/config")
  if(NOT IS_DIRECTORY "${_t}")
    set(_t "${_home}/share/autoware_launch/config/${_area}")
  endif()
  if(NOT IS_DIRECTORY "${_t}")
    message(FATAL_ERROR "${PROJECT_NAME}: no stock ${_area} parameters under ${_home}")
  endif()
  message(STATUS "${PROJECT_NAME}: ${_area} -> ${_t}")
  install(CODE "file(MAKE_DIRECTORY \"${_share}\")
    file(REMOVE \"${_share}/${_area}\")
    file(CREATE_LINK \"${_t}\" \"${_share}/${_area}\" SYMBOLIC)")
endforeach()

ament_package()
