# autosdv_install_autoware_overlay()
#
# Install the files of this package that differ between Autoware versions.
#
# They live once per version, under autoware/<version>/ in the package's own
# layout (launch/..., config/...), and never in the shared tree as well. The
# installed package is its shared files plus the overlay of the version this
# workspace is built against: AUTOWARE_VERSION, which every Autoware setup.bash
# exports. Each version has its own workspace (versions.yaml, workspace_suffix),
# so the choice is made once, at build time.
#
# Fails the configure step when AUTOWARE_VERSION is unset, when the package has
# no overlay for it, when the version directories hold different launch files (an
# include would then resolve on one version's workspace and fail on the other's),
# or when a file is both shared and versioned. Only launch/ must hold the same files
# in every version (a shared file includes them by name); a parameter file named
# only by one version's own launch files need not exist for the other.
macro(autosdv_install_autoware_overlay)
  set(_aw "$ENV{AUTOWARE_VERSION}")
  if(_aw STREQUAL "")
    message(FATAL_ERROR "AUTOWARE_VERSION is not set: source the selected Autoware "
      "(direnv, or scripts/env.sh) before building ${PROJECT_NAME}")
  endif()
  set(_overlays "${CMAKE_CURRENT_SOURCE_DIR}/autoware")
  if(NOT IS_DIRECTORY "${_overlays}/${_aw}")
    message(FATAL_ERROR "${PROJECT_NAME} has no overlay for Autoware ${_aw} (${_overlays}/${_aw})")
  endif()
  file(GLOB _versions RELATIVE "${_overlays}" "${_overlays}/*")
  file(GLOB_RECURSE _want RELATIVE "${_overlays}/${_aw}" "${_overlays}/${_aw}/*")
  file(GLOB_RECURSE _want_launch RELATIVE "${_overlays}/${_aw}" "${_overlays}/${_aw}/launch/*")
  list(SORT _want_launch)
  foreach(_v ${_versions})
    file(GLOB_RECURSE _have_launch RELATIVE "${_overlays}/${_v}" "${_overlays}/${_v}/launch/*")
    list(SORT _have_launch)
    if(NOT _have_launch STREQUAL _want_launch)
      message(FATAL_ERROR "${PROJECT_NAME}: autoware/${_v}/launch/ and autoware/${_aw}/launch/ hold different files")
    endif()
  endforeach()
  foreach(_f ${_want})
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${_f}")
      message(FATAL_ERROR "${PROJECT_NAME}: ${_f} is both shared and in autoware/${_aw}/; "
        "a version-specific file lives only under autoware/<version>/")
    endif()
  endforeach()
  install(DIRECTORY "${_overlays}/${_aw}/" DESTINATION share/${PROJECT_NAME})
  message(STATUS "${PROJECT_NAME}: Autoware ${_aw} overlay")
endmacro()
