cmake_minimum_required(VERSION 3.8)
project(autosdv_twin NONE)

find_package(ament_cmake REQUIRED)

foreach(dir vehicle isaac catalogs launch config)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${dir})
    install(DIRECTORY ${dir} DESTINATION share/${PROJECT_NAME})
  endif()
endforeach()

ament_package()
