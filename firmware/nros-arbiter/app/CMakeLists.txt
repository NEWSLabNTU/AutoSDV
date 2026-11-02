# RC override arbiter on Zephyr + nano-ros.
#
# The decisions are firmware/rc-arbiter/lib, shared with the PlatformIO build
# and covered by its host tests (`just arbiter test`); nothing here copies
# them. src/ is the Zephyr hardware layer plus the nano-ros node.
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(autosdv_arbiter LANGUAGES C CXX)

set(RC_ARBITER_LIB ${CMAKE_CURRENT_SOURCE_DIR}/../../rc-arbiter/lib)

target_include_directories(app PRIVATE
    ${RC_ARBITER_LIB}/arbiter
    ${RC_ARBITER_LIB}/vesc
    src)
target_sources(app PRIVATE
    ${RC_ARBITER_LIB}/arbiter/arbiter.cpp
    ${RC_ARBITER_LIB}/arbiter/heartbeat.cpp
    ${RC_ARBITER_LIB}/arbiter/pulse.cpp
    ${RC_ARBITER_LIB}/vesc/vesc_packet.cpp
    src/control.cpp)

# The hardware layer. On the car's XIAO RP2350 it is the pins of app.overlay;
# on QEMU's mps2/an521, which has none of them, a scripted stand-in
# (src/sim/board_sim.cpp) feeds the same control loop.
if(BOARD STREQUAL "mps2")
  target_sources(app PRIVATE src/sim/board_sim.cpp)
else()
  target_sources(app PRIVATE
      src/board_io_xiao.cpp
      src/capture.cpp
      src/status_led.cpp
      src/vesc_uart.cpp)
endif()

find_package(nano_ros REQUIRED)
find_package(std_msgs REQUIRED)

nano_ros_auto_add_library(arbiter_node STATIC src/ArbiterNode.cpp)
target_include_directories(arbiter_node PRIVATE
    ${RC_ARBITER_LIB}/arbiter
    src)

nros_components_register_node(arbiter_node
    HEADER ArbiterNode.hpp
    PLUGIN autosdv_arbiter::ArbiterNode
    EXECUTABLE arbiter
    SHAPE configure
    TYPED)
