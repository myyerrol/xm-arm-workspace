cmake_minimum_required(VERSION 2.8.3)
project(xm_arm_robot_hardware)

find_package(catkin REQUIRED COMPONENTS
  control_toolbox
  controller_manager
  hardware_interface
  realtime_tools
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  xm_arm_robot_hardware
  include/xm_arm_robot_hardware/xm_arm_robot_hardware.h
  src/xm_arm_robot_hardware.cpp
  src/main.cpp
)

target_link_libraries(
  xm_arm_robot_hardware
  ${catkin_LIBRARIES}
)
