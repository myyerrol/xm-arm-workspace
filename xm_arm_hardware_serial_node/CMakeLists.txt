cmake_minimum_required(VERSION 2.8.3)
project(xm_arm_hardware_serial_node)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  xm_arm_hardware_serial_node
  include/xm_arm_hardware_serial_node/xm_arm_hardware_serial_port.h
  include/xm_arm_hardware_serial_node/xm_arm_hardware_serial_node.h
  src/main_node.cpp
  src/xm_arm_hardware_serial_port.cpp
  src/xm_arm_hardware_serial_node.cpp
)

target_link_libraries(
  xm_arm_hardware_serial_node
  ${catkin_LIBRARIES}
)
