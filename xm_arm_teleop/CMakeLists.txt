cmake_minimum_required(VERSION 2.8.3)
project(xm_arm_teleop)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  control_msgs
  actionlib
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(xm_arm_teleop_position_keyboard
  include/xm_arm_teleop/xm_arm_teleop_position_keyboard.h
  src/xm_arm_teleop_position_keyboard.cpp
)
add_executable(xm_arm_teleop_trajectory_keyboard
  include/xm_arm_teleop/xm_arm_teleop_trajectory_keyboard.h
  src/xm_arm_teleop_trajectory_keyboard.cpp
)

target_link_libraries(
  xm_arm_teleop_position_keyboard
  ${catkin_LIBRARIES}
)
target_link_libraries(
  xm_arm_teleop_trajectory_keyboard
  ${catkin_LIBRARIES}
)
