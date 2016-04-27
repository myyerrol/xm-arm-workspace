cmake_minimum_required(VERSION 2.8.3)
project(xm_arm_trajectory_control)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  roscpp
  rospy
  trajectory_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  xm_arm_trajectory_move_test
  include/xm_arm_trajectory_control/xm_arm_trajectory_move_test.h
  src/xm_arm_trajectory_move_test.cpp
)

target_link_libraries(
	xm_arm_trajectory_move_test
  ${catkin_LIBRARIES}
)
