cmake_minimum_required(VERSION 2.8.3)
project(xm_arm_moveit_control)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_perception
  moveit_ros_planning_interface
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(xm_arm_moveit_ik_test
  src/xm_arm_moveit_ik_test.cpp
)

target_link_libraries(
  xm_arm_moveit_ik_test
  ${catkin_LIBRARIES}
)
