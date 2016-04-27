cmake_minimum_required(VERSION 2.8.3)
project(xm_arm_gripper_control)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  dynamixel_controllers
  rospy
  std_msgs
)
