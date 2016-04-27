cmake_minimum_required(VERSION 2.8.3)
project(xm_arm_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  actionlib_msgs
)

add_message_files(
  FILES
  xm_ArmSerialData.msg
  xm_ArmSerialDatagram.msg
  xm_ArmGripperCommand.msg
)

add_action_files(
  DIRECTORY
  action
  FILES
  xm_ArmGripperCommand.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)
