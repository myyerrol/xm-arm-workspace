#!/usr/bin/env python
"""
 ********************************************************************
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Team-Xmbot-Service-Robot
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Team-Xmbot-Service-Robot nor the names
 *     of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written
 *     permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES sOF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************
"""

# Authors: myyerrol
# Created: 2016.4.15

import rospy
import actionlib
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from sys import exit


class ArmTrajectoryMoveTest:
    def __init__(self):
        self.arm_client = actionlib.SimpleActionClient(
            "xm_arm/arm_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction)
        rospy.init_node('xm_arm_trajectory_move_test', anonymous=False)
        joint_dof = 6

        print "------------ Test Arm Accurate Move ------------"
        print "                    Version 2                   "
        print "Input joint's position to move arm              "
        print "Such as [joint/j x x x x x x] or [exit/e]       "
        while True:
            print ">>> ",
            keyboard_cmd = raw_input().split(" ")
            try:
                if keyboard_cmd[0] == "joint" or keyboard_cmd[0] == "j":
                    if (len(keyboard_cmd) - 1) == joint_dof:
                        joint_lift = -float(keyboard_cmd[1])
                        joint_waist = -float(keyboard_cmd[2])
                        joint_big_arm = float(keyboard_cmd[3])
                        joint_forearm = -float(keyboard_cmd[4])
                        joint_wrist_pitching = -float(keyboard_cmd[5])
                        joint_wrist_rotation = float(keyboard_cmd[6])
                        print "joint_lift:%lf joint_waist:%lf " \
                              "joint_big_arm:%lf joint_forearm:%lf " \
                              "joint_wrist_pitching:%lf " \
                              "joint_wrist_rotation:%lf" % (
                                  joint_lift,
                                  joint_waist,
                                  joint_big_arm,
                                  joint_forearm,
                                  joint_wrist_pitching,
                                  joint_wrist_rotation)
                        arm_goal = [joint_lift,
                                    joint_waist,
                                    joint_big_arm,
                                    joint_forearm,
                                    joint_wrist_pitching,
                                    joint_wrist_rotation]
                        self.send_arm_goal(arm_goal)
                    else:
                        rospy.logerr("Joint's data must reach the dof of arm!")
                elif keyboard_cmd[0] == "exit" or keyboard_cmd[0] == "e":
                    exit()
            except Exception as exce:
                print "Error!", exce

    def send_arm_goal(self, arm_goal):
        arm_joint_names = ['joint_lift',
                           'joint_waist',
                           'joint_big_arm',
                           'joint_forearm',
                           'joint_wrist_pitching',
                           'joint_wrist_rotation']

        rospy.loginfo("Waiting for follow_joint_trajectory server")
        self.arm_client.wait_for_server()
        rospy.loginfo("Connected to follow_joint_trajectory server")
        rospy.sleep(1)
        arm_trajectory = JointTrajectory()
        arm_trajectory.joint_names = arm_joint_names
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[0].positions = arm_goal
        arm_trajectory.points[0].time_from_start = rospy.Duration(10)
        rospy.loginfo("Preparing for moving the arm to goal position!")
        rospy.sleep(1)
        arm_goal_pos = FollowJointTrajectoryGoal()
        arm_goal_pos.trajectory = arm_trajectory

        arm_goal_pos.goal_time_tolerance = rospy.Duration(0)
        self.arm_client.send_goal(arm_goal_pos)
        rospy.loginfo("Send goal to the trajectory server successfully!")
        self.arm_client.wait_for_result()


if __name__ == '__main__':
    try:
        arm_position = ArmTrajectoryMoveTest()
    except KeyboardInterrupt:
        print "Exit!"
