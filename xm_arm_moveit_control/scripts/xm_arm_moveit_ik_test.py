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
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
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

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped


class MoveIKTest:
    def __init__(self):
        rospy.init_node('xm_arm_moveit_ik_test', anonymous=True)
        moveit_commander.roscpp_initialize(sys.argv)
        xm_arm = moveit_commander.MoveGroupCommander('xm_arm')
        end_effector_link = xm_arm.get_end_effector_link()
        reference_frame = 'base_footprint'

        xm_arm.set_pose_reference_frame(reference_frame)
        xm_arm.allow_replanning(True)
        xm_arm.set_goal_position_tolerance(5)
        xm_arm.set_goal_orientation_tolerance(5)
        xm_arm.set_planner_id("RRTConnectkConfigDefault")

        xm_arm.set_named_target('initial')
        xm_arm.go()
        rospy.sleep(2)

        print "-------------------- Test Arm Moveit IK ----------------------"
        print "Input the position of object(reference frame is base's center)"
        print "Such as [object/o x y z x y z w]                                "
        while not rospy.is_shutdown():
            print ">>> ",
            keyboard_cmd = raw_input().split(" ")
            try:
                if keyboard_cmd[0] == "object" or keyboard_cmd[0] == "o":
                    target_pose = PoseStamped()
                    target_pose.header.frame_id = reference_frame
                    target_pose.header.stamp = rospy.Time.now()
                    target_pose.pose.position.x = float(keyboard_cmd[1])
                    target_pose.pose.position.y = float(keyboard_cmd[2])
                    target_pose.pose.position.z = float(keyboard_cmd[3])
                    target_pose.pose.orientation.x = float(keyboard_cmd[4])
                    target_pose.pose.orientation.y = float(keyboard_cmd[5])
                    target_pose.pose.orientation.z = float(keyboard_cmd[6])
                    target_pose.pose.orientation.w = float(keyboard_cmd[7])
                    print ("position[x y z] %s, %s, %s" % (keyboard_cmd[1],
                                                           keyboard_cmd[2],
                                                           keyboard_cmd[3]))
                    print "orientation[x y z w] %s, %s, %s, %s" % (
                        keyboard_cmd[4],
                        keyboard_cmd[5],
                        keyboard_cmd[6],
                        keyboard_cmd[7])
                    xm_arm.set_start_state_to_current_state()
                    xm_arm.set_pose_target(target_pose, end_effector_link)
                    trajectory = xm_arm.plan()
                    xm_arm.execute(trajectory)
                    rospy.sleep(1)
                elif keyboard_cmd[0] == "exit" or keyboard_cmd[0] == "e":
                    exit()
            except Exception as exce:
                print "Error!", exce

        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


if __name__ == "__main__":
    try:
        moveit_ik_test = MoveIKTest()
    except KeyboardInterrupt:
        print "Exit!"
