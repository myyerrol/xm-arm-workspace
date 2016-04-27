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

import rospy
import actionlib
from xm_arm_msgs.msg import xm_ArmGripperCommandAction
from xm_arm_msgs.msg import xm_ArmGripperCommandResult
from std_msgs.msg import Float64
from dynamixel_controllers.srv import SetTorqueLimit
from math import sin, cos, sqrt
from scipy.optimize import fsolve


class GripperActionServer(object):
    gripper_result = xm_ArmGripperCommandResult()

    def __init__(self, name):
        self.gripper_action_name = name
        self.gripper_action_server = actionlib.SimpleActionServer(
            self.gripper_action_name, xm_ArmGripperCommandAction,
            execute_cb=self.execute_cb, auto_start=False)
        self.gripper_action_server.start()
        rospy.loginfo("Start action server!")
        self.gripper_pos_pub = rospy.Publisher('xm_gripper/command', Float64,
                                               queue_size=1000)

    def execute_cb(self, goal):
        rate = rospy.Rate(20)
        success = True

        rospy.loginfo("Receiving gripper's goal and torque %lf %lf",
                      goal.command.position, goal.command.torque)
        if self.gripper_action_server.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self.gripper_action_name)
            self.gripper_action_server.set_preempted()
            success = False

        rate.sleep()

        def calculate_servo_angle(x):
            offset = 0
            width = goal.command.position * 100 - offset
            angle = float(x[0])
            return [
                width - 2 * (2.5 * sin(angle) + sqrt(
                    4.5 ** 2 - 2.5 ** 2 * cos(angle) ** 2) - 2)
            ]

        servo_angle = fsolve(calculate_servo_angle, [1])
        print "Servo's angle: %lfrad" % servo_angle
        self.gripper_pos_pub.publish(servo_angle)

        rospy.loginfo("Send gripper's goal position to the topic successfully!")

        rospy.loginfo("Waiting for gripper's torque service")
        controller_service = 'xm_gripper/set_torque_limit'
        rospy.wait_for_service(controller_service)
        rospy.loginfo("Connected to the service server")

        try:
            gripper_torque = rospy.ServiceProxy(controller_service,
                                                SetTorqueLimit)
            gripper_torque(goal.command.torque)
            rospy.loginfo("Set the torque of gripper successfully!")
        except rospy.ServiceException:
            rospy.logerr("Error, please check gripper_joint service!")

        if success:
            self.gripper_result.finish = True
            rospy.loginfo("Finish all tasks!")
            self.gripper_action_server.set_succeeded(self.gripper_result)


if __name__ == '__main__':
    try:
        rospy.init_node('xm_arm_gripper_action_server')
        GripperActionServer('xm_gripper_action')
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
