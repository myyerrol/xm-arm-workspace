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
from std_msgs.msg import Float64
from sys import exit
from math import sin, cos, sqrt
from scipy.optimize import fsolve


class GripperMoveTest:
    def __init__(self):
        rospy.init_node('xm_arm_gripper_move_test', anonymous=True)
        gripper_pos_pub = rospy.Publisher('xm_gripper/command', Float64,
                                          queue_size=1000)
        print "------------ Test Gripper Accurate Move ------------"
        print "                      Version 2                     "
        print "Input object's length to calculate servo's position "
        print "Input servo's position to calculate object's length "
        print "Such as [servo/s xxx] or [object/o xxx] or [exit/e] "

        while not rospy.is_shutdown():
            print ">>> ",
            keyboard_cmd = raw_input().split(" ")
            try:
                if keyboard_cmd[0] == "servo" or keyboard_cmd[0] == "s":
                    if -1.0 <= float(keyboard_cmd[1]) <= 1.5:
                        angle_in = float(keyboard_cmd[1])
                        width_out = 2 * (2.5 * sin(angle_in) + sqrt(
                            4.5 ** 2 - 2.5 ** 2 * cos(angle_in) ** 2) - 2)
                        print "Object's length: %lfm" % (width_out / 100)
                    else:
                        print "The number must between -1.0 and 1.5"
                elif keyboard_cmd[0] == "object" or keyboard_cmd[0] == "o":
                    def calculate_servo_angle(x):
                        offset = 1.50
                        width_in = float(keyboard_cmd[1]) * 100 - offset
                        angle_out = float(x[0])
                        return [
                            width_in - 2 * (2.5 * sin(angle_out) + sqrt(
                                4.5 ** 2 - 2.5 ** 2 * cos(angle_out) ** 2) - 2)
                        ]
                    servo_angle = fsolve(calculate_servo_angle, [1])
                    print "Servo's angle: %lfrad" % servo_angle
                    gripper_pos_pub.publish(servo_angle)
                elif keyboard_cmd[0] == "exit" or keyboard_cmd[0] == "e":
                    exit()
            except Exception as exce:
                print "Error!", exce


if __name__ == '__main__':
    try:
        gripper_move_test = GripperMoveTest()
    except KeyboardInterrupt:
        print "Exit!"
