/*********************************************************************
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
 ********************************************************************/

// Authors: myyerrol
// Created: 2016.4.15

#ifndef ARM_TELEOP_POSITION_KEYBOARD_H
#define ARM_TELEOP_POSITION_KEYBOARD_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <termio.h>
#include <signal.h>
#include <sys/poll.h>
#include <vector>
#include <string>
#include <map>
#include <boost/thread/thread.hpp>

#define KEYCODE_A     0X61
#define KEYCODE_D     0X64
#define KEYCODE_S     0X73
#define KEYCODE_W     0X77
#define KEYCODE_Q     0X71
#define KEYCODE_E     0X65
#define KEYCODE_X     0x78
#define KEYCODE_A_CAP 0X41
#define KEYCODE_D_CAP 0X44
#define KEYCODE_S_CAP 0X53
#define KEYCODE_W_CAP 0X57
#define KEYCODE_Q_CAP 0X51
#define KEYCODE_E_CAP 0X45
#define KEYCODE_X_CAP 0x58

class ArmTeleopPositionKeyboard
{
public:
    ArmTeleopPositionKeyboard();
    ~ArmTeleopPositionKeyboard();
    void spinTeleopArm();
    void stopTeleopArm();
private:
    ros::NodeHandle                       xm_nh_;
    std_msgs::Float64                     arm_pos_;
    std::map<std::string, ros::Publisher> arm_pos_pub_;
    std::map<std::string, int>            arm_index_;
    double                                arm_pos_step_;
    std::vector<std::string>              arm_name_;
    std_msgs::Float64                     gripper_pos_;
    std::map<std::string, ros::Publisher> gripper_pos_pub_;
    std::map<std::string, int>            gripper_index_;
    double                                gripper_pos_step_;
    std::vector<std::string>              gripper_name_;
};

#endif // ARM_TELEOP_POSITION_KEYBOARD_H
