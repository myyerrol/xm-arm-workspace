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

#include <xm_arm_robot_hardware/xm_arm_robot_hardware.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "xm_arm_robot_hardware");
    ros::NodeHandle xm_nh;
    ArmRobotHardware xm_arm_robothw(xm_nh);

    if (xm_arm_robothw.start())
        ROS_INFO_STREAM("xm_arm_robot_hardware initialize successfully!");
    else
    {
        ROS_ERROR_STREAM("xm_arm_robot_hardware initialize by error!");
        return 0;
    }

    ros::NodeHandle cm_nh("xm_arm");
    ros::CallbackQueue cm_callback_queue;
    cm_nh.setCallbackQueue(&cm_callback_queue);
    controller_manager::ControllerManager manager(&xm_arm_robothw, cm_nh);
    ros::Rate rate(xm_arm_robothw.getFreq());
    ros::AsyncSpinner hw_spinner(1, xm_arm_robothw.getCallbackQueue());
    ros::AsyncSpinner cm_spinner(1, &cm_callback_queue);
    hw_spinner.start();
    cm_spinner.start();

    while (ros::ok())
    {
        ros::Time current_time = ros::Time::now();
        xm_arm_robothw.read(current_time,
            ros::Duration(1 / xm_arm_robothw.getFreq()));
        manager.update(current_time,
            ros::Duration(1 / xm_arm_robothw.getFreq()));
        xm_arm_robothw.write(current_time,
            ros::Duration(1 / xm_arm_robothw.getFreq()));
        rate.sleep();
    }

    hw_spinner.stop();
    cm_spinner.stop();
    return 0;
}
