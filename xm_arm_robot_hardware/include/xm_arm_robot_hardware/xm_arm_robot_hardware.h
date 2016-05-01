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

#ifndef XM_ARM_ROBOT_HARDWARE_H
#define XM_ARM_ROBOT_HARDWARE_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <controller_manager/controller_manager.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Float64.h>
#include <xm_arm_msgs/xm_ArmSerialDatagram.h>

#include <sstream>
#include <vector>
#include <string>
#include <map>
#include <boost/shared_ptr.hpp>

class ArmRobotHardware : public hardware_interface::RobotHW
{
public:
    ArmRobotHardware(ros::NodeHandle nh);
    ~ArmRobotHardware();
    ros::Time getTime();
    ros::Duration getPeriod();
    ros::CallbackQueue* getCallbackQueue();
    void read(const ros::Time, const ros::Duration period);
    void write(const ros::Time, const ros::Duration period);
    bool start();
    void stop();
    double getFreq() const;
private:
    void publishArmCommand(const u_int8_t func, const u_int8_t jnt_id,
                           const float jnt_pos);
    void publishArmJState(const u_int8_t func, const u_int8_t jnt_id);
    void getArmStateCallback(
        const xm_arm_msgs::xm_ArmSerialDatagram::ConstPtr& msg);
    void getArmStatusCallback(
        const xm_arm_msgs::xm_ArmSerialDatagram::ConstPtr& msg);
    bool checkArmStatus();
    void transPositionJointToActuator();
    void transPositionActuatorToJoint();
private:
    ros::NodeHandle    nh_;
    ros::CallbackQueue callback_queue_;
    ros::Publisher     arm_serial_pub_;
    ros::Subscriber    arm_state_sub_;
    ros::Subscriber    arm_status_sub_;
    int                arm_command_id_;
    int                arm_state_id_;
    double             freq_;
    hardware_interface::JointStateInterface    jnt_state_interfece_;
    hardware_interface::PositionJointInterface jnt_position_interface_;
    hardware_interface::VelocityJointInterface jnt_velocity_interface_;
    std::vector<std::string>      jnt_name_;
    std::vector<std::string>      act_name_;
    std::map<std::string, double> jnt_pos_;
    std::map<std::string, double> jnt_vel_;
    std::map<std::string, double> jnt_eff_;
    std::map<std::string, double> act_pos_;
    std::map<std::string, double> jnt_cmd_;
    std::map<std::string, double> act_cmd_;
    std::vector<ros::Time>        jnt_stamp_;
    enum HARDWARE_STATUS {UNKNOWN, READY, RUNNING, ERROR};
    std::vector<HARDWARE_STATUS> jnt_status_;
};

#endif // XM_ARM_ROBOT_HARDWARE_H
