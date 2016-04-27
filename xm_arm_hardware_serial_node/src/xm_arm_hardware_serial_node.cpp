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

// Authors: startar, myyerrol
// Created: 2012.4.8, 2016.4.15

#include <xm_arm_hardware_serial_node/xm_arm_hardware_serial_node.h>

namespace xm_serial_node
{
SerialNode::SerialNode(const ros::NodeHandle &nh,
                       const ros::NodeHandle &private_nh)
    : nh_(nh),
      private_nh_(private_nh)
{
    cout << "SerialNode Object created!" << endl;

    topic_name_[3] = "status";
    topic_name_[4] = "state";

    loadParams();
    ptr_serial_port_ = make_shared<SerialPort>();
    ptr_serial_port_->setSerialParams(serial_params_);
    ptr_serial_port_->setTimeOut(timeout_);
    ptr_serial_port_->setCallbackFunc(bind(&SerialNode::getSerialCallback,
        this, _1));

    serial_sub_ = nh_.subscribe("xm_arm_serial/send_arm_command", 1000,
        &SerialNode::getDatagramCallback, this);

    ptr_serial_port_->startThread();
}

SerialNode::~SerialNode()
{
    ptr_serial_port_->stopThread();
}

void SerialNode::loadParams()
{
    serial_params_.serial_port_  = "/dev/ttyUSB0";
    serial_params_.baud_rate_    = 921600;
    serial_params_.flow_control_ = 0;
    serial_params_.parity_bits_  = 0;
    serial_params_.stop_bits_    = 0;
    timeout_                     = 100;

    private_nh_.getParam("serial_port_", serial_params_.serial_port_);
    private_nh_.getParam("baud_rate_", (int&)(serial_params_.baud_rate_));
    private_nh_.getParam("flow_control_", (int&)(serial_params_.flow_control_));
    private_nh_.getParam("parity_bits_", (int&)(serial_params_.parity_bits_));
    private_nh_.getParam("stop_bits_", (int&)(serial_params_.stop_bits_));
    private_nh_.getParam("timeout", timeout_);
}

void SerialNode::getDatagramCallback(
    const xm_arm_msgs::xm_ArmSerialDatagram::ConstPtr &msg)
{
    cout << "Sending new datagram!" << endl;
    ptr_serial_port_->writeDataGram(*msg);
}

void SerialNode::getSerialCallback(
    xm_arm_msgs::xm_ArmSerialDatagramPtr ptr_datagram)
{
    ros::Publisher &pub = serial_pub_[ptr_datagram->receiver];

    if (!pub)
    {
        stringstream ss;
        ss << "xm_arm_serial/receive_arm_" <<
            topic_name_[(int)(ptr_datagram->receiver)];
        pub = nh_.advertise<xm_arm_msgs::xm_ArmSerialDatagram>(ss.str(), 1000);
    }

    pub.publish(ptr_datagram);
}

} // namespace xm_serial_node
