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

#include <xm_arm_teleop/xm_arm_teleop_position_keyboard.h>

int            g_kfd = 0;
struct termios g_cooked;
struct termios g_raw;

ArmTeleopPositionKeyboard::ArmTeleopPositionKeyboard()
{
    arm_name_.push_back("joint_lift");
    arm_name_.push_back("joint_waist");
    arm_name_.push_back("joint_big_arm");
    arm_name_.push_back("joint_forearm");
    arm_name_.push_back("joint_wrist_pitching");
    arm_name_.push_back("joint_wrist_rotation");

    gripper_name_.push_back("joint_finger_left");
    gripper_name_.push_back("joint_finger_right");

    ros::NodeHandle n_private("~");
    n_private.param("arm_pose_step", arm_pos_step_, 0.0174);
    n_private.param("gripper_pos_step", gripper_pos_step_, 0.0174);

    for (int i = 0; i < arm_name_.size(); i++)
    {
        arm_index_[arm_name_[i]] = i;
        arm_pos_pub_[arm_name_[i]] = xm_nh_.advertise<std_msgs::Float64>(
            "xm_arm/" + arm_name_[i] + "_position_controller/command", 1000);
    }

    for (int i = 0; i < gripper_name_.size(); i++)
    {
        gripper_index_[gripper_name_[i]] = i;
        gripper_pos_pub_[gripper_name_[i]] = xm_nh_.advertise<std_msgs::Float64>(
            "xm_arm/" + gripper_name_[i] + "_position_controller/command", 1000);
    }
}

ArmTeleopPositionKeyboard::~ArmTeleopPositionKeyboard()
{
    xm_nh_.shutdown();
}

void ArmTeleopPositionKeyboard::spinTeleopArm()
{
    char   keyboard_cmd;
    double arm_position[6];
    double gripper_position[2];
    bool   flag = false;
    memset(arm_position, 0, sizeof(arm_position));
    memset(gripper_position, 0, sizeof(gripper_position));

    tcgetattr(g_kfd, &g_cooked);
    memcpy(&g_raw, &g_cooked, sizeof(struct termios));
    g_raw.c_lflag &=~ (ICANON | ECHO);
    g_raw.c_cc[VEOL] = 1;
    g_raw.c_cc[VEOF] = 2;
    tcsetattr(g_kfd, TCSANOW, &g_raw);

    puts("-----------------------------------");
    puts("  Teleop Position Arm By KeyBoard  ");
    puts("             Version 2             ");
    puts("-----------------------------------");
    puts("Q                W                E");
    puts("A                                 D");
    puts("                 S                 ");
    puts("                 X                 ");
    puts("-----------------------------------");
    puts("W:Lift-UP          S:Waist-R       ");
    puts("A:Big_Arm-UP       D:Forearm-UP    ");
    puts("Q:Wrist_Pitching-UP                ");
    puts("E:Wrist_Rotation-CLOCK++           ");
    puts("X:Gripper Open                     ");
    puts("-----------------------------------");
    puts("Shift+W:Lift-DOWN  Shift+S:Waist-L ");
    puts("Shift+A:Big_Arm-DOWN               ");
    puts("Shift+D:Forearm-DOWN               ");
    puts("Shift+Q:Wrist_Pitching-DOWN        ");
    puts("Shift+E:Wrist_Rotation-CLOCK--     ");
    puts("Shift+X:Gripper Close              ");
	puts("-----------------------------------");
    puts("PRESS CTRL-C TO QUIT               ");

    struct pollfd ufd;
    ufd.fd = g_kfd;
    ufd.events = POLLIN;

    while(true)
    {
        boost::this_thread::interruption_point();
        int num;

        if((num = poll(&ufd, 1, 250)) < 0)
        {
            perror("Function poll():");
            return;
        }
        else if(num > 0)
        {
            if(read(g_kfd, &keyboard_cmd, 1) < 0)
            {
                perror("Function read():");
                return;
            }
        }
        else
        {
            if(flag == true)
                continue;
        }

        switch(keyboard_cmd)
        {
            case KEYCODE_W:
                arm_pos_step_ = 0.01;
                arm_position[arm_index_["joint_lift"]] += arm_pos_step_;
                arm_pos_step_ = 0.0174;
                if(arm_position[arm_index_["joint_lift"]] >= 0.20)
                    arm_position[arm_index_["joint_lift"]] = 0.20;
                arm_pos_.data = -arm_position[arm_index_["joint_lift"]];
                arm_pos_pub_["joint_lift"].publish(arm_pos_);
                flag = true;
                break;
            case KEYCODE_S:
                arm_position[arm_index_["joint_waist"]] += arm_pos_step_;
                if(arm_position[arm_index_["joint_waist"]] >= 1.047)
                    arm_position[arm_index_["joint_waist"]] = 1.047;
                arm_pos_.data = -arm_position[arm_index_["joint_waist"]];
                arm_pos_pub_["joint_waist"].publish(arm_pos_);
                flag = true;
                break;
            case KEYCODE_A:
                arm_position[arm_index_["joint_big_arm"]] += arm_pos_step_;
                if(arm_position[arm_index_["joint_big_arm"]] >= 1.309)
                    arm_position[arm_index_["joint_big_arm"]] = 1.309;
                arm_pos_.data = arm_position[arm_index_["joint_big_arm"]];
                arm_pos_pub_["joint_big_arm"].publish(arm_pos_);
                flag = true;
                break;
            case KEYCODE_D:
                arm_position[arm_index_["joint_forearm"]] += arm_pos_step_;
                if(arm_position[arm_index_["joint_forearm"]] >= 2.234)
                    arm_position[arm_index_["joint_forearm"]] = 2.234;
                arm_pos_.data = -arm_position[arm_index_["joint_forearm"]];
                arm_pos_pub_["joint_forearm"].publish(arm_pos_);
                flag = true;
                break;
            case KEYCODE_Q:
                arm_position[arm_index_["joint_wrist_pitching"]] +=
                    arm_pos_step_;
                if(arm_position[arm_index_["joint_wrist_pitching"]] >= 2.182)
                    arm_position[arm_index_["joint_wrist_pitching"]] = 2.182;
                arm_pos_.data =
                    -arm_position[arm_index_["joint_wrist_pitching"]];
                arm_pos_pub_["joint_wrist_pitching"].publish(
                    arm_pos_);
                flag = true;
                break;
            case KEYCODE_E:
                arm_position[arm_index_["joint_wrist_rotation"]] +=
                    arm_pos_step_;
                arm_pos_.data =
                    arm_position[arm_index_["joint_wrist_rotation"]];
                arm_pos_pub_["joint_wrist_rotation"].publish(arm_pos_);
                flag = true;
                break;
            case KEYCODE_X:
                gripper_position[gripper_index_["joint_finger_left"]] -=
                    gripper_pos_step_;
                gripper_position[gripper_index_["joint_finger_right"]] +=
                    gripper_pos_step_;
                gripper_pos_.data =
                    gripper_position[gripper_index_["joint_finger_left"]];
                gripper_pos_pub_["joint_finger_left"].publish(gripper_pos_);
                gripper_pos_.data =
                    gripper_position[gripper_index_["joint_finger_right"]];
                gripper_pos_pub_["joint_finger_right"].publish(gripper_pos_);
                flag = true;
                break;
            case KEYCODE_W_CAP:
                arm_pos_step_ = 0.01;
                arm_position[arm_index_["joint_lift"]] -= arm_pos_step_;
                arm_pos_step_ = 0.0174;
                if(arm_position[arm_index_["joint_lift"]] <= -0.20)
                    arm_position[arm_index_["joint_lift"]] = -0.20;
                arm_pos_.data = -arm_position[arm_index_["joint_lift"]];
                arm_pos_pub_["joint_lift"].publish(arm_pos_);
                flag = true;
                break;
            case KEYCODE_S_CAP:
                arm_position[arm_index_["joint_waist"]] -= arm_pos_step_;
                if(arm_position[arm_index_["joint_waist"]] <= -1.047)
                    arm_position[arm_index_["joint_waist"]] = -1.047;
                arm_pos_.data = -arm_position[arm_index_["joint_waist"]];
                arm_pos_pub_["joint_waist"].publish(arm_pos_);
                flag = true;
                break;
            case KEYCODE_A_CAP:
                arm_position[arm_index_["joint_big_arm"]] -= arm_pos_step_;
                if(arm_position[arm_index_["joint_big_arm"]] <= -1.396)
                    arm_position[arm_index_["joint_big_arm"]] = -1.396;
                arm_pos_.data = arm_position[arm_index_["joint_big_arm"]];
                arm_pos_pub_["joint_big_arm"].publish(arm_pos_);
                flag = true;
                break;
            case KEYCODE_D_CAP:
                arm_position[arm_index_["joint_forearm"]] -= arm_pos_step_;
                if(arm_position[arm_index_["joint_forearm"]] <= -2.234)
                    arm_position[arm_index_["joint_forearm"]] = -2.234;
                arm_pos_.data = -arm_position[arm_index_["joint_forearm"]];
                arm_pos_pub_["joint_forearm"].publish(arm_pos_);
                flag = true;
                break;
            case KEYCODE_Q_CAP:
                arm_position[arm_index_["joint_wrist_pitching"]] -=
                    arm_pos_step_;
                if(arm_position[arm_index_["joint_wrist_pitching"]] <= -2.182)
                    arm_position[arm_index_["joint_wrist_pitching"]] = -2.182;
                arm_pos_.data =
                    -arm_position[arm_index_["joint_wrist_pitching"]];
                arm_pos_pub_["joint_wrist_pitching"].publish(arm_pos_);
                flag = true;
                break;
            case KEYCODE_E_CAP:
                arm_position[arm_index_["joint_wrist_rotation"]] -=
                    arm_pos_step_;
                arm_pos_.data =
                    arm_position[arm_index_["joint_wrist_rotation"]];
                arm_pos_pub_["joint_wrist_rotation"].publish(arm_pos_);
                flag = true;
                break;
            case KEYCODE_X_CAP:
                gripper_position[gripper_index_["joint_finger_left"]] +=
                    gripper_pos_step_;
                gripper_position[gripper_index_["joint_finger_right"]] -=
                    gripper_pos_step_;
                gripper_pos_.data =
                    gripper_position[gripper_index_["joint_finger_left"]];
                gripper_pos_pub_["joint_finger_left"].publish(gripper_pos_);
                gripper_pos_.data =
                    gripper_position[gripper_index_["joint_finger_right"]];
                gripper_pos_pub_["joint_finger_right"].publish(gripper_pos_);
                flag = true;
                break;
            default:
                flag = false;
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"xm_arm_teleop_position_keyboard",
        ros::init_options::NoSigintHandler);

    ArmTeleopPositionKeyboard teleop_position_keyboard;

    boost::thread make_thread = boost::thread(boost::bind(
        &ArmTeleopPositionKeyboard::spinTeleopArm, &teleop_position_keyboard));

    ros::spin();

    make_thread.interrupt();
    make_thread.join();
    tcsetattr(g_kfd, TCSANOW, &g_cooked);

    return 0;
}
