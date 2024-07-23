#include <ros/ros.h>
#include <cmath>
#include <iostream>
#include <std_msgs/Float32MultiArray.h>
#include "utility.h"
#include "Hardware/can.h"
#include "Hardware/motor.h"
#include "Hardware/teleop.h"
#include "App/arm_control.h"
#include "App/arm_control.cpp"
#include "App/keyboard.h"
#include "App/play.h"
#include "App/solve.h"
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <thread>
#include <atomic>
#include "arm_control/PosCmd.h"
#include "arm_control/JointControl.h"
#include "arm_control/JointInformation.h"
#include "arm_control/ChassisCtrl.h"
#include "arm_control/MagicCmd.h"

//！！！！！！！！！！！！启动前注意模式！！！区分X5 及 5a型号！！！！！！！！！！！！！！！！！！！！！！！！！！！！
//！！！！！！！！！！！！启动前注意模式！！！区分X5 及 5a型号！！！！！！！！！！！！！！！！！！！！！！！！！！！！
//！！！！！！！！！！！！启动前注意模式！！！区分X5 及 5a型号！！！！！！！！！！！！！！！！！！！！！！！！！！！！
int CONTROL_MODE=2; 
// 0 >> x5 pos_control  
// 1 >> 5a pos_control
// 2 >> x5 joint_control for moveit 
// 3 >> 5a joint_control for moveit 


int main(int argc, char **argv)
{
    ros::init(argc, argv, "follow_1"); 
    ros::NodeHandle node;
    Teleop_Use()->teleop_init(node);
    arx_arm ARX_ARM((int) CONTROL_MODE);

    //关节角度控制
    ros::Subscriber sub_joint = node.subscribe<arm_control::JointControl>("/follow_joint_control_1", 10, 
                                  [&ARX_ARM](const arm_control::JointControl::ConstPtr& msg) {
                                      ARX_ARM.ros_control_pos_t[0] = msg->joint_pos[0];
                                      ARX_ARM.ros_control_pos_t[1] = msg->joint_pos[1];
                                      ARX_ARM.ros_control_pos_t[2] = msg->joint_pos[2];
                                      ARX_ARM.ros_control_pos_t[3] = msg->joint_pos[3];
                                      ARX_ARM.ros_control_pos_t[4] = msg->joint_pos[4];
                                      ARX_ARM.ros_control_pos_t[5] = msg->joint_pos[5];
                                      ARX_ARM.arx5_cmd.gripper     = msg->joint_pos[6];
                                  });
    //姿态控制
    ros::Subscriber sub_cmd = node.subscribe<arm_control::PosCmd>("/follow_pos_cmd_1", 10, 
                                [&ARX_ARM](const arm_control::PosCmd::ConstPtr& msg) {
                                    ARX_ARM.arx5_cmd.x_t          = msg->x;
                                    ARX_ARM.arx5_cmd.y_t          = msg->y;
                                    ARX_ARM.arx5_cmd.z_t          = msg->z;
                                    ARX_ARM.arx5_cmd.waist_roll   = msg->roll;
                                    ARX_ARM.arx5_cmd.waist_pitch  = msg->pitch;
                                    ARX_ARM.arx5_cmd.waist_yaw    = msg->yaw;
                                    ARX_ARM.arx5_cmd.gripper      = msg->gripper;
                                });
    ros::Publisher pub_current = node.advertise<arm_control::JointInformation>("joint_information", 10);
    ros::Publisher pub_pos = node.advertise<arm_control::PosCmd>("/follow1_pos_back", 10);
    


    arx5_keyboard ARX_KEYBOARD;
    ros::Rate loop_rate(200);
    can CAN_Handlej;

    std::thread keyThread(&arx5_keyboard::detectKeyPress, &ARX_KEYBOARD);
    sleep(1);

    while(ros::ok())
    { 
        char key = ARX_KEYBOARD.keyPress.load();
        ARX_ARM.getKey(key);//获取键盘信息 用于控制夹爪

        ARX_ARM.get_joint();//获取关节信息
        ARX_ARM.get_pos();//获取姿态信息

        ARX_ARM.update_real();//更新关节信息
    
//自定义消息
        //发送末端姿态
            arm_control::PosCmd msg_pos_back;            
            msg_pos_back.x      =ARX_ARM.solve.End_Effector_Pose[0];
            msg_pos_back.y      =ARX_ARM.solve.End_Effector_Pose[1];
            msg_pos_back.z      =ARX_ARM.solve.End_Effector_Pose[2];
            msg_pos_back.roll   =ARX_ARM.solve.End_Effector_Pose[3];
            msg_pos_back.pitch  =ARX_ARM.solve.End_Effector_Pose[4];
            msg_pos_back.yaw    =ARX_ARM.solve.End_Effector_Pose[5];
            msg_pos_back.gripper=ARX_ARM.current_pos[6];
            msg_pos_back.header.stamp = ros::Time::now();
            pub_pos.publish(msg_pos_back);            


        //发送关节信息
            arm_control::JointInformation msg_joint;   
            for(int i=0;i<7;i++)
            {
                msg_joint.joint_pos[i] = ARX_ARM.current_pos[i];
                msg_joint.joint_vel[i] = ARX_ARM.current_vel[i];
                msg_joint.joint_cur[i] = ARX_ARM.current_torque[i];
            }   
            msg_joint.header.stamp = ros::Time::now();
            pub_current.publish(msg_joint);

        ros::spinOnce();
        loop_rate.sleep();
        
    }
    return 0;
}