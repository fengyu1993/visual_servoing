#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <franka/robot.h>
#include "direct_visual_servoing.h"
#include "ros_DVS.h"
#include "examples_common.h"
#include "franka_control_base.h"

int main(int argc, char** argv)
{
    // ׼��
    ros::init(argc, argv, "DVS");  
    // ��е���ƶ�������λ��
    Mat joint_angle_work = (Mat_<double>(7,1) << 0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4);
    franka_move_to_target_joint_angle(joint_angle_work);
    // ��е���ƶ�����ʼλ��
    Ros_DVS DVS_control;
    Mat joint_angle_initial = DVS_control.joint_angle_initial_;
    franka_move_to_target_joint_angle(joint_angle_initial);
    // ����
    ros::Rate loop_rate(DVS_control.control_rate_);
    while (ros::ok())
    {
        try{
            if(DVS_control.flag_success_){
                ROS_INFO("visual servoing success");
                break;
            }else{
                ros::spinOnce();
                loop_rate.sleep();
            }
        }catch(...){
            return 1;
        }
    }
    // ��е���ƶ�������λ��
    franka_move_to_target_joint_angle(joint_angle_work);
    // ����
    return 0;
}

















