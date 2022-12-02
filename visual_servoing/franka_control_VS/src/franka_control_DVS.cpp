#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include "direct_visual_servoing.h"
#include "ros_DVS.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

int main(int argc, char** argv)
{
    // ׼��
    ros::init(argc, argv, "DVS");  
    ros::AsyncSpinner spinner(1);
    spinner.start();  
    Ros_DVS DVS_control;
    // ��е���ƶ�����ʼλ��
    DVS_control.control_switcher_.switch_controllers("moveit", "velocity");
    std::vector<double> joint_group_positions_start= {0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4};
    DVS_control.franka_move_to_target_joint_angle(joint_group_positions_start);
    // ��е���ƶ�����ʼ�ŷ�λ��
    std::vector<double> joint_group_positions_init_VS;
    ros::param::get("joint_angle_initial", joint_group_positions_init_VS);
    DVS_control.franka_move_to_target_joint_angle(joint_group_positions_init_VS);
    // ת��������
    DVS_control.control_switcher_.switch_controllers("velocity", "moveit");
    // �Ӿ��ŷ�����
    spinner.stop();
    cout << "Start visual servoing control ... " << endl;
    cout << "Press Enter to start..." << endl;
    cin.ignore();
    DVS_control.initialize_time_sync();
    DVS_control.start_VS = true;
    ros::Rate loop_rate(DVS_control.control_rate_);
    int num = 0; 
    while (ros::ok())
    {
        try{
            if(DVS_control.flag_success_){
                ROS_INFO("visual servoing success");
                DVS_control.start_VS = false;
                break;
            }else{
                ros::spinOnce();
                loop_rate.sleep();
            }
        }catch(...){
            return 1;
        }
    }
    // ת��������
    cout << "Move to work position ... " << endl;
    cout << "Press Enter to start..." << endl;
    cin.ignore();
    DVS_control.control_switcher_.switch_controllers("moveit", "velocity");
    spinner.start();
    // ��е���ƶ�����ʼλ��
    DVS_control.franka_move_to_target_joint_angle(joint_group_positions_start);
    // ����
    return 0;
}

