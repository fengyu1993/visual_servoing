#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include "discrete_cosine_transform_vs.h"
#include "ros_KM_VS.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

int main(int argc, char** argv)
{
    // ׼��
    ros::init(argc, argv, "KM_VS");  
    ros::AsyncSpinner spinner(1);
    spinner.start();  
    Ros_KM_VS KM_VS_control;
    // ��е���ƶ�����ʼλ��
    KM_VS_control.control_switcher_.switch_controllers("moveit", "velocity");
    std::vector<double> joint_group_positions_start= {0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4};
    KM_VS_control.franka_move_to_target_joint_angle(joint_group_positions_start);
    // ��е���ƶ�����ʼ�ŷ�λ��
    std::vector<double> joint_group_positions_init_VS;
    ros::param::get("joint_angle_initial", joint_group_positions_init_VS);
    KM_VS_control.franka_move_to_target_joint_angle(joint_group_positions_init_VS);
    // ת��������
    KM_VS_control.control_switcher_.switch_controllers("velocity", "moveit");
    // �Ӿ��ŷ�����
    spinner.stop();
    cout << "Start visual servoing control ... " << endl;
    cout << "Press Enter to start..." << endl;
    cin.ignore();
    KM_VS_control.initialize_time_sync();
    KM_VS_control.start_VS = true;
    ros::Rate loop_rate(KM_VS_control.control_rate_);
    while (ros::ok())
    {
        try{
            if(KM_VS_control.flag_success_){
                ROS_INFO("visual servoing success");
                KM_VS_control.start_VS = false;
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
    KM_VS_control.control_switcher_.switch_controllers("moveit", "velocity");
    spinner.start();
    // ��е���ƶ�����ʼλ��
    KM_VS_control.franka_move_to_target_joint_angle(joint_group_positions_start);
    // ����
    return 0;
}

