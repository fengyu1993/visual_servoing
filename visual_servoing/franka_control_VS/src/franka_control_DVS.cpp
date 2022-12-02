#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include "direct_visual_servoing.h"
#include "ros_DVS.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

int main(int argc, char** argv)
{
    // 准备
    ros::init(argc, argv, "DVS");  
    ros::AsyncSpinner spinner(1);
    spinner.start();  
    Ros_DVS DVS_control;
    // 机械臂移动到起始位姿
    DVS_control.control_switcher_.switch_controllers("moveit", "velocity");
    std::vector<double> joint_group_positions_start= {0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4};
    DVS_control.franka_move_to_target_joint_angle(joint_group_positions_start);
    // 机械臂移动到初始伺服位姿
    std::vector<double> joint_group_positions_init_VS;
    ros::param::get("joint_angle_initial", joint_group_positions_init_VS);
    DVS_control.franka_move_to_target_joint_angle(joint_group_positions_init_VS);
    // 转换控制器
    DVS_control.control_switcher_.switch_controllers("velocity", "moveit");
    // 视觉伺服控制
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
    // 转换控制器
    cout << "Move to work position ... " << endl;
    cout << "Press Enter to start..." << endl;
    cin.ignore();
    DVS_control.control_switcher_.switch_controllers("moveit", "velocity");
    spinner.start();
    // 机械臂移动到起始位姿
    DVS_control.franka_move_to_target_joint_angle(joint_group_positions_start);
    // 结束
    return 0;
}

