#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include "discrete_cosine_transform_vs.h"
#include "ros_HM_VS.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

int main(int argc, char** argv)
{
    // 准备
    ros::init(argc, argv, "HM_VS");  
    ros::AsyncSpinner spinner(1);
    spinner.start();  
    Ros_HM_VS HM_VS_control;
    // 机械臂移动到起始位姿
    HM_VS_control.control_switcher_.switch_controllers("moveit", "velocity");
    std::vector<double> joint_group_positions_start= {0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4};
    HM_VS_control.franka_move_to_target_joint_angle(joint_group_positions_start);
    // 机械臂移动到初始伺服位姿
    std::vector<double> joint_group_positions_init_VS;
    ros::param::get("joint_angle_initial", joint_group_positions_init_VS);
    HM_VS_control.franka_move_to_target_joint_angle(joint_group_positions_init_VS);
    // 转换控制器
    HM_VS_control.control_switcher_.switch_controllers("velocity", "moveit");
    // 视觉伺服控制
    spinner.stop();
    HM_VS_control.initialize_time_sync();
    cout << "Start visual servoing control ... " << endl;
    cout << "Press Enter to start..." << endl;
    cin.ignore();
    ros::Rate loop_rate(HM_VS_control.control_rate_);
    int num = 0;
    while (ros::ok())
    {
        try{
            if(HM_VS_control.flag_success_){
                ROS_INFO("visual servoing success");
                HM_VS_control.start_VS = false;
                break;
            }else{
                ros::spinOnce();
                loop_rate.sleep();
            }
        }catch(...){
            return 1;
        }
        num++;
        if(num > 5)
        {
            HM_VS_control.start_VS = false;
            break;
        }
    }
    // 转换控制器
    cout << "Move to work position ... " << endl;
    cout << "Press Enter to start..." << endl;
    cin.ignore();
    HM_VS_control.control_switcher_.switch_controllers("moveit", "velocity");
    spinner.start();
    // 机械臂移动到起始位姿
    HM_VS_control.franka_move_to_target_joint_angle(joint_group_positions_start);
    // 结束
    return 0;
}

