#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <franka/robot.h>
#include "direct_visual_servoing.h"
#include "ros_DVS.h"
#include "examples_common.h"
#include "franka_control_base.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

int main(int argc, char** argv)
{
    // 准备
    ros::init(argc, argv, "DVS");  
    ros::AsyncSpinner spinner(1);
    spinner.start();  
    ControlSwitcher control_switcher;
    control_switcher.switch_controllers("moveit", "velocity");
    // ros::param::set("controller", "moveit");
    static const std::string PLANNING_GROUP = "panda_arm";
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
    // 机械臂移动到初始位姿
    std::vector<double> joint_group_positions_work = {0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4};
    franka_move_to_target_joint_angle(move_group_interface, joint_group_positions_work);
    // 机械臂移动到工作位姿
    std::vector<double> joint_group_positions_init;
    ros::param::get("joint_angle_initial", joint_group_positions_init);
    franka_move_to_target_joint_angle(move_group_interface, joint_group_positions_init);
    // 转换控制器
    control_switcher.switch_controllers("velocity", "moveit");
    // ros::param::set("controller", "velocity");
    spinner.stop();
    // 视觉伺服控制
    cout << "Start visual servoing control ... " << endl;
    cout << "Press Enter to start..." << endl;
    cin.ignore();
    Ros_DVS DVS_control;
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
        num++;
        if(num > 5)
        {
            DVS_control.start_VS = false;
            break;
        }
    }
    // 转换控制器
    control_switcher.switch_controllers("moveit", "velocity");
    // ros::param::set("controller", "moveit");
    cout << "Move to work position ... " << endl;
    cout << "Press Enter to start..." << endl;
    cin.ignore();
    spinner.start();
    // 机械臂移动到工作位姿
    franka_move_to_target_joint_angle(move_group_interface, joint_group_positions_work);
    // 结束
    return 0;
}

















