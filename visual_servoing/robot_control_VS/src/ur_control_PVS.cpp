#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include "direct_visual_servoing.h"
#include "ros_ur_PVS.h"
#include "key.h"

int main(int argc, char** argv)
{
    // 准备
    ros::init(argc, argv, "PVS");  
    ros::AsyncSpinner spinner(1);
    spinner.start();  
    cout << "Move to initial pose ... " << endl;
    cout << "Press Enter to start the experiment ..." << endl;
    cin.ignore();
    Ros_PVS PVS_control;
    // 机械臂移动到起始位姿
    PVS_control.control_switcher_.switch_controllers("position", "twist");
    std::vector<double> joint_group_positions_start= {-73.9*CV_PI/180.0, -86.36*CV_PI/180.0, -98.73*CV_PI/180.0, -96.0*CV_PI/180.0, 72.22*CV_PI/180.0, -176.42*CV_PI/180.0};
    cout << "Move to initial pose ... " << endl;
    cout << "Press Enter to start..." << endl;
    cin.ignore();
    PVS_control.robot_move_to_target_joint_angle(joint_group_positions_start);
    cout << "Move to initial pose finished " << endl;
    // 机械臂移动到初始伺服位姿
    cout << "Move to initial VS pose ... " << endl;
    cout << "Press Enter to start..." << endl;
    cin.ignore();
    PVS_control.robot_move_to_target_joint_angle(PVS_control.joint_angle_initial_VS_);
    spinner.stop();
    // 转换控制器
    PVS_control.control_switcher_.switch_controllers("twist", "position");
    // 视觉伺服控制
    cout << "Start visual servoing control ... " << endl;
    cout << "Press Enter to start..." << endl;
    cin.ignore();
    // PVS_control.initialize_time_sync();
    // PVS_control.start_PVS = true;
    ros::Rate loop_rate(PVS_control.control_rate_);
    int num = 0; 

    auto KBC = Keyboard_ctrl();
    double vel_linear = 0.01;
    double vel_angle = 0.03;
    bool flag_finish = false;
    Mat camera_velocity = Mat::zeros(6,1,CV_64FC1);
  
    while (ros::ok())
    {
        if(flag_finish)
            break;
        //  从按键获取速度
        auto key = KBC.get_keyboard_press_key();
        switch (key) 
        {
            case KEYCODE_W : // w: vx+
                cout << "Move to vx+ ..." << endl; 
                camera_velocity = (Mat_<double>(6,1) << vel_linear, 0.0, 0.0, 0.0, 0.0, 0.0);
                break;
            case KEYCODE_S: // s: vx-
                cout << "Move to vx- ..." << endl; 
                camera_velocity = (Mat_<double>(6,1) << -vel_linear, 0.0, 0.0, 0.0, 0.0, 0.0);
                break;
            case KEYCODE_A: // a: vy+
                cout << "Move to vy+ ..." << endl; 
                camera_velocity = (Mat_<double>(6,1) << 0.0, vel_linear, 0.0, 0.0, 0.0, 0.0);
                break;
            case KEYCODE_D: // d: vy-
                cout << "Move to vy- ..." << endl; 
                camera_velocity = (Mat_<double>(6,1) << 0.0, -vel_linear, 0.0, 0.0, 0.0, 0.0);
                break;
            case KEYCODE_R: // r: vz+
                cout << "Move to vz+ ..." << endl; 
                camera_velocity = (Mat_<double>(6,1) << 0.0, 0.0, vel_linear, 0.0, 0.0, 0.0);
                break;
            case KEYCODE_F: // f: vz-
                cout << "Move to vz- ..." << endl; 
                camera_velocity = (Mat_<double>(6,1) << 0.0, 0.0, -vel_linear, 0.0, 0.0, 0.0);
                break;
            case KEYCODE_Q: // q: stop
                cout << "Stop ..." << endl; 
                flag_finish = true;
                camera_velocity = (Mat_<double>(6,1) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
                break;
            case KEYCODE_W_CAP: // W: wx+
                cout << "Move to wx+ ..." << endl; 
                camera_velocity = (Mat_<double>(6,1) << 0.0, 0.0, 0.0, vel_angle, 0.0, 0.0);
                break;      
            case KEYCODE_S_CAP: // S: wx-
                cout << "Move to wx- ..." << endl; 
                camera_velocity = (Mat_<double>(6,1) << 0.0, 0.0, 0.0, -vel_angle, 0.0, 0.0);
                break;     
            case KEYCODE_A_CAP: // A: wy+
                cout << "Move to vy+ ..." << endl; 
                camera_velocity = (Mat_<double>(6,1) << 0.0, 0.0, 0.0, 0.0, vel_angle, 0.0);
                break;   
            case KEYCODE_D_CAP: // D: wy-
                cout << "Move to wy- ..." << endl; 
                camera_velocity = (Mat_<double>(6,1) << 0.0, 0.0, 0.0, 0.0, -vel_angle, 0.0);
                break;  
            case KEYCODE_R_CAP: // R: wz+
                cout << "Move to wz+ ..." << endl; 
                camera_velocity = (Mat_<double>(6,1) << 0.0, 0.0, 0.0, 0.0, 0.0, vel_angle);
                break; 
            case KEYCODE_F_CAP: // F: wz-
                cout << "Move to wz- ..." << endl; 
                camera_velocity = (Mat_<double>(6,1) << 0.0, 0.0, 0.0, 0.0, 0.0, -vel_angle);
                break; 
            case KEYCODE_Q_CAP: // Q: stop
                cout << "Stop ..." << endl; 
                flag_finish = true;
                camera_velocity = (Mat_<double>(6,1) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
                break;
            default :
                camera_velocity = (Mat_<double>(6,1) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
                break;   
        }

        PVS_control.twist_publist(camera_velocity);
        // 休息
        loop_rate.sleep();
        // try{
        //     if(PVS_control.flag_success_){
        //         ROS_INFO("visual servoing success");
        //         PVS_control.start_PVS = false;
        //         break;
        //     }else{
        //         ros::spinOnce();
        //         loop_rate.sleep();
        //     }
        // }catch(...){
        //     return 1;
        // }
    }
    // 转换控制器
    PVS_control.control_switcher_.switch_controllers("position", "twist");
    // 机械臂移动到起始位姿
    cout << "Move to work position ... " << endl;
    cout << "Press Enter to start..." << endl;
    cin.ignore();
    spinner.start();
    PVS_control.robot_move_to_target_joint_angle(joint_group_positions_start);
    // 结束
    return 0;
}

