#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include "direct_visual_servoing.h"
#include "ros_ur_PVS.h"
#include "key.h"
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {

    try {
        // 验证图像格式
        if (msg->encoding != sensor_msgs::image_encodings::TYPE_8UC4) {
            ROS_WARN_THROTTLE(1.0, "Invalid encoding: %s (expected 8UC4)", 
                            msg->encoding.c_str());
            return;
        }
        // 转换为OpenCV格式
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC4);
        // 拆分为四个单通道图像
        std::vector<cv::Mat> channels;
        cv::split(cv_ptr->image, channels);      
        if (channels.size() < 4) {
            ROS_ERROR("Expected 4 channels, got %zu", channels.size());
            return;
        }
        // 重命名通道
        cv::Mat gray_0 = channels[0];  
        cv::Mat gray_45 = channels[1];   
        cv::Mat gray_90 = channels[2];   
        cv::Mat gray_135 = channels[3];  
        // 显示
        cv::imshow("I0", gray_0);
        cv::imshow("I45", gray_45);
        cv::imshow("I90", gray_90);
        cv::imshow("I135", gray_135);
        int key = cv::waitKey(1);
        // 退出
        if (key == 'q' || key == 27) {
            ros::shutdown();
        }
    } catch (const cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    } catch (const std::exception& e) {
        ROS_ERROR("Exception: %s", e.what());
    }

}


int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "gray_4ch_viewer");
    ros::NodeHandle nh;
    // 创建图像传输对象
    image_transport::ImageTransport it(nh);
    // 创建窗口
    cv::namedWindow("I0", cv::WINDOW_AUTOSIZE);  cv::resizeWindow("I0",  306, 256);
    cv::namedWindow("I45", cv::WINDOW_AUTOSIZE); cv::resizeWindow("I45", 306, 256);
    cv::namedWindow("I90", cv::WINDOW_AUTOSIZE); cv::resizeWindow("I90", 306, 256);
    cv::namedWindow("I135", cv::WINDOW_AUTOSIZE);cv::resizeWindow("I135",306, 256);
    // 订阅四通道图像
    image_transport::Subscriber sub = it.subscribe("camera/polarized_Iall_gray",  1, imageCallback);
    ROS_INFO("Started polarized angles viewer. Press 'q' to quit.");
    // 保持运行
    ros::spin();
    // 清理
    cv::destroyAllWindows();
    return 0;

    // // 准备
    // ros::init(argc, argv, "PVS");  
    // ros::AsyncSpinner spinner(1);
    // spinner.start();  
    // cout << "Move to initial pose ... " << endl;
    // cout << "Press Enter to start the experiment ..." << endl;
    // cin.ignore();
    // Ros_PVS PVS_control;
    // // 机械臂移动到起始位姿
    // PVS_control.control_switcher_.switch_controllers("position", "twist");
    // std::vector<double> joint_group_positions_start= {-73.9*CV_PI/180.0, -86.36*CV_PI/180.0, -98.73*CV_PI/180.0, -96.0*CV_PI/180.0, 72.22*CV_PI/180.0, -176.42*CV_PI/180.0};
    // cout << "Move to initial pose ... " << endl;
    // cout << "Press Enter to start..." << endl;
    // cin.ignore();
    // PVS_control.robot_move_to_target_joint_angle(joint_group_positions_start);
    // cout << "Move to initial pose finished " << endl;
    // // 机械臂移动到初始伺服位姿
    // cout << "Move to initial VS pose ... " << endl;
    // cout << "Press Enter to start..." << endl;
    // cin.ignore();
    // PVS_control.robot_move_to_target_joint_angle(PVS_control.joint_angle_initial_VS_);
    // spinner.stop();
    // // 转换控制器
    // PVS_control.control_switcher_.switch_controllers("twist", "position");
    // // 视觉伺服控制
    // cout << "Start visual servoing control ... " << endl;
    // cout << "Press Enter to start..." << endl;
    // cin.ignore();
    // // PVS_control.initialize_time_sync();
    // // PVS_control.start_PVS = true;
    // ros::Rate loop_rate(PVS_control.control_rate_);
    // int num = 0; 

    // auto KBC = Keyboard_ctrl();
    // double vel_linear = 0.01;
    // double vel_angle = 0.03;
    // bool flag_finish = false;
    // Mat camera_velocity = Mat::zeros(6,1,CV_64FC1);
  
    // while (ros::ok())
    // {
    //     if(flag_finish)
    //         break;
    //     //  从按键获取速度
    //     auto key = KBC.get_keyboard_press_key();
    //     switch (key) 
    //     {
    //         case KEYCODE_W : // w: vx+
    //             cout << "Move to vx+ ..." << endl; 
    //             camera_velocity = (Mat_<double>(6,1) << vel_linear, 0.0, 0.0, 0.0, 0.0, 0.0);
    //             break;
    //         case KEYCODE_S: // s: vx-
    //             cout << "Move to vx- ..." << endl; 
    //             camera_velocity = (Mat_<double>(6,1) << -vel_linear, 0.0, 0.0, 0.0, 0.0, 0.0);
    //             break;
    //         case KEYCODE_A: // a: vy+
    //             cout << "Move to vy+ ..." << endl; 
    //             camera_velocity = (Mat_<double>(6,1) << 0.0, vel_linear, 0.0, 0.0, 0.0, 0.0);
    //             break;
    //         case KEYCODE_D: // d: vy-
    //             cout << "Move to vy- ..." << endl; 
    //             camera_velocity = (Mat_<double>(6,1) << 0.0, -vel_linear, 0.0, 0.0, 0.0, 0.0);
    //             break;
    //         case KEYCODE_R: // r: vz+
    //             cout << "Move to vz+ ..." << endl; 
    //             camera_velocity = (Mat_<double>(6,1) << 0.0, 0.0, vel_linear, 0.0, 0.0, 0.0);
    //             break;
    //         case KEYCODE_F: // f: vz-
    //             cout << "Move to vz- ..." << endl; 
    //             camera_velocity = (Mat_<double>(6,1) << 0.0, 0.0, -vel_linear, 0.0, 0.0, 0.0);
    //             break;
    //         case KEYCODE_Q: // q: stop
    //             cout << "Stop ..." << endl; 
    //             flag_finish = true;
    //             camera_velocity = (Mat_<double>(6,1) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    //             break;
    //         case KEYCODE_W_CAP: // W: wx+
    //             cout << "Move to wx+ ..." << endl; 
    //             camera_velocity = (Mat_<double>(6,1) << 0.0, 0.0, 0.0, vel_angle, 0.0, 0.0);
    //             break;      
    //         case KEYCODE_S_CAP: // S: wx-
    //             cout << "Move to wx- ..." << endl; 
    //             camera_velocity = (Mat_<double>(6,1) << 0.0, 0.0, 0.0, -vel_angle, 0.0, 0.0);
    //             break;     
    //         case KEYCODE_A_CAP: // A: wy+
    //             cout << "Move to vy+ ..." << endl; 
    //             camera_velocity = (Mat_<double>(6,1) << 0.0, 0.0, 0.0, 0.0, vel_angle, 0.0);
    //             break;   
    //         case KEYCODE_D_CAP: // D: wy-
    //             cout << "Move to wy- ..." << endl; 
    //             camera_velocity = (Mat_<double>(6,1) << 0.0, 0.0, 0.0, 0.0, -vel_angle, 0.0);
    //             break;  
    //         case KEYCODE_R_CAP: // R: wz+
    //             cout << "Move to wz+ ..." << endl; 
    //             camera_velocity = (Mat_<double>(6,1) << 0.0, 0.0, 0.0, 0.0, 0.0, vel_angle);
    //             break; 
    //         case KEYCODE_F_CAP: // F: wz-
    //             cout << "Move to wz- ..." << endl; 
    //             camera_velocity = (Mat_<double>(6,1) << 0.0, 0.0, 0.0, 0.0, 0.0, -vel_angle);
    //             break; 
    //         case KEYCODE_Q_CAP: // Q: stop
    //             cout << "Stop ..." << endl; 
    //             flag_finish = true;
    //             camera_velocity = (Mat_<double>(6,1) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    //             break;
    //         default :
    //             camera_velocity = (Mat_<double>(6,1) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    //             break;   
    //     }

    //     PVS_control.twist_publist(camera_velocity);
    //     // 休息
    //     loop_rate.sleep();
    //     // try{
    //     //     if(PVS_control.flag_success_){
    //     //         ROS_INFO("visual servoing success");
    //     //         PVS_control.start_PVS = false;
    //     //         break;
    //     //     }else{
    //     //         ros::spinOnce();
    //     //         loop_rate.sleep();
    //     //     }
    //     // }catch(...){
    //     //     return 1;
    //     // }
    // }
    // // 转换控制器
    // PVS_control.control_switcher_.switch_controllers("position", "twist");
    // // 机械臂移动到起始位姿
    // cout << "Move to work position ... " << endl;
    // cout << "Press Enter to start..." << endl;
    // cin.ignore();
    // spinner.start();
    // PVS_control.robot_move_to_target_joint_angle(joint_group_positions_start);
    // // 结束
    // return 0;
}

