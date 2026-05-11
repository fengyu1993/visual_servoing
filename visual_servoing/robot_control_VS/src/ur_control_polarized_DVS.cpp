#include "ros_ur_DVS.h"
#include "key.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "visual_servoing_node");
    ROS_INFO("cyh_2");
    Ros_ur_DVS DVS_control;
    ROS_INFO("cyh_3");

    //       使用 AsyncSpinner 让 ROS 在后台线程处理回调
    cv::namedWindow("VS_image", cv::WINDOW_AUTOSIZE);
    ros::AsyncSpinner spinner(1); 

    spinner.start();
    ros::Rate rate(30); // 30Hz
    cv::Mat display_img;
    while (ros::ok()) {

        if (!DVS_control.VS_image_.empty()) {

            DVS_control.VS_image_.convertTo(display_img, CV_8U);
            cv::imshow("VS_image", display_img);

            int key = cv::waitKey(10); // 等待10ms，给人眼反应时间

            if (key == 'q' || key == 27) {

                cv::destroyAllWindows();

                ros::shutdown();

                break;

            }

        }

        rate.sleep();

    }
    return 0;

    // // 准备
    // ros::init(argc, argv, "DVS");  
    // ros::AsyncSpinner spinner(1);
    // spinner.start();  
    // cout << "Move to initial pose ... " << endl;
    // cout << "Press Enter to start the experiment ..." << endl;
    // cin.ignore();
    // Ros_ur_DVS DVS_control;
    // // 机械臂移动到起始位姿
    // DVS_control.control_switcher_.switch_controllers("position", "twist");
    // std::vector<double> joint_group_positions_start= {1.0*CV_PI/180.0, -80*CV_PI/180.0, 75*CV_PI/180.0, -75*CV_PI/180.0, -90*CV_PI/180.0, 80*CV_PI/180.0};
    // cout << "Move to initial pose ... " << endl;
    // cout << "Press Enter to start..." << endl;
    // cin.ignore();
    // DVS_control.robot_move_to_target_joint_angle(joint_group_positions_start);
    // cout << "Move to initial pose finished " << endl;
    // // 机械臂移动到初始伺服位姿
    // cout << "Move to initial VS pose ... " << endl;
    // cout << "Press Enter to start..." << endl;
    // cin.ignore();
    // DVS_control.robot_move_to_target_joint_angle(DVS_control.joint_angle_initial_VS_);
    // spinner.stop();
    // // 转换控制器
    // DVS_control.control_switcher_.switch_controllers("twist", "position");
    // // 视觉伺服控制
    // cout << "Start visual servoing control ... " << endl;
    // cout << "Press Enter to start..." << endl;
    // cin.ignore();
    // DVS_control.start_DVS = true;
    // ros::Rate loop_rate(DVS_control.control_rate_);
    // int num = 0; 

    // while (ros::ok())
    // {
    //     try{
    //         if(DVS_control.flag_success_){
    //             ROS_INFO("visual servoing success");
    //             DVS_control.start_DVS = false;
    //             break;
    //         }else{
    //             ros::spinOnce();
    //             loop_rate.sleep();
    //         }
    //     }catch(...){
    //         return 1;
    //     }
    // }
    // // 转换控制器
    // DVS_control.control_switcher_.switch_controllers("position", "twist");
    // // 机械臂移动到起始位姿
    // cout << "Move to work position ... " << endl;
    // cout << "Press Enter to start..." << endl;
    // cin.ignore();
    // spinner.start();
    // DVS_control.robot_move_to_target_joint_angle(joint_group_positions_start);
    // // 结束
    // return 0;
}

