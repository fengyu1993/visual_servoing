#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Twist.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <librealsense2/rs.hpp>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/ListControllers.h>
#include <map>
#include <string>
#include <moveit/move_group_interface/move_group_interface.h>
#include <thread>
#include "ros_VS.h"
#include <cartesian_interface/cartesian_command_interface.h>
#include <twist_controller/TwistControllerConfig.h>
#include "key.h"


using namespace cv;
using namespace std;
using namespace sensor_msgs;

void robot_move_to_target_joint_angle(moveit::planning_interface::MoveGroupInterface& move_group_interface, std::vector<double> joint_group_positions_target);
void get_camera_pose(Mat& effector_to_base, Mat& camera_to_effector);
Mat get_T(tf::StampedTransform  transform);
Mat Quaternion2Matrix (Mat q);
Mat velocity_effector_to_base(Mat velocity, Mat camera_to_base);
Mat get_effector_velocity_base(Mat camera_velocity, Mat effector_to_camera);

int main(int argc, char** argv)
{
    // 准备
    ros::init(argc, argv, "control_keyboard");  
    ros::AsyncSpinner spinner(1);
    spinner.start();  
    ros::NodeHandle nh; 
    // moveit::planning_interface::MoveGroupInterface move_group_interface("manipulator");
    Mat camera_velocity_base;
    ros::Publisher pub_camera_twist = nh.advertise<geometry_msgs::Twist>("/twist_controller/command", 5); 
    // cout << "\ncyh_2\n" << endl;
    // 机械臂移动到起始位姿
    // ControlSwitcher     control_switcher;
    // cout << "\ncyh_3\n" << endl;
    // control_switcher.switch_controllers("moveit", "ros_controllers_cartesian");
    // cout << "\ncyh_4\n" << endl;
    // std::vector<double> joint_group_positions_start= {0, -CV_PI/3.0, CV_PI/3.0, -CV_PI/2.0, -CV_PI/2.0, 0};
    std::vector<double> joint_group_positions_start= {-62.34, -79.66, -87.52, -199.25, -55.19, 84.33};
    // cout << "Move to initial pose ... " << endl;
    // cout << "Press Enter to start..." << endl;
    // cin.ignore();
    // robot_move_to_target_joint_angle(move_group_interface, joint_group_positions_start);
    // 转换控制器
    
    // control_switcher.switch_controllers("twist_controller", "moveit");
    spinner.stop();
    // 按键控制
    cout << "Start keyboard control ... " << endl;
    cout << "w -> +vx" << endl << "s -> -vx" << endl << "a -> +vy" << endl << "d -> -vy"<< endl << "r -> +vz" << endl << "f -> -vz" << endl;
    cout << "W -> +wx" << endl << "S -> -wx" << endl << "A -> +wy" << endl << "D -> -wy"<< endl << "R -> +wz" << endl << "F -> -wz" << endl;

    cout << "Press Enter to start..." << endl;
    cin.ignore();
    ros::Rate loop_rate(30);

    auto KBC = Keyboard_ctrl();
    double vel_linear = 0.01;
    double vel_angle = 0.04;
    Mat camera_velocity = Mat::zeros(6,1,CV_64FC1);

    int cnt = 0;

    while (ros::ok())
    {
        //  从按键获取速度
        auto key = KBC.get_keyboard_press_key();

        switch (key) 
        {
            case KEYCODE_W : // w: vx+
                camera_velocity = (Mat_<double>(6,1) << vel_linear, 0.0, 0.0, 0.0, 0.0, 0.0);
                break;
            case KEYCODE_S: // s: vx-
                camera_velocity = (Mat_<double>(6,1) << -vel_linear, 0.0, 0.0, 0.0, 0.0, 0.0);
                break;
            case KEYCODE_A: // a: vy+
                camera_velocity = (Mat_<double>(6,1) << 0.0, vel_linear, 0.0, 0.0, 0.0, 0.0);
                break;
            case KEYCODE_D: // d: vy-
                camera_velocity = (Mat_<double>(6,1) << 0.0, -vel_linear, 0.0, 0.0, 0.0, 0.0);
                break;
            case KEYCODE_R: // r: vz+
                camera_velocity = (Mat_<double>(6,1) << 0.0, 0.0, vel_linear, 0.0, 0.0, 0.0);
                break;
            case KEYCODE_F: // f: vz-
                camera_velocity = (Mat_<double>(6,1) << 0.0, 0.0, -vel_linear, 0.0, 0.0, 0.0);
                break;
            case KEYCODE_Q: // q: stop
                camera_velocity = (Mat_<double>(6,1) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
                break;
            case KEYCODE_W_CAP: // W: wx+
                camera_velocity = (Mat_<double>(6,1) << 0.0, 0.0, 0.0, vel_angle, 0.0, 0.0);
                break;      
            case KEYCODE_S_CAP: // S: wx-
                camera_velocity = (Mat_<double>(6,1) << 0.0, 0.0, 0.0, -vel_angle, 0.0, 0.0);
                break;     
            case KEYCODE_A_CAP: // A: wy+
                camera_velocity = (Mat_<double>(6,1) << 0.0, 0.0, 0.0, 0.0, vel_angle, 0.0);
                break;   
            case KEYCODE_D_CAP: // D: wy-
                camera_velocity = (Mat_<double>(6,1) << 0.0, 0.0, 0.0, 0.0, -vel_angle, 0.0);
                break;  
            case KEYCODE_R_CAP: // R: wz+
                camera_velocity = (Mat_<double>(6,1) << 0.0, 0.0, 0.0, 0.0, 0.0, vel_angle);
                break; 
            case KEYCODE_F_CAP: // F: wz-
                camera_velocity = (Mat_<double>(6,1) << 0.0, 0.0, 0.0, 0.0, 0.0, -vel_angle);
                break; 
            case KEYCODE_Q_CAP: // Q: stop
                camera_velocity = (Mat_<double>(6,1) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
                break;
            default :
                camera_velocity = (Mat_<double>(6,1) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
                break;   
        }

        // // 速度转换
        Mat effector_to_base = Mat::eye(4, 4, CV_64F);
        Mat camera_to_effector = Mat::eye(4, 4, CV_64F);
        get_camera_pose(effector_to_base, camera_to_effector);
        Mat effector_twist = get_effector_velocity_base(camera_velocity, camera_to_effector);
        // Mat effector_twist = camera_velocity;
        Mat effector_velocity_base = velocity_effector_to_base(effector_twist, effector_to_base);
        // Mat effector_velocity_base = camera_velocity;
        // 发布速度信息
        geometry_msgs::Twist effector_Twist;
        effector_Twist.linear.x = effector_velocity_base.at<double>(0,0);
        effector_Twist.linear.y = effector_velocity_base.at<double>(1,0);
        effector_Twist.linear.z = effector_velocity_base.at<double>(2,0);
        effector_Twist.angular.x = effector_velocity_base.at<double>(3,0);
        effector_Twist.angular.y = effector_velocity_base.at<double>(4,0);
        effector_Twist.angular.z = effector_velocity_base.at<double>(5,0);
        pub_camera_twist.publish(effector_Twist);
        // 休息
        loop_rate.sleep();
    }



    // 机械臂移动到起始位姿
    // control_switcher.switch_controllers("moveit", "twist_controller");
    // robot_move_to_target_joint_angle(move_group_interface, joint_group_positions_start);
    // 结束
    return 0;

}

void robot_move_to_target_joint_angle(moveit::planning_interface::MoveGroupInterface& move_group_interface, std::vector<double> joint_group_positions_target)
{
    move_group_interface.setJointValueTarget(joint_group_positions_target);
    move_group_interface.setMaxAccelerationScalingFactor(0.05);
    move_group_interface.setMaxVelocityScalingFactor(0.05);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(success)
    {
        std::cout << "Press Enter to move the robot..." << std::endl;
        std::cin.ignore();
        move_group_interface.move();
        std::cout << "Move finish" << std::endl;
    }
    else
    {
        std::cout << "moveit joint plan fail ! ! !" << std::endl;
    }
}

void get_camera_pose(Mat& effector_to_base, Mat& camera_to_effector)
{
    tf::TransformListener listener_camera_pose;
    tf::StampedTransform transform;//
    listener_camera_pose.waitForTransform("base", "tool0_controller", ros::Time(0), ros::Duration(3.0));
    listener_camera_pose.lookupTransform("base", "tool0_controller", ros::Time(0), transform);
    effector_to_base = get_T(transform);

    listener_camera_pose.waitForTransform("tool0_controller", "camera_polar_frame", ros::Time(0), ros::Duration(3.0));
    listener_camera_pose.lookupTransform("tool0_controller", "camera_polar_frame", ros::Time(0), transform);
    camera_to_effector = get_T(transform);

}

Mat get_T(tf::StampedTransform  transform)
{
    double x = transform.getOrigin().getX();
    double y = transform.getOrigin().getY();
    double z = transform.getOrigin().getZ();
    double W = transform.getRotation().getW();
    double X = transform.getRotation().getX();
    double Y = transform.getRotation().getY();
    double Z = transform.getRotation().getZ();

    Mat T = Mat::eye(4,4,CV_64FC1);
    Mat p = (Mat_<double>(3,1) << x, y, z);
    p.copyTo(T.rowRange(0,3).colRange(3,4));
    Mat q = (Mat_<double>(4,1) << W, X, Y, Z);
    Mat R = Quaternion2Matrix(q);
    R.copyTo(T.rowRange(0,3).colRange(0,3));

    return T;    
}

Mat Quaternion2Matrix (Mat q)
{
  double w = q.at<double>(0);
  double x = q.at<double>(1);
  double y = q.at<double>(2);
  double z = q.at<double>(3);

  double xx = x*x;
  double yy = y*y;
  double zz = z*z;
  double xy = x*y;
  double wz = w*z;
  double wy = w*y;
  double xz = x*z;
  double yz = y*z;
  double wx = w*x;

  double ret[3][3];
  ret[0][0] = 1.0-2*(yy+zz);
  ret[0][1] = 2*(xy-wz);
  ret[0][2] = 2*(wy+xz);
 
  ret[1][0] = 2*(xy+wz);
  ret[1][1] = 1.0-2*(xx+zz);
  ret[1][2] = 2*(yz-wx);
 
  ret[2][0] = 2*(xz-wy);
  ret[2][1] = 2*(yz+wx);
  ret[2][2] = 1.0-2*(xx+yy);
 
  return cv::Mat(3,3,CV_64FC1,ret).clone();    
}

Mat velocity_effector_to_base(Mat velocity, Mat camera_to_base)
{
    Mat R_camera_to_base = camera_to_base.rowRange(0,3).colRange(0,3);
    Mat V_effector_to_base = Mat::zeros(6,1,CV_64FC1);
    V_effector_to_base.rowRange(0,3).colRange(0,1) = R_camera_to_base * velocity.rowRange(0,3).colRange(0,1);
    V_effector_to_base.rowRange(3,6).colRange(0,1) = R_camera_to_base * velocity.rowRange(3,6).colRange(0,1);

    return V_effector_to_base;
}

Mat get_effector_velocity_base(Mat camera_velocity, Mat camera_to_effector)
{
    Mat AdT = Mat::zeros(6,6,CV_64FC1);
    Mat R = camera_to_effector.rowRange(0,3).colRange(0,3);
    Mat p = camera_to_effector.rowRange(0,3).colRange(3,4);
    Mat effector_velocity_base= Mat::zeros(6,1,CV_64FC1);
    Mat p_so3 = (Mat_<double>(3,3) << 0, -p.at<double>(2,0), p.at<double>(1,0), 
                                p.at<double>(2,0), 0, -p.at<double>(0,0),
                                 -p.at<double>(1,0), p.at<double>(0,0), 0);
    effector_velocity_base.rowRange(0,3).colRange(0,1) = R * camera_velocity.rowRange(0,3).colRange(0,1) + 
                        p_so3 * R * camera_velocity.rowRange(3,6).colRange(0,1);                   
    effector_velocity_base.rowRange(3,6).colRange(0,1) = R * camera_velocity.rowRange(3,6).colRange(0,1);

    return  effector_velocity_base;
}



