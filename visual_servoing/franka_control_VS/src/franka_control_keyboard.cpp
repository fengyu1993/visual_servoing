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
// <param name="robot_description" command="$(find xacro)/xacro --inorder '$(find realsense2_description)/urdf/test_l515_camera.urdf.xacro' use_nominal_extrinsics:=true add_plug:=true use_mesh:=true" />
using namespace cv;
using namespace std;
using namespace sensor_msgs;

void franka_move_to_target_joint_angle(moveit::planning_interface::MoveGroupInterface& move_group_interface, std::vector<double> joint_group_positions_target);
void get_camera_pose(Mat& camera_to_base, Mat& effector_to_camera);
Mat get_T(tf::StampedTransform  transform);
Mat Quaternion2Matrix (Mat q);
Mat velocity_effector_to_base(Mat& velocity, Mat& camera_to_base, Mat& effector_to_camera);
int scanKeyboard();
Mat cross_w(Mat& w);

int main(int argc, char** argv)
{
    // 准备
    ros::init(argc, argv, "DVS");  
    ros::AsyncSpinner spinner(1);
    spinner.start();  
    ros::NodeHandle nh; 
    moveit::planning_interface::MoveGroupInterface move_group_interface("panda_arm");
    Mat camera_velocity_base;
    ros::Publisher pub_camera_twist = nh.advertise<geometry_msgs::Twist>("/cartesian_velocity_node_controller/cartesian_velocity", 5); 
    // 机械臂移动到起始位姿
    ControlSwitcher     control_switcher;
    control_switcher.switch_controllers("moveit", "velocity");
    std::vector<double> joint_group_positions_start= {0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4};
    franka_move_to_target_joint_angle(move_group_interface, joint_group_positions_start);
    // 转换控制器
    control_switcher.switch_controllers("velocity", "moveit");
    spinner.stop();
    // 按键控制
    cout << "Start keyboard control ... " << endl;
    cout << "w -> +vx" << endl << "s -> -vx" << endl << "a -> +vy" << endl << "d -> -vy"<< endl << "r -> +vz" << endl << "f -> -vz" << endl;
    cout << "W -> +wx" << endl << "S -> -wx" << endl << "A -> +wy" << endl << "D -> -wy"<< endl << "R -> +wz" << endl << "F -> -wz" << endl;

    ros::Rate loop_rate(30);

    bool flag = false;
    namedWindow("cyh", WINDOW_NORMAL);
    double vel = 0.035;
    while (ros::ok())
    {
        if(flag)
            break;
        //  从按键获取速度
        Mat camera_velocity; 
        int key = waitKey(30);
        switch (key) // scanKeyboard()
        {
            case 'w' : // vx+
                camera_velocity = (Mat_<double>(6,1) << vel, 0.0, 0.0, 0.0, 0.0, 0.0);
                break;
            case 's': // vx-
                camera_velocity = (Mat_<double>(6,1) << -vel, 0.0, 0.0, 0.0, 0.0, 0.0);
                break;
            case 'a': // vy+
                camera_velocity = (Mat_<double>(6,1) << 0.0, vel, 0.0, 0.0, 0.0, 0.0);
                break;
            case 'd': // vy-
                camera_velocity = (Mat_<double>(6,1) << 0.0, -vel, 0.0, 0.0, 0.0, 0.0);
                break;
            case 'r': // vz+
                camera_velocity = (Mat_<double>(6,1) << 0.0, 0.0, vel, 0.0, 0.0, 0.0);
                break;
            case 'f': // vz-
                camera_velocity = (Mat_<double>(6,1) << 0.0, 0.0, -vel, 0.0, 0.0, 0.0);
                break;
            case 'W': // wx+
                camera_velocity = (Mat_<double>(6,1) << 0.0, 0.0, 0.0, vel, 0.0, 0.0);
                break;      
            case 'S': // wx-
                camera_velocity = (Mat_<double>(6,1) << 0.0, 0.0, 0.0, -vel, 0.0, 0.0);
                break;     
            case 'A': // wy+
                camera_velocity = (Mat_<double>(6,1) << 0.0, 0.0, 0.0, 0.0, vel, 0.0);
                break;   
            case 'D': // wy-
                camera_velocity = (Mat_<double>(6,1) << 0.0, 0.0, 0.0, 0.0, -vel, 0.0);
                break;  
            case 'R': // wz+
                camera_velocity = (Mat_<double>(6,1) << 0.0, 0.0, 0.0, 0.0, 0.0, vel);
                break; 
            case 'F': // wz-
                camera_velocity = (Mat_<double>(6,1) << 0.0, 0.0, 0.0, 0.0, 0.0, -vel);
                break; 
            case 'q':
                flag = true;
                camera_velocity = (Mat_<double>(6,1) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
                break;
            default :
                camera_velocity = (Mat_<double>(6,1) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
                break;   
        }

        // 速度转换
        Mat camera_to_base = Mat::eye(4, 4, CV_64F);
        Mat effector_to_camera = Mat::eye(4, 4, CV_64F);
        get_camera_pose(camera_to_base, effector_to_camera);
        
        Mat effector_velocity_base = velocity_effector_to_base(camera_velocity, camera_to_base, effector_to_camera);
        // Mat effector_velocity_base = camera_velocity;
        // 发布速度信息
        // cout << "effector_velocity_base = \n" << effector_velocity_base << endl;
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
    control_switcher.switch_controllers("moveit", "velocity");
    franka_move_to_target_joint_angle(move_group_interface, joint_group_positions_start);
    // 结束
    return 0;

}

void franka_move_to_target_joint_angle(moveit::planning_interface::MoveGroupInterface& move_group_interface, std::vector<double> joint_group_positions_target)
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

void get_camera_pose(Mat& camera_to_base, Mat& effector_to_camera)
{
    tf::TransformListener listener_camera_pose;
    tf::StampedTransform transform;
    listener_camera_pose.waitForTransform("panda_link0", "camera_color_optical_frame", ros::Time(0), ros::Duration(3.0));
    listener_camera_pose.lookupTransform("panda_link0", "camera_color_optical_frame", ros::Time(0), transform);
    camera_to_base = get_T(transform);

    listener_camera_pose.waitForTransform("camera_color_optical_frame", "panda_hand", ros::Time(0), ros::Duration(3.0));
    listener_camera_pose.lookupTransform("camera_color_optical_frame", "panda_hand", ros::Time(0), transform);
    effector_to_camera = get_T(transform);

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

Mat velocity_effector_to_base(Mat& velocity, Mat& camera_to_base, Mat& effector_to_camera)
{
    Mat R_camera_to_base = camera_to_base.rowRange(0,3).colRange(0,3);
    Mat p_camera_to_base = camera_to_base.rowRange(0,3).colRange(3,4);
    Mat p_cross = cross_w(p_camera_to_base);
    Mat AdT_camera_to_base = Mat::zeros(6,6,CV_64FC1);
    R_camera_to_base.copyTo(AdT_camera_to_base.rowRange(0,3).colRange(0,3));
    R_camera_to_base.copyTo(AdT_camera_to_base.rowRange(3,6).colRange(3,6));
    AdT_camera_to_base.rowRange(0,3).colRange(3,6) = p_cross * R_camera_to_base;
    Mat V_camera_to_base = AdT_camera_to_base * velocity;
    Mat v_camera_to_base = V_camera_to_base.rowRange(0,3).colRange(0,1);
    Mat w_camera_to_base = V_camera_to_base.rowRange(3,6).colRange(0,1);
    

    Mat V_effector_to_base = Mat::zeros(6,1,CV_64FC1);
    w_camera_to_base.copyTo(V_effector_to_base.rowRange(3,6).colRange(0,1));
    Mat cross_w_camera_to_base = cross_w(w_camera_to_base);
    Mat p_effector_to_camera = effector_to_camera.rowRange(0,3).colRange(3,4);
    V_effector_to_base.rowRange(0,3).colRange(0,1) = v_camera_to_base + cross_w_camera_to_base * R_camera_to_base * p_effector_to_camera;

    cout << "camera_to_base = \n" << camera_to_base << endl;
    cout << "velocity = \n" << velocity.t() << endl;
    cout << "effector_to_camera = \n" << effector_to_camera << endl;
    cout << "V_effector_to_base = \n" << V_effector_to_base.t() << endl;

    return V_effector_to_base;
}

Mat cross_w(Mat& w)
{
    double xa = w.at<double>(0,0), ya = w.at<double>(1,0), za = w.at<double>(2,0);
    Mat p_cross = (Mat_<double>(3,3)<< 0.0, -za, ya, za, 0.0, -xa, -ya, xa, 0.0);
    return p_cross;
}


       
    //    tf::Quaternion q3={X,Y,Z,W};
    //    tf::Matrix3x3 matrix;
    //    matrix.setRotation(q3);
    // ROS_INFO("x=%f, y=%f, z=%f, W=%f, X=%f, Y=%f, Z=%f", x, y, z, W, X, Y, Z);
    // cout << "T = \n" << T << endl;
    // std::cout << "matrix  = \n" << matrix[0][0] << "\t" << matrix[0][1] << "\t" << matrix[0][2] << "\n"
    //         << matrix[1][0] << "\t" << matrix[1][1] << "\t" << matrix[1][2] << "\n"
    //         << matrix[2][0] << "\t" << matrix[2][1] << "\t" << matrix[2][2] << "\n\n";
