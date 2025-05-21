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
#include <control_switcher_ur.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <thread>
#include <cartesian_interface/cartesian_command_interface.h>
// #include <twist_controller/TwistControllerConfig.h>
#include "key.h"
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

using namespace cv;
using namespace std;
using namespace sensor_msgs;

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Client;

void robot_move_to_target_joint_angle(Client& client, control_msgs::FollowJointTrajectoryGoal& goal, std::vector<double> joint_group_positions_target);
void get_camera_pose(tf::TransformListener& listener_pose, Mat& effector_to_base, Mat& camera_to_effector);
Mat get_T(tf::StampedTransform  transform);
Mat velocity_effector_to_base(Mat velocity, Mat camera_to_base);
Mat get_effector_velocity_base(Mat camera_velocity, Mat effector_to_camera);

Mat joint_group_positions;

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    joint_group_positions = (Mat_<double>(1,6) << msg->position[0], msg->position[1], 
                            msg->position[2], msg->position[3], msg->position[4], 
                            msg->position[5]);
    joint_group_positions = joint_group_positions * 180.0 / CV_PI;
    // ROS_INFO("Joint Positions:");
    // for(size_t i = 0; i < msg->position.size(); ++i) {
    //     ROS_INFO("Joint %d position: %f", static_cast<int>(i), msg->position[i]);
    // }
}


int main(int argc, char** argv)
{
    // 准备
    ros::init(argc, argv, "control_keyboard");  
    ros::AsyncSpinner spinner(1);
    spinner.start();  
    ros::NodeHandle nh; 
    Mat camera_velocity_base;
    ros::Publisher pub_camera_twist = nh.advertise<geometry_msgs::Twist>("/twist_controller/command", 5); 
    ros::Subscriber sub = nh.subscribe("/joint_states", 1000, jointStateCallback);
    tf::TransformListener listener_pose;
    // 创建动作客户端，连接到名为"pos_joint_traj_controller/follow_joint_trajectory"的动作服务器
    Client client("pos_joint_traj_controller/follow_joint_trajectory", true);
    ROS_INFO("Waiting for action server to start.");
    client.waitForServer();  // 将会一直等待直到动作服务器可用
    ROS_INFO("Server started, sending goal.");
    // 创建目标消息
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names.push_back("shoulder_pan_joint");
    goal.trajectory.joint_names.push_back("shoulder_lift_joint");
    goal.trajectory.joint_names.push_back("elbow_joint");
    goal.trajectory.joint_names.push_back("wrist_1_joint");
    goal.trajectory.joint_names.push_back("wrist_2_joint");
    goal.trajectory.joint_names.push_back("wrist_3_joint");
    // 机械臂移动到起始位姿
    ControlSwitcher_UR     control_switcher;
    cout << "Move to initial pose ... " << endl;
    cout << "Press Enter to start..." << endl;
    cin.ignore();
    control_switcher.switch_controllers("position", "twist");
    std::vector<double> joint_group_positions_start= {1.0*CV_PI/180.0, -80*CV_PI/180.0, 75*CV_PI/180.0, -75*CV_PI/180.0, -90*CV_PI/180.0, 80*CV_PI/180.0};
    cout << "Press Enter to start..." << endl;
    cin.ignore();
    robot_move_to_target_joint_angle(client, goal, joint_group_positions_start);
    cout << "Move to initial pose finished " << endl;
    // 按键控制
    cout << "Start keyboard control ... " << endl;
    cout << "w -> +vx" << endl << "s -> -vx" << endl << "a -> +vy" << endl << "d -> -vy"<< endl << "r -> +vz" << endl << "f -> -vz" << endl;
    cout << "W -> +wx" << endl << "S -> -wx" << endl << "A -> +wy" << endl << "D -> -wy"<< endl << "R -> +wz" << endl << "F -> -wz" << endl;
    cout << "Press Enter to start..." << endl;
    cin.ignore();
    // 转换控制器
    control_switcher.switch_controllers("twist", "position");
    spinner.stop();
    // 控制
    ros::Rate loop_rate(30);
    auto KBC = Keyboard_ctrl();
    double vel_linear = 0.01;
    double vel_angle = 0.04;
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
            case KEYCODE_J: // Output joint variables
                cout << "joint_angle = \n" <<  joint_group_positions << endl;
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
            case KEYCODE_J_CAP: // Output joint variables
                cout << "joint_angle = \n" <<  joint_group_positions << endl;
                break;
            default:
                camera_velocity = (Mat_<double>(6,1) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
                break;   
        }
        // // 速度转换
        Mat effector_to_base = Mat::eye(4, 4, CV_64F);
        Mat camera_to_effector = Mat::eye(4, 4, CV_64F);
        get_camera_pose(listener_pose, effector_to_base, camera_to_effector);
        Mat effector_twist = get_effector_velocity_base(camera_velocity, camera_to_effector);
        // Mat effector_twist = camera_velocity;
        Mat effector_velocity_base = velocity_effector_to_base(effector_twist, effector_to_base);
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
    cout << "Move to initial pose ... " << endl;
    cout << "Press Enter to start..." << endl;
    cin.ignore();
    control_switcher.switch_controllers("position", "twist");;
    robot_move_to_target_joint_angle(client, goal, joint_group_positions_start);
    // 结束
    return 0;

}

void robot_move_to_target_joint_angle(Client& client, control_msgs::FollowJointTrajectoryGoal& goal, std::vector<double> joint_group_positions_target)
{
    // 添加轨迹点
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.resize(6); 
    for (int i=0; i<6; i++)
        point.positions[i] = joint_group_positions_target[i];
    point.time_from_start = ros::Duration(5.0);
    goal.trajectory.points.clear();
    goal.trajectory.points.push_back(point);
    goal.trajectory.header.stamp = ros::Time::now(); 
    client.sendGoal(goal);
    // 等待结果
    bool finished_before_timeout = client.waitForResult(ros::Duration(6.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = client.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
    }
    else
        ROS_INFO("Action did not finish before the time out.");
}

void get_camera_pose(tf::TransformListener& listener_pose, Mat& effector_to_base, Mat& camera_to_effector)
{
    tf::StampedTransform transform;
    const int max_attempts = 5;
    const double retry_delay = 0.1;
    for (int attempt = 0; attempt < max_attempts; ++attempt)
    {
        try
        {
            tf::StampedTransform transform; //tool0_controller
            // 尝试获取当前时刻的变换
            listener_pose.lookupTransform("base", "tool0_controller", ros::Time(0), transform);
            effector_to_base = get_T(transform);

            listener_pose.lookupTransform("tool0_controller", "camera_polar_frame", ros::Time(0), transform);
            camera_to_effector = get_T(transform);  
        }
        catch (tf::TransformException &ex)
        {
            // 如果获取变换失败，打印错误信息并稍作等待
            ROS_WARN("Failed to get transform, retrying... (%d/%d)", attempt + 1, max_attempts);
            ros::Duration(retry_delay).sleep();
        }
    }
}

Mat get_T(tf::StampedTransform  transform)
{
    tf::Matrix3x3 rotation_matrix = transform.getBasis();
    tf::Vector3 translation_vector = transform.getOrigin();

    Mat T = Mat::eye(4,4,CV_64FC1);
    Mat p = (Mat_<double>(3,1) << translation_vector[0], translation_vector[1], translation_vector[2]);
    p.copyTo(T.rowRange(0,3).colRange(3,4));
    Mat R = (Mat_<double>(3,3) << rotation_matrix[0][0], rotation_matrix[0][1], rotation_matrix[0][2], 
                                  rotation_matrix[1][0], rotation_matrix[1][1], rotation_matrix[1][2], 
                                  rotation_matrix[2][0], rotation_matrix[2][1], rotation_matrix[2][2]);
    R.copyTo(T.rowRange(0,3).colRange(0,3));

    return T;    
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



