#ifndef ROS_UR_DVS
#define ROS_UR_DVS

#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/ListControllers.h>
#include "direct_visual_servoing.h"
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include "control_switcher_ur.h"

using namespace cv;
using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Client;

class Ros_ur_DVS 
{
    protected:
        ros::NodeHandle                         nh_; 
        message_filters::Subscriber<Image>      image_color_sub_;
        message_filters::Subscriber<Image>      image_depth_sub_;
        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *client;
        control_msgs::FollowJointTrajectoryGoal goal;
        TimeSynchronizer<Image, Image>          *sync_;
        tf::TransformListener                   listener_pose_;
        Mat                                     effector_velocity_base_;
        ros::Publisher                          pub_twist_; 

    public:
        Direct_Visual_Servoing *DVS;
        double              control_rate_;
        bool                flag_success_;
        Mat                 joint_angle_initial_VS_;
        bool                start_DVS;
        ControlSwitcher_UR control_switcher_;
        string name_link0_, name_camera_frame_, name_effector_;

    public:
        Ros_ur_DVS();
        void Callback(const ImageConstPtr& image_polar_msg, const ImageConstPtr& image_depth_msg);     
        void get_parameters_resolution(int& resolution_x, int& resolution_y);
        Mat get_parameter_Matrix(string str, int row, int col);
        void initialize_time_sync();
        void get_image_data_convert(const ImageConstPtr& image_polar_msg, const ImageConstPtr& image_depth_msg, Mat& color_img, Mat& depth_img);
        Mat rgb_image_operate(Mat& image_rgb);
        Mat depth_image_operate(Mat& image_depth);
        Mat get_T(tf::StampedTransform  transform);
        void get_parameters_DVS(double& lambda, double& epsilon, Mat& image_gray_desired, Mat& image_depth_desired, Mat& camera_intrinsic, Mat& pose_desired);    
        void robot_move_to_target_joint_angle(std::vector<double> joint_group_positions_target);
        void get_camera_effector_pose(Mat& effector_to_base, Mat& camera_to_effector);
        Mat get_camera_pose();
        Mat get_effector_velocity(Mat camera_velocity, Mat effector_to_camera);
        Mat velocity_effector_to_base(Mat velocity, Mat effector_to_base);
        void twist_publist(Mat camera_velocity);
};

#endif


