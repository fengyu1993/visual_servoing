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
#include <image_transport/image_transport.h>
#include <robot_control_VS/PolarizedImages.h>

using namespace cv;
using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Client;

class Ros_ur_DVS 
{
    protected:
        ros::NodeHandle                         nh_; 
        ros::Subscriber             image_polarized_sub_;
        Client                                  *client;
        control_msgs::FollowJointTrajectoryGoal goal;
        tf::TransformListener                   listener_pose_;
        ros::Publisher                          pub_twist_; 
        geometry_msgs::Twist                    msg_camera_twist_;

    public:
        Direct_Visual_Servoing *DVS;
        double              control_rate_;
        double              itera_num_all_;
        bool                flag_success_;
        Mat                 joint_angle_initial_VS_;
        Mat                 joint_angle_start_;    
        Mat                 depth_new_;
        Mat                 img_new_;
        bool                start_DVS;
        ControlSwitcher_UR  control_switcher_;
        string              name_link0_, name_camera_frame_, name_effector_;
        Mat                 R_temp_;
        Mat                 p_temp_;
        double              px_temp_, py_temp_, pz_temp_;
        Mat                 p_so3_temp_;
        Mat                 effector_twist_;
        Mat                 effector_twist_base_;
        Mat                 T_effector_to_base_;
        Mat                 T_camera_to_effector_;
        Mat                 T_camera_to_base_;
        tf::StampedTransform transform_temp_; 
        Mat                 camera_velocity_;
        Mat                 camera_velocity_base_;


    public:
        Ros_ur_DVS();
        virtual bool get_visual_servoing_image(const robot_control_VS::PolarizedImagesConstPtr& gray_msg, cv::Mat& VS_image);
        void Callback(const robot_control_VS::PolarizedImagesConstPtr& gray_msg);    
        void get_parameters_resolution(int& resolution_x, int& resolution_y);
        Mat get_parameter_Matrix(string str, int row, int col);
        void rgb_image_operate(Mat& image_rgb, Mat& image_gray);
        void get_T(tf::StampedTransform  transform, Mat& T);
        void get_parameters_DVS(double& lambda, double& epsilon, Mat& image_gray_desired, Mat& image_depth_desired, Mat& camera_intrinsic, Mat& pose_desired);    
        void robot_move_to_target_joint_angle(std::vector<double> joint_group_positions_target);
        void get_camera_effector_pose(Mat& effector_to_base, Mat& camera_to_effector);
        void get_camera_pose(Mat& T_camera_to_base);
        void get_effector_twist(const Mat& camera_velocity, const Mat& effector_to_camera, Mat& effector_twist);
        void velocity_effector_to_base(const Mat& velocity, const Mat& effector_to_base, Mat& effector_twist_bast);
        void twist_publist(Mat camera_velocity);
        Mat velocity_camera_to_base(Mat velocity, Mat pose);
};

#endif


