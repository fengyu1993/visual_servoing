#ifndef ROS_PVS
#define ROS_PVS

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
#include "polarimetric_visual_servoing.h"

using namespace cv;
using namespace std;
using namespace sensor_msgs;
using namespace message_filters;


class ControlSwitcher
{
    ros::NodeHandle nh_;
    map<string, string> controllers_;
    ros::ServiceClient switcher_srv_;
    
    public:
    ControlSwitcher()
    {
        map<string, string> controllers;
        controllers.insert(pair<string,string>("moveit","position_joint_trajectory_controller"));
        controllers.insert(pair<string,string>("velocity","cartesian_velocity_node_controller"));

        this->controllers_ = controllers;
        ros::service::waitForService("/controller_manager/switch_controller");
        this->switcher_srv_ = this->nh_.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");
    }

    bool switch_controllers(string start_controller_name, string stop_controller_name)
    {
        sleep(0.5);

        string start_controller = this->controllers_[start_controller_name];
        string stop_controller = this->controllers_[stop_controller_name];

        controller_manager_msgs::SwitchController controller_switch_msg;
        controller_switch_msg.request.strictness = 1;
        controller_switch_msg.request.start_controllers.push_back(start_controller);
        controller_switch_msg.request.stop_controllers.push_back(stop_controller);

        if (this->switcher_srv_.call(controller_switch_msg))
        {
            cout << "Successfully switched " << stop_controller_name << " to " << start_controller_name << endl;
            return true;
        }
        else
        {
            cout << "Failed switched " << stop_controller_name << " to " << start_controller_name << endl;
            return false;
        }
    }
};

class Ros_PVS 
{
    protected:
        ros::NodeHandle                         nh_; 
        message_filters::Subscriber<Image>      image_polar_sub_;
        message_filters::Subscriber<Image>      image_depth_sub_;
        TimeSynchronizer<Image, Image>          *sync_;
        tf::TransformListener                   listener_camera_pose_;
        moveit::planning_interface::MoveGroupInterface *move_group_interface_;
        Mat                                     camera_velocity_base_;
        ros::Publisher                          pub_camera_twist_; 

    public:
        Polarimetric_Visual_Servoing *PVS;
        double              control_rate_;
        bool                flag_success_;
        Mat                 joint_angle_initial_;
        bool                start_PVS;
        ControlSwitcher     control_switcher_;
        string name_link0_, name_camera_frame_;

    public:
        Ros_PVS();
        void Callback(const ImageConstPtr& image_color_msg, const ImageConstPtr& image_depth_msg);     
        void get_parameters_resolution(int& resolution_x, int& resolution_y);
        Mat get_parameter_Matrix(string str, int row, int col);
        void initialize_time_sync();
        void get_image_data_convert(const ImageConstPtr& image_polor_msg, const ImageConstPtr& image_depth_msg, Mat& polar_O_new, Mat& polar_A_new, Mat& polar_Phi_new, Mat& depth_img);
        void polar_image_operate(Mat& img_polar, Mat& polar_O_new, Mat& polar_A_new, Mat& polar_Phi_new);
        Mat depth_image_operate(Mat& image_depth);
        Mat get_camera_pose();
        Mat get_T(tf::StampedTransform  transform);
        Mat Quaternion2Matrix (Mat q);
        void get_parameters_PVS(double& lambda, double& epsilon, double& eta, double& phi_pol, double& k, Mat& polar_O_desired, Mat& polar_A_desired, Mat& polar_Phi_desired, Mat& image_depth_desired, Mat& camera_intrinsic, Mat& pose_desired);    
        Mat velocity_camera_to_base(Mat velocity, Mat pose);
        void robot_move_to_target_joint_angle(std::vector<double> joint_group_positions_target);
};

#endif