#ifndef ROS_VS
#define ROS_VS

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

class Ros_VS 
{
    protected:
        ros::NodeHandle                         nh_; 
        message_filters::Subscriber<Image>      image_color_sub_;
        message_filters::Subscriber<Image>      image_depth_sub_;
        TimeSynchronizer<Image, Image>          *sync_;
        tf::TransformListener                   listener_camera_pose_;
        moveit::planning_interface::MoveGroupInterface *move_group_interface_;
        Mat                                     camera_velocity_base_;
        ros::Publisher                          pub_camera_twist_; 

    public:
        double              control_rate_;
        bool                flag_success_;
        Mat                 joint_angle_initial_;
        bool                start_VS;
        ControlSwitcher     control_switcher_;

    public:
        Ros_VS();
        void initialize_time_sync();
        virtual void Callback(const ImageConstPtr& image_color_msg, const ImageConstPtr& image_depth_msg) = 0;  
        void get_parameters_resolution(int& resolution_x, int& resolution_y);
        void get_parameters_VS(double& lambda, double& epsilon, Mat& image_gray_desired, Mat& image_depth_desired, Mat& camera_intrinsic, Mat& pose_desired);    
        void get_parameters_DOM(int& order_min, int& order_max, double& delta_epsilon, double& lambda_order);    
        void set_resolution_parameters(int resolution_x, int resolution_y);
        void get_image_data_convert(const ImageConstPtr& image_color_msg, const ImageConstPtr& image_depth_msg, Mat& color_img, Mat& depth_img);
        Mat get_camera_pose();
        Mat velocity_camera_to_base(Mat velocity, Mat pose);
        Mat get_parameter_Matrix(string str, int row, int col);
        Mat rgb_image_operate(Mat& image_rgb);
        Mat depth_image_operate(Mat& image_depth);
        Mat get_T(tf::StampedTransform  transform);
        Mat Quaternion2Matrix (Mat q);
        void franka_move_to_target_joint_angle(std::vector<double> joint_group_positions_target);
};

#endif