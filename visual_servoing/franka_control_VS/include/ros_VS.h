#ifndef ROS_VS
#define ROS_VS

#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>

using namespace cv;
using namespace std;

class Ros_VS
{
    protected:
        ros::NodeHandle     nh_; 
        ros::Subscriber     sub_;
        ros::Publisher      pub_; 
    public:
        int                 control_rate_;
        bool                flag_success;
    public:
        Ros_VS();
        virtual void Callback(const sensor_msgs::ImageConstPtr &msg) = 0; 
        void get_parameters(int& resolution_x, int& resolution_y, double& lambda, double& epsilon, Mat& image_gray_desired, Mat& image_depth_desired, Mat& image_gray_initial, Mat& camera_intrinsic, Mat& pose_desired);    
};

#endif