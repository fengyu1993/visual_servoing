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

using namespace cv;
using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

class Ros_VS
{
    protected:
        ros::NodeHandle     nh_; 
        message_filters::Subscriber<Image>      image_color_sub_;
        message_filters::Subscriber<Image>      image_depth_sub_;
        ros::Publisher      pub_; 
    public:
        int                 control_rate_;
        bool                flag_success;
    public:
        Ros_VS();
        virtual void Callback(const ImageConstPtr& image_color_msg, const ImageConstPtr& image_depth_msg){}; 
        void get_parameters(int& resolution_x, int& resolution_y, double& lambda, double& epsilon, Mat& image_gray_desired, Mat& image_depth_desired, Mat& image_gray_initial, Mat& camera_intrinsic, Mat& pose_desired);    
};

#endif