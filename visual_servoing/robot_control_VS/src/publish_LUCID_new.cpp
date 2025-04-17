#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Twist.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <cv_bridge/cv_bridge.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <fstream>
#include <string>
#include <ctime> 
#include <chrono>
#include <tf/transform_listener.h>
#include <librealsense2/rs.hpp>
#include <librealsense2/h/rs_option.h>
#include <image_transport/image_transport.h> 
#include "ArenaApi.h"
#include "SaveApi.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <thread>
#include "key.h"

using namespace std;
using namespace cv;
using namespace sensor_msgs;
using namespace message_filters;

int resolution_x, resolution_y;
image_transport::Publisher image_polar_pub;
image_transport::Publisher image_polar_offset_pub;

void get_Param(ros::NodeHandle nh)
{
    nh.getParam("resolution_x", resolution_x);
    nh.getParam("resolution_y", resolution_y);
}

void Callback(const ImageConstPtr& image_polar_msg)
{
  try
  {
    cv::imshow("view", cv_bridge::toCvShare(image_polar_msg, "bgr8")->image);
    cv::waitKey(30); 
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", image_polar_msg->encoding.c_str());
  }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "publish_L515_LUCID");
    ros::NodeHandle nh; 

    image_transport::ImageTransport it(nh);  
    image_polar_pub = it.advertise("/VS/polarized_image", 1); 
	  image_polar_offset_pub = it.advertise("/VS/polarized_offset_image", 1); 

	  get_Param(nh);

    cv::namedWindow("view"); // 创建一个窗口用于显示图像
    cv::startWindowThread(); // 启动窗口线程

    ros::Subscriber sub = nh.subscribe("/arena_camera_node/image_raw", 10, Callback);

	ros::spin();
    cv::destroyWindow("view");
    return 0;
}




















