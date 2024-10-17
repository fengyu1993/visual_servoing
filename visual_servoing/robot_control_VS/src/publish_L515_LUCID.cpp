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

using namespace std;
using namespace cv;
using namespace sensor_msgs;
using namespace message_filters;

int resolution_x, resolution_y;
image_transport::Publisher image_polar_pub;
image_transport::Publisher image_gray_pub;
image_transport::Publisher image_depth_pub;

Mat get_polar_offest(const ImageConstPtr& image_polar_msg)
{
	// ros->opencv
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(image_polar_msg, sensor_msgs::image_encodings::BGR8);
	Mat polar = cv_ptr->image;
	// get channel[0]: polar offset
	std::vector<cv::Mat> channels;
    cv::split(polar, channels);
	Mat polar_offset = channels[0];
	return polar_offset;
}

Mat get_gray(const ImageConstPtr& image_color_msg)
{
	// ros->opencv
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(image_color_msg, sensor_msgs::image_encodings::BGR8);	
	Mat gray_image;
    cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);
	return gray_image;
}

void Callback(const ImageConstPtr& image_polar_msg, const ImageConstPtr& image_depth_msg, const ImageConstPtr& image_color_msg)
{
    // ROS_INFO_STREAM("Received synchronized messages: "
    //                 << "image timestamp: " << image_polar_msg->header.stamp
    //                 << ", depth timestamp: " << image_depth_msg->header.stamp
    //                 << ", rgb timestamp: " << image_color_msg->header.stamp);

	Mat polar_offset = get_polar_offest(image_polar_msg);
	Mat gray_image = get_gray(image_color_msg);

	sensor_msgs::ImagePtr  image_polar_offset_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", polar_offset).toImageMsg(); 
	sensor_msgs::ImagePtr  image_gray_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", gray_image).toImageMsg(); 

	image_polar_pub.publish(image_polar_offset_msg);
	image_depth_pub.publish(image_depth_msg);
	image_gray_pub.publish(image_gray_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "publish_L515_LUCID");
    ros::NodeHandle nh; 

    image_transport::ImageTransport it(nh);  
    image_polar_pub = it.advertise("/VS/polarized_image", 1); 
    image_gray_pub = it.advertise("/VS/color_image", 1);
	image_depth_pub = it.advertise("/VS/depth_image", 1);

    nh.getParam("resolution_x", resolution_x);
    nh.getParam("resolution_y", resolution_y);

	message_filters::Subscriber<Image>   image_polar_sub(nh,"/LUCID/polarized_image_raw", 1);
	message_filters::Subscriber<Image>   image_depth_sub(nh,"/L515/depth_image_raw", 1);
	message_filters::Subscriber<Image>   image_color_sub(nh,"/L515/color_image_raw", 1);

	typedef message_filters::sync_policies::ApproximateTime<Image, Image, Image> MySyncPolicy;
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_polar_sub, image_depth_sub, image_color_sub);
    sync.registerCallback(boost::bind(&Callback, _1, _2, _3));

	ros::spin();
    return 0;
}