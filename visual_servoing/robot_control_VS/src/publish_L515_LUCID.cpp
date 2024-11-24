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
int polar_row_init, polar_row_end, polar_col_init, polar_col_end;
int gray_row_init, gray_row_end, gray_col_init, gray_col_end;
image_transport::Publisher image_polar_pub;
image_transport::Publisher image_polar_offset_pub;
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

Mat get_depth(const ImageConstPtr& image_depth_msg)
{
	// ros->opencv
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(image_depth_msg, sensor_msgs::image_encodings::MONO16);	
	return cv_ptr->image;	
}

Mat get_polar(const ImageConstPtr& image_polar_msg)
{
	// ros->opencv
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(image_polar_msg, sensor_msgs::image_encodings::BGR8);	
	return cv_ptr->image;
}

void save_image(Mat polar_offset, Mat gray_image)
{

	// cout << "save image_horizon success" << endl;
	// imwrite("/home/cyh/Work/visual_servoing_ws/src/visual_servoing/robot_control_VS/param/polar_img_horizon.png", polar_offset);
    // imwrite("/home/cyh/Work/visual_servoing_ws/src/visual_servoing/robot_control_VS/param/gray_img_horizon.png", gray_image);
 	cout << "save image_vertical success" << endl;
	imwrite("/home/cyh/Work/visual_servoing_ws/src/visual_servoing/robot_control_VS/param/polar_img_vertical.png", polar_offset);
    imwrite("/home/cyh/Work/visual_servoing_ws/src/visual_servoing/robot_control_VS/param/gray_img_vertical.png", gray_image);

}

void Callback(const ImageConstPtr& image_polar_msg, const ImageConstPtr& image_depth_msg, const ImageConstPtr& image_color_msg)
{
	Mat polar_offset_image = get_polar_offest(image_polar_msg);
	Mat gray_image = get_gray(image_color_msg);
	Mat depth_image = get_depth(image_depth_msg);
	Mat polar_image = get_polar(image_polar_msg);

	// save_image(polar_offset_image, gray_image);

	Mat gray_image_resize_temp = gray_image.rowRange(gray_row_init, gray_image.rows-gray_row_end).colRange(gray_col_init, gray_image.cols - gray_col_end);
 	Mat depth_image_resize_temp = depth_image.rowRange(gray_row_init, depth_image.rows-gray_row_end).colRange(gray_col_init, depth_image.cols - gray_col_end); 
	Mat polar_offset_image_resize_temp = polar_offset_image.rowRange(polar_row_init, polar_offset_image.rows - polar_row_end).colRange(polar_col_init, polar_offset_image.cols - polar_col_end);
	Mat polar_image_resize_temp = polar_image.rowRange(polar_row_init, polar_image.rows - polar_row_end).colRange(polar_col_init, polar_image.cols - polar_col_end);
   	
	cv::Size targetSize(resolution_x, resolution_y); 
    cv::Mat gray_image_resize, depth_image_resize, polar_offset_image_resize, polar_image_resize;
    cv::resize(gray_image_resize_temp, gray_image_resize, targetSize, 0, 0, cv::INTER_LINEAR);
	cv::resize(depth_image_resize_temp, depth_image_resize, targetSize, 0, 0, cv::INTER_LINEAR);
	cv::resize(polar_image_resize_temp, polar_image_resize, targetSize, 0, 0, cv::INTER_LINEAR);
	cv::resize(polar_offset_image_resize_temp, polar_offset_image_resize, targetSize, 0, 0, cv::INTER_LINEAR);

	sensor_msgs::ImagePtr  image_polar_offset_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", polar_offset_image_resize).toImageMsg(); 
	sensor_msgs::ImagePtr  image_gray_resize_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", gray_image_resize).toImageMsg(); 
	sensor_msgs::ImagePtr  image_depth_resize_msg = cv_bridge::CvImage(std_msgs::Header(), "mono16", depth_image_resize).toImageMsg(); 
	sensor_msgs::ImagePtr  image_polar_resize_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", polar_image_resize).toImageMsg();
	
	image_polar_pub.publish(image_polar_resize_msg);
	image_polar_offset_pub.publish(image_polar_offset_msg);
	image_depth_pub.publish(image_depth_resize_msg);
	image_gray_pub.publish(image_gray_resize_msg);
}

void get_Param(ros::NodeHandle nh)
{
    nh.getParam("resolution_x", resolution_x);
    nh.getParam("resolution_y", resolution_y);

	nh.getParam("polar_row_init", polar_row_init);
    nh.getParam("polar_row_end", polar_row_end);
	nh.getParam("polar_col_init", polar_col_init);
    nh.getParam("polar_col_end", polar_col_end);

	nh.getParam("gray_row_init", gray_row_init);
    nh.getParam("gray_row_end", gray_row_end);
	nh.getParam("gray_col_init", gray_col_init);
    nh.getParam("gray_col_end", gray_col_end);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "publish_L515_LUCID");
    ros::NodeHandle nh; 

    image_transport::ImageTransport it(nh);  
    image_polar_pub = it.advertise("/VS/polarized_image", 1); 
	image_polar_offset_pub = it.advertise("/VS/polarized_offset_image", 1); 
    image_gray_pub = it.advertise("/VS/color_image", 1);
	image_depth_pub = it.advertise("/VS/depth_image", 1);

	get_Param(nh);

	message_filters::Subscriber<Image>   image_polar_sub(nh,"/LUCID/polarized_image_raw", 1);
	message_filters::Subscriber<Image>   image_depth_sub(nh,"/L515/depth_image_raw", 1);
	message_filters::Subscriber<Image>   image_color_sub(nh,"/L515/color_image_raw", 1);

	typedef message_filters::sync_policies::ApproximateTime<Image, Image, Image> MySyncPolicy;
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_polar_sub, image_depth_sub, image_color_sub);
    sync.registerCallback(boost::bind(&Callback, _1, _2, _3));

	
	ros::spin();

	
    return 0;
}