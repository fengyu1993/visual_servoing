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
Mat Theta;

void get_Param(ros::NodeHandle nh)
{
    nh.getParam("resolution_x", resolution_x);
    nh.getParam("resolution_y", resolution_y);
}

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

void Callback(const ImageConstPtr& image_polar_msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  std::vector<cv::Mat> channels;
  Mat polar;
  Mat O_image;
  int row = resolution_y / 2;
  int col = resolution_x / 2;

  try
  {
    // Get O Dolp Aolp
    cv_ptr = cv_bridge::toCvCopy(image_polar_msg, sensor_msgs::image_encodings::BGR8);
    cv_ptr->image.convertTo(polar, CV_64F);
    cv::split(polar, channels);
    Mat O = channels[0];  
    O.convertTo(O_image, CV_8UC1);
    Mat Dolp = channels[1] / 255.0;
    Mat Aolp = channels[2] * (CV_PI / 180.0) - CV_PI;
    cv::imshow("O", O_image);
    cv::imshow("Aolp", Aolp);
    cv::imshow("Dolp", Dolp);
    cv::waitKey(10); 


    // Get theta
    double eta = 1.5;
    double theta = 0.3;
    double pho = Dolp.at<double>(row, col);
    for (int i=0; i<20; i++)
    {
      double pho_0 = (2 * pow(sin(theta),2) * cos(theta) * sqrt(pow(eta,2) - pow(sin(theta),2))) / (pow(eta,2) - pow(sin(theta),2) - pow(eta,2)*pow(sin(theta),2) + 2*pow(sin(theta),4));
      double d_pho_0 = (2*sin(theta)*(pow(eta,2) - pow(sin(theta),2) - pow(eta,2)*pow(sin(theta),2)) * (2*pow(eta,2) - pow(sin(theta),2) - pow(eta,2)*pow(sin(theta),2))) / 
                          (sqrt(pow(eta,2) - pow(sin(theta),2)) * pow(pow(eta,2) - pow(sin(theta),2) - pow(eta,2)*pow(sin(theta),2) + 2*pow(sin(theta),4),2));
      theta = (pho - pho_0) / d_pho_0 + theta;
    }
    // Get normal vector
    double phi = Aolp.at<double>(row, col);
    Mat normal_vec = (cv::Mat_<double>(3,1) << cos(phi)*sin(theta), sin(phi)*sin(theta), -cos(theta));
    cout << "normal_vec = " << normal_vec << endl;
    cout << "theta = " << theta << ", phi = " << phi << endl;
    

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
    Theta = Mat::zeros(resolution_y, resolution_x, CV_64F);

    cv::namedWindow("O"); 
    cv::namedWindow("Aolp"); 
    cv::namedWindow("Dolp"); 
    cv::startWindowThread(); // 启动窗口线程

    ros::Subscriber sub = nh.subscribe("/arena_camera_node/image_raw", 10, Callback);

	  ros::spin();
    cv::destroyAllWindows();
    return 0;
}




















