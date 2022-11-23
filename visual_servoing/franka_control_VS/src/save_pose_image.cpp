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

using namespace std;
using namespace cv;
using namespace sensor_msgs;
using namespace message_filters;

void callback(const ImageConstPtr& image_color_msg, const ImageConstPtr& image_depth_msg)
{
    ROS_INFO("CYH");
    Mat color_img, depth_img;
    // rgb转灰度 [0,255]->[1,0]
    cv_bridge::CvImagePtr cv_ptr_color = cv_bridge::toCvCopy(image_color_msg, sensor_msgs::image_encodings::BGR8);
    color_img = cv_ptr_color->image;
    // 深度图
    cv_bridge::CvImagePtr cv_ptr_depth = cv_bridge::toCvCopy(image_depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
    depth_img = cv_ptr_depth->image;
    // 显示
    // cout << "color_img = " << color_img.colRange(0,5).rowRange(0,5) << endl;
    // cout << "depth_img = " << depth_img.colRange(0,5).rowRange(0,5) << endl;
    imshow("Color", color_img);
    imshow("Depth", depth_img);
    // 保存图像和位姿
    if((char)waitKey(10) == 32) // 32: space
    {
        ROS_INFO("Save depth and color image.");
        // 保存深度图
        imwrite("/home/cyh/Work/visual_servoing_ws/src/visual_servoing/franka_control_VS/param/image_depth_desired.png", depth_img);
        // 保存彩色图
        imwrite("/home/cyh/Work/visual_servoing_ws/src/visual_servoing/franka_control_VS/param/image_rgb_desired.png", color_img);
        // 保存相机位姿
        
    }

    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "save_pose_image");
 
    ros::NodeHandle nh;
 
    cout<< "Press space to save rgb_raw and depth_raw to a file."<<endl;
 
    message_filters::Subscriber<Image> image_color_sub(nh,"/camera/color/image_raw", 1);
    message_filters::Subscriber<Image> image_depth_sub(nh,"/camera/aligned_depth_to_color/image_raw", 1);
 
    TimeSynchronizer<Image, Image> sync(image_color_sub, image_depth_sub, 10);
    sync.registerCallback(boost::bind(&callback, _1, _2));
 
    ros::Rate rate(20.0);
 
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
 
 
    return 0;
}