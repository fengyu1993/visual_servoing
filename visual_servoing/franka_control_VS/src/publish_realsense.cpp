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

using namespace std;
using namespace cv;
using namespace sensor_msgs;
using namespace message_filters;


rs2::frame depth_filter(rs2::frame& depth_img);
Mat hole_fill(Mat& img_depth);


int main(int argc, char** argv)
{
    ros::init(argc, argv, "publish_realsense");
    ros::NodeHandle nh;
    int resolution_x, resolution_y;
    nh.getParam("resolution_x", resolution_x);
    nh.getParam("resolution_y", resolution_y);

    image_transport::ImageTransport it(nh);  
    image_transport::Publisher image_color_raw_pub = it.advertise("/camera/color/image_raw", 1); 
    image_transport::Publisher image_depth_raw_pub = it.advertise("/camera/aligned_depth_to_color/image_raw", 1);
    image_transport::Publisher image_color_show_pub = it.advertise("/camera/color/image_show", 1);
    image_transport::Publisher image_depth_show_pub = it.advertise("/camera/aligned_depth_to_color/image_show", 1);

    ros::Rate loop_rate(30);

    rs2::pipeline pipe;
    rs2::config cfg;
    rs2::colorizer color_map;
    cfg.enable_stream(RS2_STREAM_COLOR, resolution_x, resolution_y);
    cfg.enable_stream(RS2_STREAM_DEPTH, resolution_x, resolution_y);
    pipe.start(cfg);
    rs2::align align_to_color(RS2_STREAM_COLOR);

    int w_depth, h_depth, w_rgb, h_rgb;

    while (ros::ok())
    {
        // read image
        rs2::frameset frameset = pipe.wait_for_frames();
        rs2::frameset frameset_depth = align_to_color.process(frameset);
        // With the aligned frameset we proceed as usual
        auto depth = frameset_depth.get_depth_frame();
        auto color = frameset_depth.get_color_frame();
        // filter
        depth = depth_filter(depth);
        // convert
        w_depth = depth.as<rs2::video_frame>().get_width();
        h_depth = depth.as<rs2::video_frame>().get_height();
        w_rgb = color.as<rs2::video_frame>().get_width();
        h_rgb = color.as<rs2::video_frame>().get_height();
        Mat img_rgb(cv::Size(w_rgb, h_rgb), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
        Mat depth_show(cv::Size(w_depth, h_depth), CV_8UC3, (void*)depth.apply_filter(color_map).get_data(), cv::Mat::AUTO_STEP);
        Mat img_depth(cv::Size(w_depth, h_depth), CV_16UC1, (void*)depth.get_data(), cv::Mat::AUTO_STEP);
        Mat rgb_show;
        cvtColor(img_rgb, rgb_show, COLOR_BGR2RGB);
        img_depth = hole_fill(img_depth);
        // publist
        sensor_msgs::ImagePtr  rgb_show_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgb_show).toImageMsg(); 
        sensor_msgs::ImagePtr  depth_show_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", depth_show).toImageMsg(); 
        sensor_msgs::ImagePtr  rgb_raw_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_rgb).toImageMsg(); 
        sensor_msgs::ImagePtr  depth_raw_msg = cv_bridge::CvImage(std_msgs::Header(), "mono16", img_depth).toImageMsg(); 
        image_color_show_pub.publish(rgb_show_msg);
        image_depth_show_pub.publish(depth_show_msg);
        image_color_raw_pub.publish(rgb_raw_msg);
        image_depth_raw_pub.publish(depth_raw_msg);

        loop_rate.sleep();
    }

    return 0;
}

rs2::frame depth_filter(rs2::frame& depth_img)
{
    // Declare filters
    rs2::decimation_filter dec_filter;  // Decimation - reduces depth frame density
    dec_filter.set_option(rs2_option::RS2_OPTION_FILTER_MAGNITUDE, 1);
    rs2::threshold_filter thr_filter;   // Threshold  - removes values outside recommended range
    rs2::spatial_filter spat_filter;    // Spatial    - edge-preserving spatial smoothing
    rs2::temporal_filter temp_filter;   // Temporal   - reduces temporal noise    
    rs2::hole_filling_filter hole_fill_filter;

    
    // Declare disparity transform from depth to disparity and vice versa
    const std::string disparity_filter_name = "Disparity";
    rs2::disparity_transform depth_to_disparity(true);
    rs2::disparity_transform disparity_to_depth(false);

    rs2::frame filtered = dec_filter.process(depth_img);
    filtered = thr_filter.process(filtered);
    filtered = depth_to_disparity.process(filtered);
    filtered = spat_filter.process(filtered);
    filtered = temp_filter.process(filtered);
    filtered = disparity_to_depth.process(filtered);
    filtered = hole_fill_filter.process(filtered);

    return filtered;
}

Mat hole_fill(Mat& img_depth)
{
    Mat img_fill;
    img_depth.copyTo(img_fill);
    unsigned short ave = mean(img_fill)[0];
    for(int i = 0; i < img_fill.rows; i++)
    {
        for(int j = 0; j < img_fill.cols; j++)
        {
            if(img_fill.at<unsigned short>(i,j) == 0)
            {
                img_fill.at<unsigned short>(i,j) = ave;
            }
        }
    }
    return img_fill;
}