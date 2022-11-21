
#include "direct_visual_servoing.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include "ros_DVS.h"

int main(int argc, char** argv)
{
    // ×¼±¸
    ros::init(argc, argv, "DVS");    
    Ros_DVS DVS_control;
    Mat pose;

    // ¿ØÖÆ
    ros::Rate loop_rate(DVS_control.control_rate_);

    while (ros::ok())
    {
        try{
            if(DVS_control.flag_success)
            {
                ROS_INFO("visual servoing success");
                break;
            }
            else
            {
                ros::spinOnce();
                loop_rate.sleep();
            }
        }
        catch(...){
            return 1;
        }
    }

    return 0;
}

















