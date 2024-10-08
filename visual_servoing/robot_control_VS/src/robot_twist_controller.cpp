#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/kinematics_metrics/kinematics_metrics.h>
#include <opencv2/imgproc.hpp>
#include <math.h>
#include <opencv2/opencv.hpp>
#include "geometry_msgs/Twist.h"

ros::Publisher twist_controller_pub;

void twist_CallBack(const geometry_msgs::Twist::ConstPtr& twist_msg)
{
    cv::Mat effector_velocity_base = (cv::Mat_<double>(1,6) << twist_msg->angular.x, twist_msg->angular.y, twist_msg->angular.z,
                                        twist_msg->linear.x, twist_msg->linear.y, twist_msg->linear.z);
    std::cout << effector_velocity_base << std::endl;

    twist_controller_pub.publish(twist_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv,"twist_cyh");
    ros::NodeHandle nd;
    
    ros::Subscriber sub = nd.subscribe("twist_cyh", 100, twist_CallBack);
    twist_controller_pub = nd.advertise<geometry_msgs::Twist>("/twist_controller/command", 50);	
    ros::spin();
    return 0;   
}
