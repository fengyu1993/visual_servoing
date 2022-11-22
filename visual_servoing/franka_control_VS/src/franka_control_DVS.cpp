
#include "direct_visual_servoing.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include "ros_DVS.h"


Mat get_parameter_Matrix(string str, int row, int col)
{
    ros::NodeHandle nh;
    Mat Matrix;
    XmlRpc::XmlRpcValue param_yaml;
    nh.getParam(str, param_yaml);
    double data[param_yaml.size()];
    for(int i=0; i<param_yaml.size(); i++) 
    {
        data[i] = param_yaml[i];
    }
    Mat Matrix_temp = Mat(row, col, CV_64FC1, data);
    Matrix_temp.copyTo(Matrix);   
    return Matrix;
}

int main(int argc, char** argv)
{
    // ׼��
    ros::init(argc, argv, "DVS");  
    Ros_DVS DVS_control;
    // ��е���ƶ�����ʼλ��
    Mat pose_initial = DVS_control.DVS->get_pose_initial_parameter();
    
    // ����
    ros::Rate loop_rate(DVS_control.control_rate_);

    while (ros::ok())
    {
        try{
            if(DVS_control.flag_success){
                ROS_INFO("visual servoing success");
                break;
            }else{
                ros::spinOnce();
                loop_rate.sleep();
            }
        }catch(...){
            return 1;
        }
    }

    return 0;
}

















