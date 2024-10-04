#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/kinematics_metrics/kinematics_metrics.h>


// void twist_CallBack(const std_msgs::String::ConstPtr& msg)
// {
//     ROS_INFO("info: [%s]", msg->data.c_str());
// }

int main(int argc, char** argv)
{
    // ros::init(argc, argv,"twist_controller");
    // ros::NodeHandle nd;
    
    // ros::Subscriber sub = nd.subscribe("twist", 100, twist_CallBack);
    // ros::spin();
    // return 0;
}
