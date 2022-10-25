#include <iostream>
#include <vector>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <cmath>

using namespace std;

bool get_target_pose(geometry_msgs::Pose &Pose)
{
    ros::NodeHandle node_handle;
    vector<double> target_position;
    vector<double> target_orientation;
    std::ostringstream s;

    if(!(node_handle.getParam("target_pose/position", target_position)))
    {
        ROS_ERROR("target position get fail");
        return false;
    }
    else
    {
        Pose.position.x = target_position[0];
        Pose.position.y = target_position[1];
        Pose.position.z = target_position[2];
        // ROS_INFO_STREAM( s.str() << "\nposition_cyh = "
        //         << Pose.position.x 
        //         << ", " << Pose.position.y
        //         << ", " << Pose.position.z);
    }


    if(!(node_handle.getParam("target_pose/orientation", target_orientation)))
    {
        ROS_ERROR("target orientation get fail");
        return false;
    }
    else
    {
        Pose.orientation.x = target_orientation[0];
        Pose.orientation.y = target_orientation[1];
        Pose.orientation.z = target_orientation[2];
        Pose.orientation.w = target_orientation[3];
        // ROS_INFO_STREAM( s.str() << "\norientation_cyh = "
        //         << Pose.orientation.x
        //         << ", " << Pose.orientation.y
        //         << ", " << Pose.orientation.z
        //         << ", " << Pose.orientation.w);
    }

    return true;
}

double get_pose_error(geometry_msgs::Pose target_pose, geometry_msgs::Pose current_pose)
{
    double error;
    error = pow(target_pose.position.x - current_pose.position.x, 2)
                + pow(target_pose.position.y - current_pose.position.y, 2)
                + pow(target_pose.position.z - current_pose.position.z, 2)
                + pow(target_pose.orientation.x - current_pose.orientation.x, 2)
                + pow(target_pose.orientation.y - current_pose.orientation.y, 2)
                + pow(target_pose.orientation.z - current_pose.orientation.z, 2)
                + pow(target_pose.orientation.w - current_pose.orientation.w, 2);
    return error;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "track_marker_cyh");

    ros::NodeHandle node_handle;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "panda_arm";
    
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    
    geometry_msgs::Pose target_pose;
    geometry_msgs::Pose current_pose;

    ros::Duration(5).sleep();

    while (ros::ok())
    {
        get_target_pose(target_pose);
        current_pose = move_group_interface.getCurrentPose().pose;
        double error = get_pose_error(target_pose, current_pose);

        if (error > 5e-4)
        {
            move_group_interface.setPoseTarget(target_pose);

            move_group_interface.plan(my_plan);

            move_group_interface.move();
        }
    }

    return 0;   
}

