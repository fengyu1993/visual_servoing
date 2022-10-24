#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "track_marker_cyh");

    ros::NodeHandle node_handle;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "panda_arm";
    
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // node_handle.getParam("target_pose", joint_names)
    return 0;   
}


    // ROS_INFO_STREAM( s.str() << ": pose changed"
    //             << "\nposition = "
    //             << target_pose->position.x
    //             << ", " << target_pose->position.y
    //             << ", " << target_pose->position.z
    //             << "\norientation = "
    //             << target_pose->orientation.w
    //             << ", " << target_pose->orientation.x
    //             << ", " << target_pose->orientation.y
    //             << ", " << target_pose->orientation.z);  