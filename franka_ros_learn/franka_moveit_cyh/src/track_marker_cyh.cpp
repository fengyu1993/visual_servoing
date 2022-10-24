#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// moveit::planning_interface::MoveGroupInterface move_group_interface("panda_arm");
// moveit::planning_interface::MoveGroupInterface::Plan my_plan;

void track_marker_Callback(const geometry_msgs::Pose::ConstPtr& target_pose)
{
    // move_group_interface.setPoseTarget(*target_pose); 
    
    // move_group_interface.plan(my_plan);

    // move_group_interface.move();
    std::ostringstream s;
    
    ROS_INFO_STREAM( s.str() << ": pose changed"
                << "\nposition = "
                << target_pose->position.x
                << ", " << target_pose->position.y
                << ", " << target_pose->position.z
                << "\norientation = "
                << target_pose->orientation.w
                << ", " << target_pose->orientation.x
                << ", " << target_pose->orientation.y
                << ", " << target_pose->orientation.z);  
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "track_marker_cyh");
    
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("target_pose", 100, track_marker_Callback);

    ros::spin();

    return 0;   
}