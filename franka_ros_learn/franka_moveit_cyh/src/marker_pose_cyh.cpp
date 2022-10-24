#include <iostream>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <math.h>
#include <string>
#include <Eigen/Dense>

using namespace visualization_msgs;
using namespace Eigen;

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;
ros::Publisher Pose_pub;

Marker makeBox(InteractiveMarker &msg)
{
    Marker marker;

    // marker.type = Marker::CUBE;
    marker.type = Marker::SPHERE;
    marker.scale.x = msg.scale ;
    marker.scale.y = msg.scale ;
    marker.scale.z = msg.scale ;

    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 0.8;

    return marker;
}

InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg)
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( makeBox(msg) );
  msg.controls.push_back( control );

  return msg.controls.back();
}

void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{

    std::ostringstream s;
    s << "Feedback from marker '" << feedback->marker_name << "' "
        << " / control '" << feedback->control_name << "'";
    
    
    if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE)
    { 
        Pose_pub.publish(feedback->pose);   
    }
    server->applyChanges();
}

void make6DofMarker(bool fixed, unsigned int interaction_mode, geometry_msgs::Pose pose_marker, std::string name, bool show_6dof)
{
    InteractiveMarker int_marker;  tf::Vector3 position;
    int_marker.header.frame_id = "panda_link0";

    int_marker.pose = pose_marker;
    int_marker.scale = 0.15;

    int_marker.name = name.c_str();
    int_marker.description = "Simple 6-DOF Control";

    makeBoxControl(int_marker);
    int_marker.controls[0].interaction_mode = interaction_mode;

    InteractiveMarkerControl control;

    if( fixed )
    {
        int_marker.name += "_fixed";
        int_marker.description += "\n(fixed orientation)";
        control.orientation_mode = InteractiveMarkerControl::FIXED;
    }

    if(interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
    {
        std::string mode_text;
        if(interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_3D)           mode_text = "MOVE_3D";
        if( interaction_mode == visualization_msgs::InteractiveMarkerControl::ROTATE_3D )       mode_text = "ROTATE_3D";
        if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D )  mode_text = "MOVE_ROTATE_3D";
        int_marker.name += "_" + mode_text;
        int_marker.description = std::string("3D Control") + (show_6dof ? " + 6-DOF controls" : "") + "\n" + mode_text;
    }

    if(show_6dof)
    {
        tf::Quaternion orien(1.0, 0.0, 0.0, 1.0);
        orien.normalize();
        tf::quaternionTFToMsg(orien, control.orientation);
        control.name = "rotate_x";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_x";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        orien = tf::Quaternion(0.0, 1.0, 0.0, 1.0);
        orien.normalize();
        tf::quaternionTFToMsg(orien, control.orientation);
        control.name = "rotate_z";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_z";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        orien = tf::Quaternion(0.0, 0.0, 1.0, 1.0);
        orien.normalize();
        tf::quaternionTFToMsg(orien, control.orientation);
        control.name = "rotate_y";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_y";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);
    }

    server->insert(int_marker);
    server->setCallback(int_marker.name, &processFeedback);
    if(interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
        menu_handler.apply(*server, int_marker.name);
}

int main(int argc, char** argv)
{    
    ros::init(argc, argv, "marker_pose_cyh");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    server.reset( new interactive_markers::InteractiveMarkerServer("basic_controls", "", false));

    ros::Duration(0.1).sleep();

    static const std::string PLANNING_GROUP = "panda_arm";
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
    geometry_msgs::Pose current_pose = move_group_interface.getCurrentPose().pose;

    std::ostringstream s;
    ROS_INFO_STREAM( s.str() << "position = "
                << current_pose.position.x
                << ", " << current_pose.position.y
                << ", " << current_pose.position.z
                << "\norientation = "
                << current_pose.orientation.w
                << ", " << current_pose.orientation.x
                << ", " << current_pose.orientation.y
                << ", " << current_pose.orientation.z);

    make6DofMarker(false, visualization_msgs::InteractiveMarkerControl::NONE, current_pose, "marker_cyh", true);
 
    server->applyChanges();

    Pose_pub = nh.advertise<geometry_msgs::Pose>("target_pose", 100);

    ros::waitForShutdown();

    server.reset();
}