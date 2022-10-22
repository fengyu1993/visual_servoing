#include <iostream>
#include <ros/ros.h>
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

Marker makeBox(InteractiveMarker &msg)
{
    Marker marker;

    marker.type = Marker::CUBE;
    marker.scale.x = msg.scale * 0.5;
    marker.scale.y = msg.scale * 0.5;
    marker.scale.z = msg.scale * 0.5;

    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 1.0;

    return marker;
}

InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg )
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( makeBox(msg) );
  msg.controls.push_back( control );

  return msg.controls.back();
}

void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  server->applyChanges();
}

void make6DofMarker(bool fixed, unsigned int interaction_mode, const Matrix4d T, std::string name, bool show_6dof)
{
    Matrix3d R; Vector3d p;

    R << T.block<3,3>(0,0);
    p << T.block<3,1>(0,3);

    Eigen::Quaterniond quater =  Eigen::Quaterniond(R);

    InteractiveMarker int_marker;  tf::Vector3 position;
    int_marker.header.frame_id = "panda_link0";
    position = tf::Vector3(p(0), p(1), p(2));

    int_marker.pose.position.x = p(0); int_marker.pose.position.y = p(1); int_marker.pose.position.z = p(2);
    int_marker.pose.orientation.x = quater.x();
    int_marker.pose.orientation.y = quater.y();
    int_marker.pose.orientation.z = quater.z();
    int_marker.pose.orientation.w = quater.w();
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
    ros::NodeHandle n;

    server.reset( new interactive_markers::InteractiveMarkerServer("basic_controls", "", false));

    ros::Duration(0.1).sleep();

    Matrix4d Pose = Matrix4d::Ones();
    Pose(0,1) = 1.0;
    Pose(0,2) = 1.0;
    Pose(0,3) = 1.0;


    make6DofMarker(false, visualization_msgs::InteractiveMarkerControl::NONE, Pose, "marker_cyh", true);
 
    server->applyChanges();

    ros::spin();

    server.reset();
}