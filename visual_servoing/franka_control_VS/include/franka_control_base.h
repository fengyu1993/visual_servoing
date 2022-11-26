#pragma once

#include <franka/robot.h>
#include "examples_common.h"
#include <opencv2/opencv.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <map>
#include <string>
#include <iostream>
#include "ros/ros.h"

using namespace cv;
using namespace std;

void franka_move_to_target_joint_angle(moveit::planning_interface::MoveGroupInterface& move_group_interface, std::vector<double> joint_group_positions_target );


