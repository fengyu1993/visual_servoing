#pragma once

#include <franka/robot.h>
#include "examples_common.h"
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

void franka_move_to_target_joint_angle(Mat joint_angle);