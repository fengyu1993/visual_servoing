# UR5 robot

## Overview

This ROS package is specifically designed for controlling the UR5 robot. It includes functionalities for operating the actual UR5 robot through a `ROS` node. The main features of this package are initiating a ROS node to manage the UR5 robot's movements and configurations, leveraging the capabilities of the `moveit` Python interface.

## Features

- **Launch File**: The `real_robot.launch` file is a modified version of an existing official launch file. It includes the real robot's IP address and the necessary configuration files to connect and control the UR5 robot.
- **Python Control Class**: This repository contains a Python class for simple robot control. This class utilizes `moveit`'s Python interface to handle basic robot motions and has been installed in ROS through `setup.py`, allowing for easy use across different packages.
- **Robot Movement Services**: `move_service.py` and `move_client.py` allow for precise control of the robot's movements through service calls, enabling easy integration with other ROS packages and systems.

## Installation

Clone this repository into your catkin workspace's src directory:

```bash
cd ~/catkin_ws/src
git clone https://github.com/OneOneLiu/ur5_robot.git
```

## Usage
To use this package, ensure your UR5 robot is properly connected, and the correct IP address and configuration files are set. Modify the [`real_robot.launch`](launch/real_robot.launch) file to update the IP address and point to your specific configuration file.

Once configured, launch the node with:
```bash{.line-numbers}
roslaunch ur5_robot real_robot.launch
```
