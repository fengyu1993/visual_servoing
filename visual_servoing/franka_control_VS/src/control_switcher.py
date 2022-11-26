#!/usr/bin/python3

# Software License Agreement (BSD License)
#
# Copyright (c) 2021, OMRON SINIC X
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of OMRON SINIC X nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Felix von Drigalski

import sys
import rospy
import actionlib
import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list
import controller_manager_msgs.srv as cm_srv
from franka_msgs.msg import ErrorRecoveryActionGoal
from franka_msgs.msg import FrankaState, Errors as FrankaErrors
import franka_gripper.msg
import tf
import numpy as np

import geometry_msgs.msg
from franka_msgs.srv import SetJointImpedance
from franka_msgs.srv import SetCartesianImpedance
from franka_msgs.srv import SetLoad
from franka_msgs.srv import SetFullCollisionBehavior


class ControlSwitcher:
    # Class to switch between controllers in ROS
    def __init__(self, controllers, controller_manager_node='/controller_manager'):
        self.controllers = controllers
        rospy.wait_for_service(controller_manager_node + "/switch_controller")
        rospy.wait_for_service(controller_manager_node + "/list_controllers")

        self.switcher_srv = rospy.ServiceProxy(controller_manager_node + "/switch_controller", cm_srv.SwitchController)
        self.lister_srv = rospy.ServiceProxy(controller_manager_node + "/list_controller", cm_srv.ListControllers)

    def switch_controllers(self, start_controller_names):
        rospy.sleep(0.5)
        # Get list of controller full names to start and stop
        start_controllers = [self.controllers[start_controller] for start_controller in start_controller_names]
        stop_controllers = [self.controllers[n] for n in self.controllers if n not in start_controller_names]

        controller_switch_msg = cm_srv.SwitchControllerRequest()
        controller_switch_msg.strictness = 1
        controller_switch_msg.start_controllers = start_controllers
        controller_switch_msg.stop_controllers = stop_controllers

        result = self.switcher_srv(controller_switch_msg).ok
        if result:
            rospy.logdebug('Successfully switched to controllers {} ({})'.format(start_controllers, start_controller_names))
            return result
        else:
            rospy.logdebug("Failed switching controllers")
            return False

    def stop_controllers(self):
        self.switch_controllers([])


rospy.init_node("control_switcher")

cs = ControlSwitcher({'moveit':   'position_joint_trajectory_controller',
                      'velocity': 'cartesian_velocity_node_controller'})

value_old = rospy.get_param('controller')

while not rospy.is_shutdown():
    value_new = rospy.get_param('controller')
    if value_new != value_old:
        if value_new == "moveit":
            cs.switch_controllers(['moveit'])
        elif value_new == "velocity":
            cs.switch_controllers(['velocity'])
        else:
            cs.stop_controllers()
    value_old = value_new

