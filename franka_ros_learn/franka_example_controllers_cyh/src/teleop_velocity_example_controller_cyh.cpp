#include <franka_example_controllers_cyh/teleop_velocity_example_controller_cyh.h>

#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <controller_interface/controller_base.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>


namespace franka_example_controllers_cyh
{
    bool TeleopVelocityExampleController_cyh::init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle)
    {

    }

    void TeleopVelocityExampleController_cyh::starting(const ros::Time&)
    {

    }

    void TeleopVelocityExampleController_cyh::update(const ros::Time&, const ros::Duration& period)
    {

    }

    void TeleopVelocityExampleController_cyh::stopping(const ros::Time&) 
    {

    }

}

PLUGINLIB_EXPORT_CLASS(franka_example_controllers_cyh::TeleopVelocityExampleController_cyh,
                       controller_interface::ControllerBase)