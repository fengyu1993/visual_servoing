#include <franka_example_controllers_cyh/teleop_example_controller_cyh.h>

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
    bool TeleopExampleController_cyh::init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle)
    {
        position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();
        if (position_joint_interface_ == nullptr)
        {
            ROS_ERROR("TeleopExampleController_cyh: Error getting position joint interface from hardware!");
            return false;
        }
        std::vector<std::string> joint_names;
        if (!node_handle.getParam("joint_names", joint_names))
        {
            ROS_ERROR("TeleopExampleController_cyh: Could not parse joint names");
        }
        if (joint_names.size() != 7)
        {
            ROS_ERROR_STREAM("TeleopExampleController_cyh: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
            return false;
        }
        position_joint_handles_.resize(7);
        for (size_t i = 0; i < 7; ++i)
        {
            try{
                position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
            }
            catch (const hardware_interface::HardwareInterfaceException& e)
            {
                ROS_ERROR_STREAM(
                    "TeleopExampleController_cyh: Exception getting joint handles: " << e.what());
                return false;
            }
        }
        
        return true;
    }

    void TeleopExampleController_cyh::starting(const ros::Time&)
    {
        q_home_ = {0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4};
        for(size_t i = 0; i < 7; ++i)
        {
            q_current_[i] = position_joint_handles_[i].getPosition();
        }
        elapsed_time_ = ros::Duration(0.0);
    }

    void TeleopExampleController_cyh::update(const ros::Time&, const ros::Duration& period)
    {
        elapsed_time_ += period;
        std::array<double,7> q_error = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        std::array<double,7> dq = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        std::array<double,7> delta_angle = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        double lambda = 0.2;

        for(size_t i = 0; i < 7; ++i)
        {
            q_current_[i] = position_joint_handles_[i].getPosition();
        }

        for(size_t i = 0; i < 7; i++)
        {
            q_error[i] = q_current_[i] - q_home_[i];
            if(q_error[i] > 0.01)
            {
                dq[i] = -lambda * q_error[i];
                delta_angle[i] = dq[i] * 0.001;
                ROS_INFO("Return to start: q_error[%ld] = %0.2f\n", i, q_error[i]);
                position_joint_handles_[i].setCommand(q_current_[i] + delta_angle[i]);
            }
            else
            {
                ROS_INFO("Return to start: Done");
            }
        }       
    }

    void TeleopExampleController_cyh::stopping(const ros::Time&) 
    {

    }

}

PLUGINLIB_EXPORT_CLASS(franka_example_controllers_cyh::TeleopExampleController_cyh,
                       controller_interface::ControllerBase)