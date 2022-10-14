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
        velocity_joint_interface_ = robot_hardware->get<hardware_interface::VelocityJointInterface>();
        if (velocity_joint_interface_ == nullptr) {
            ROS_ERROR(
                "TeleopExampleController_cyh: Error getting velocity joint interface from hardware!");
            return false;
        }

        std::string arm_id;
        if (!node_handle.getParam("arm_id", arm_id)) {
            ROS_ERROR("TeleopExampleController_cyh: Could not get parameter arm_id");
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

        velocity_joint_handles_.resize(7);
        for (size_t i = 0; i < 7; ++i) {
            try {
            velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names[i]);
            } catch (const hardware_interface::HardwareInterfaceException& ex) {
            ROS_ERROR_STREAM(
                "TeleopExampleController_cyh: Exception getting joint handles: " << ex.what());
            return false;
            }
        }

        auto* state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
        if (state_interface == nullptr) {
            ROS_ERROR("TeleopExampleController_cyh: Could not get state interface from hardware");
            return false;
        }

        try {
            auto state_handle = state_interface->getHandle(arm_id + "_robot");
            robot_state_ = state_handle.getRobotState();
        } catch (hardware_interface::HardwareInterfaceException& ex) {
            ROS_ERROR_STREAM("TeleopExampleController_cyh: Exception getting state handle from interface: " << ex.what());
            return false;
        } 

        if (!node_handle.getParam("joint_start_pose", q_home_)  || q_home_.size() != 7)
        {
            ROS_ERROR("TeleopExampleController_cyh: Invalid or no q_home_ parameters provided, aborting controller init!");
            return false;
        }

        if (!node_handle.getParam("lambda_joint_velocity", lambda_joint_velocity_))
        {
            ROS_ERROR("TeleopExampleController_cyh: Invalid or no lambda_joint_velocity_ parameters provided, aborting controller init!");
            return false;
        }       

        return true;
    }

    void TeleopExampleController_cyh::starting(const ros::Time&)
    {
        elapsed_time_ = ros::Duration(0.0);
    }

    void TeleopExampleController_cyh::update(const ros::Time&, const ros::Duration& period)
    {
        elapsed_time_ += period;
        std::array<double,7> q_error = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        std::array<double,7> dq = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        for(size_t i = 0; i < 7; ++i)
        {
            q_current_[i] = robot_state_.q[i];
        }

        int cnt = 0;
        for(size_t i = 0; i < 7; i++)
        {
            q_error[i] = q_current_[i] - q_home_[i];
            dq[i] = -lambda_joint_velocity_ * q_error[i];
            if (dq[i] > dq_max_)
            {
                dq[i] = dq_max_;
            }
            else if(dq[i] < 0.001)
            {
                dq[i] = 0;
                cnt += 1;
            }
            velocity_joint_handles_[i].setCommand(dq[i]);
        }  
        if(cnt == 7)
        {
            ROS_INFO("Return to start: Done");
        }     
    }

    void TeleopExampleController_cyh::stopping(const ros::Time&) 
    {

    }

}

PLUGINLIB_EXPORT_CLASS(franka_example_controllers_cyh::TeleopExampleController_cyh,
                       controller_interface::ControllerBase)