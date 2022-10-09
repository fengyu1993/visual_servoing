#include <franka_example_controllers_cyh/joint_velocity_example_controller_cyh.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_example_controllers_cyh
{
    bool JointVelocityExampleController_cyh::init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle)
    {
        velocity_joint_interface_ = robot_hardware->get<hardware_interface::VelocityJointInterface>();
        if (velocity_joint_interface_ == nullptr)
        {
            ROS_ERROR("JointVelocityExampleController_cyh: Error getting velocity joint interface from hardware!");
            return false;
        }

        std::string arm_id;
        if(!node_handle.getParam("arm_id", arm_id))
        {
            ROS_ERROR("JointVelocityExampleController_cyh: Could not get parameter arm_id");
            return false;
        }

        std::vector<std::string> joint_names;
        if(!node_handle.getParam("joint_names", joint_names))
        {
            ROS_ERROR("JointVelocityExampleController_cyh: Could not parse joint names");
        }
        if(joint_names.size() != 7)
        {
            ROS_ERROR_STREAM("JointVelocityExampleController_cyh: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
            return false;
        }
        velocity_joint_handles_.resize(7);
        for(size_t i = 0; i < 7; ++i)
        {
            try{
                velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names[i]);
            }catch (const hardware_interface::HardwareInterfaceException& ex) {
                ROS_ERROR_STREAM(
                    "JointVelocityExampleController_cyh: Exception getting joint handles: " << ex.what());
                return false;
            }
        }

        auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
        if(state_interface == nullptr)
        {
            ROS_ERROR("JointVelocityExampleController: Could not get state interface from hardware");
            return false;
        }

        try{
            auto state_handle = state_interface->getHandle(arm_id + "_robot");

            std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
            for (size_t i = 0; i < q_start.size(); i++)
            {
                if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1)
                {
                    ROS_ERROR_STREAM(
                        "JointVelocityExampleController_cyh: Robot is not in the expected starting position for "
                        "running this example. Run `roslaunch franka_example_controllers_cyh move_to_start.launch "
                        "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
                    return false;
                }
            }
        }catch (const hardware_interface::HardwareInterfaceException& e) {
            ROS_ERROR_STREAM(
                "JointVelocityExampleController_cyh: Exception getting state handle: " << e.what());
                return false;
        }

        return true;
    }

    void JointVelocityExampleController_cyh::starting(const ros::Time& )
    {
        elapsed_time_ = ros::Duration(0.0);
    }
    
    void JointVelocityExampleController_cyh::update(const ros::Time&, const ros::Duration& period)
    {
        elapsed_time_ += period; 

        ros::Duration time_max(8.0);
        double omega_max = 0.1;
        double cycle = std::floor(std::pow(-1.0, (elapsed_time_.toSec() - std::fmod(elapsed_time_.toSec(), time_max.toSec())) /
                         time_max.toSec()));
        double omega = cycle * omega_max / 2.0 *
                 (1.0 - std::cos(2.0 * M_PI / time_max.toSec() * elapsed_time_.toSec()));

        for(auto joint_handle:velocity_joint_handles_)
        {
            joint_handle.setCommand(omega);
        }
    }

    void JointVelocityExampleController_cyh::stopping(const ros::Time&)
    {

    }
} // namespace name

PLUGINLIB_EXPORT_CLASS(franka_example_controllers_cyh::JointVelocityExampleController_cyh,
                       controller_interface::ControllerBase)
