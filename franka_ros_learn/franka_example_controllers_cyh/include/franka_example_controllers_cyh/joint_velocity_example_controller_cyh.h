#pragma once

#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <ros/node_handle.h>
#include <ros/time.h>

namespace franka_example_controllers_cyh {
    class JointVelocityExampleController_cyh : 
            public controller_interface::MultiInterfaceController
            <hardware_interface::VelocityJointInterface, 
            franka_hw::FrankaStateInterface>
    {
        public:
        bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
        void update(const ros::Time&, const ros::Duration& period) override;
        void starting(const ros::Time&) override;
        void stopping(const ros::Time&) override;

        private:
        hardware_interface::VelocityJointInterface* velocity_joint_interface_;
        std::vector<hardware_interface::JointHandle> velocity_joint_handles_;
        ros::Duration elapsed_time_;
    };
}