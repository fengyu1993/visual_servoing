#pragma once

#include <array>
#include <string>
#include <vector>
#include <memory>
#include <string>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>


namespace franka_example_controllers_cyh
{
    class MoveToStart_cyh : public controller_interface::MultiInterfaceController
        <   franka_hw::FrankaModelInterface,
            hardware_interface::PositionJointInterface, 
            franka_hw::FrankaStateInterface >
    {
        public:
        bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
        void starting(const ros::Time&) override;
        void update(const ros::Time&, const ros::Duration& period) override;
        void stopping(const ros::Time&) override;

        private:
        hardware_interface::PositionJointInterface* position_joint_interface_;
        std::vector<hardware_interface::JointHandle> position_joint_handles_;
        franka::RobotState robot_state_;
        ros::Duration elapsed_time_;
        
        std::vector<double> q_home_{};
        std::vector<double> q_current_{};
        double dq_max_;
    };
}


