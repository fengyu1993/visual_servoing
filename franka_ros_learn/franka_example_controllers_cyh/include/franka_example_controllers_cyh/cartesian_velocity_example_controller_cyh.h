#pragma once 

#include <memory>
#include <string>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

namespace franka_example_controllers_cyh
{
    class CartesianVelocityExampleController_cyh : 
            public controller_interface::MultiInterfaceController
                <franka_hw::FrankaVelocityCartesianInterface, franka_hw::FrankaStateInterface> 
    {
        public:
        bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
        void update(const ros::Time&, const ros::Duration& period) override;
        void starting(const ros::Time&) override;
        void stopping(const ros::Time&) override;

        private:
        franka_hw::FrankaVelocityCartesianInterface* velocity_cartesian_interface_;
        std::unique_ptr<franka_hw::FrankaCartesianVelocityHandle> velocity_cartesian_handle_;
        ros::Duration elapsed_time_;
    };
}