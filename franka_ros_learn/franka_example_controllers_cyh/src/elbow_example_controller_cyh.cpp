#include <franka_example_controllers_cyh/elbow_example_controller_cyh.h>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <controller_interface/controller_base.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_example_controllers_cyh{
    bool ElbowExampleController_cyh::init(hardware_interface::RobotHW* robot_hardware,
                                  ros::NodeHandle& node_handle) 
    {
        cartesian_pose_interface_ = robot_hardware->get<franka_hw::FrankaPoseCartesianInterface>();
        if (cartesian_pose_interface_ == nullptr)
        {
            ROS_ERROR("ElbowExampleController_cyh: Could not get Cartesian Pose interface from hardware");
            return false;
        }

        std::string arm_id;
        if(!node_handle.getParam("arm_id", arm_id))
        {
            ROS_ERROR("ElbowExampleController_cyh: Could not get parameter arm_id");
            return false;
        }

        try{
            cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(cartesian_pose_interface_->getHandle(arm_id + "_robot"));
        }catch(const hardware_interface::HardwareInterfaceException& e){
            ROS_ERROR_STREAM("ElbowExampleController_cyh: Exception getting Cartesian handle: " << e.what());
            return false;
        }

        auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
        if(state_interface == nullptr)
        {
            ROS_ERROR("ElbowExampleController_cyh: Could not get state interface from hardware");
            return false;
        }

        try{
            auto state_handle = state_interface->getHandle(arm_id + "_robot");

            std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
            for(size_t i=0; i < q_start.size(); i++)
            {
                if(std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1)
                {
                    ROS_ERROR_STREAM(
                        "ElbowExampleController_cyh: Robot is not in the expected starting position for "
                        "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
                        "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
                    return false;
                }
            }    
        }catch(const hardware_interface::HardwareInterfaceException& e) 
        {
            ROS_ERROR_STREAM("ElbowExampleController_cyh: Exception getting state handle: " << e.what());
            return false;
        } 
        return true;
    }

    void ElbowExampleController_cyh::starting(const ros::Time& )
    {
        initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
        initial_elbow_ = cartesian_pose_handle_->getRobotState().elbow_d;
        elapsed_time_ = ros::Duration(0.0);
    }

    void ElbowExampleController_cyh::update(const ros::Time& , const ros::Duration& period)
    {
        elapsed_time_ += period;
        
        double angle = M_PI / 10.0 * (1.0 - std::cos(M_PI / 5.0 * elapsed_time_.toSec()));
        auto elbow = initial_elbow_;
        elbow[0] += angle;

        cartesian_pose_handle_->setCommand(initial_pose_, elbow);
    }

}

PLUGINLIB_EXPORT_CLASS(franka_example_controllers_cyh::ElbowExampleController_cyh,
                       controller_interface::ControllerBase)



















