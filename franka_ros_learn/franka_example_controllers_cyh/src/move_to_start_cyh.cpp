#include <franka_example_controllers_cyh/move_to_start_cyh.h>

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
    bool MoveToStart_cyh::init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle)
    {
        position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();
        if (position_joint_interface_ == nullptr)
        {
            ROS_ERROR("MoveToStart_cyh: Error getting position joint interface from hardware!");
            return false;
        }

        std::string arm_id;
        if (!node_handle.getParam("arm_id", arm_id)) {
            ROS_ERROR("MoveToStart_cyh: Could not get parameter arm_id");
            return false;
        }

        std::vector<std::string> joint_names;
        if (!node_handle.getParam("joint_names", joint_names))
        {
            ROS_ERROR("MoveToStart_cyh: Could not parse joint names");
        }
        if (joint_names.size() != 7)
        {
            ROS_ERROR_STREAM("MoveToStart_cyh: Wrong number of joint names, got "
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
                    "MoveToStart_cyh: Exception getting joint handles: " << e.what());
                return false;
            }
        }

        auto* state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
        if (not state_interface) {
            ROS_ERROR("MoveToStart_cyh: Could not get state interface from hardware");
            return false;
        }
        try {
            state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
                state_interface->getHandle(arm_id + "_robot"));
        } catch (hardware_interface::HardwareInterfaceException& ex) {
            ROS_ERROR_STREAM("MoveToStart_cyh: Exception getting state handle from interface: " << ex.what());
            return false;
        } 

        if (!node_handle.getParam("joint_start_pose", q_home_)  || q_home_.size() != 7)
        {
            ROS_ERROR("MoveToStart_cyh: Invalid or no q_home_ parameters provided, aborting controller init!");
            return false;
        }     

        if (!node_handle.getParam("move_time", move_time_))
        {
            ROS_ERROR("MoveToStart_cyh: Invalid or no move_time_ parameters provided, aborting controller init!");
            return false;
        }        

        return true;
    }

    void MoveToStart_cyh::starting(const ros::Time&)
    {
        elapsed_time_ = ros::Duration(0.0);
        num_control = 0;
    }

    void MoveToStart_cyh::update(const ros::Time& time_0, const ros::Duration& period)
    {
        elapsed_time_ += period;
        num_control += 1;
        double t = num_control * period.toSec();
        printf("t=%0.5f\n", t);
        if (t > move_time_) 
        {
            t = move_time_;
        }
        std::array<double,7> q_error = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        franka::RobotState robot_state = state_handle_->getRobotState();
        for(size_t i = 0; i < 7; ++i)
        {
            printf("cyh_%ld\n", i);
            q_current_[i] = robot_state.q.at(i);
        }
        printf("cyh\n");
        int cnt = 0;
        for(size_t i = 0; i < 7; i++)
        {
            q_error[i] = q_home_[i] - q_current_[i];

            if (q_error[i] <= 0.005)
            {
                printf("cyh_a\n");
                cnt = cnt + 1;
                position_joint_handles_[i].setCommand(q_home_[i]);
            }
            else
            {
                printf("cyh_b\n");
                double lambda = 10*pow(t / move_time_, 3) 
                            - 15*pow(t / move_time_, 4) 
                            + 6*pow(t / move_time_, 5);  
                double step = q_error[i]*lambda;
                printf("step=%0.5f\n", step);
                if (step > 0.1)
                {
                    step = 0.1;
                }   
                position_joint_handles_[i].setCommand(step + q_current_[i]);
            }
        }  
        if(cnt == 7)
        {
            ROS_INFO("Return to start: Done");
        }     
    }

    void MoveToStart_cyh::stopping(const ros::Time&) 
    {

    }

}

PLUGINLIB_EXPORT_CLASS(franka_example_controllers_cyh::MoveToStart_cyh,
                       controller_interface::ControllerBase)