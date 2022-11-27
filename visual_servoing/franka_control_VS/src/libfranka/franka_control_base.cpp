#include "franka_control_base.h"

void franka_move_to_target_joint_angle(moveit::planning_interface::MoveGroupInterface& move_group_interface, std::vector<double> joint_group_positions_target)
{
    move_group_interface.setJointValueTarget(joint_group_positions_target);
    move_group_interface.setMaxAccelerationScalingFactor(0.05);
    move_group_interface.setMaxVelocityScalingFactor(0.05);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(success)
    {
        std::cout << "Press Enter to move the robot..." << std::endl;
        std::cin.ignore();
        move_group_interface.move();
        std::cout << "Move finish" << std::endl;
    }
    else
    {
        std::cout << "moveit joint plan fail ! ! !" << std::endl;
    }
}
ControlSwitcher::ControlSwitcher()
{
    map<string, string> controllers;
    controllers.insert(pair<string,string>("moveit","position_joint_trajectory_controller"));
    controllers.insert(pair<string,string>("velocity","cartesian_velocity_node_controller"));

    this->controllers_ = controllers;
    ros::service::waitForService("/controller_manager/switch_controller");
    this->switcher_srv_ = this->nh_.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");
 }

bool ControlSwitcher::switch_controllers(string start_controller_name, string stop_controller_name)
{
    sleep(0.5);

    string start_controller = this->controllers_[start_controller_name];
    string stop_controller = this->controllers_[stop_controller_name];

    controller_manager_msgs::SwitchController controller_switch_msg;
    controller_switch_msg.request.strictness = 1;
    controller_switch_msg.request.start_controllers.push_back(start_controller);
    controller_switch_msg.request.stop_controllers.push_back(stop_controller);

    if (this->switcher_srv_.call(controller_switch_msg))
    {
        cout << "Successfully switched " << stop_controller_name << " to " << start_controller_name << endl;
        return true;
    }
    else
    {
         cout << "Failed switched " << stop_controller_name << " to " << start_controller_name << endl;
         return false;
    }
}

