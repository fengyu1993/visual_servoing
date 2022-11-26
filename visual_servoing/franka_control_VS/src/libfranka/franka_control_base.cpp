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

