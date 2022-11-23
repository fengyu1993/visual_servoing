#include "franka_control_base.h"

void franka_move_to_target_joint_angle(Mat joint_angle)
{
    // franka::Robot robot("172.16.0.1");
    // setDefaultBehavior(robot);

    // std::array<double, 7> q_goal = 
    // {{((double*)joint_angle.data)[0], ((double*)joint_angle.data)[1], ((double*)joint_angle.data)[2],
    //   ((double*)joint_angle.data)[3], ((double*)joint_angle.data)[4], ((double*)joint_angle.data)[5],
    //   ((double*)joint_angle.data)[6]}};

    // MotionGenerator motion_generator(0.2, q_goal);
    // std::cout << "WARNING: it will move the robot! "
    //           << "Please make sure to have the user stop button at hand!" << std::endl
    //           << "Press Enter to continue..." << std::endl;
    // std::cin.ignore();
    // robot.control(motion_generator);
    // std::cout << "Finished moving to joint configuration." << std::endl;
}