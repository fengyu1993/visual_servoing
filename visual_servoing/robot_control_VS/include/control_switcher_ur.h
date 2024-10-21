#ifndef CONTROLSWITCHER_UR
#define CONTROLSWITCHER_UR

#include <iostream>
#include <ros/ros.h>

using namespace std;

class ControlSwitcher_UR
{
    ros::NodeHandle nh_;
    map<string, string> controllers_;
    ros::ServiceClient switcher_srv_;
    
    public:
    ControlSwitcher_UR()
    {
        map<string, string> controllers;
        controllers.insert(pair<string,string>("position","pos_joint_traj_controller"));
        controllers.insert(pair<string,string>("twist","twist_controller"));

        this->controllers_ = controllers;
        ros::service::waitForService("/controller_manager/switch_controller");
        this->switcher_srv_ = this->nh_.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");
    }

    bool switch_controllers(string start_controller_name, string stop_controller_name)
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
};

#endif
