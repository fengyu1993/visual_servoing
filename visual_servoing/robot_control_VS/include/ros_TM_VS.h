#ifndef ROS_TM_VS
#define ROS_TM_VS

#include "ros_VS.h"
#include "Tchebichef_moments_vs.h"

class Ros_TM_VS : public Ros_VS
{
    public:
        Techebichef_Moments_VS *TM_VS;

    public:
        Ros_TM_VS();
        virtual void Callback(const ImageConstPtr& image_color_msg, const ImageConstPtr& image_depth_msg);     
};

#endif