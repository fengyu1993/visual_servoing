#ifndef ROS_DVS
#define ROS_DVS

#include "ros_VS.h"
#include "direct_visual_servoing.h"

class Ros_DVS : public Ros_VS
{
    public:
        Direct_Visual_Servoing *DVS;

    public:
        Ros_DVS();
        virtual void Callback(const ImageConstPtr& image_color_msg, const ImageConstPtr& image_depth_msg);     
};

#endif