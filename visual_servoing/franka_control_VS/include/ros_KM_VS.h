#ifndef ROS_KM_VS
#define ROS_KM_VS

#include "ros_VS.h"
#include "Krawtchouk_moments_vs.h"

class Ros_KM_VS : public Ros_VS
{
    public:
        Krawtchouk_Moments_VS *KM_VS;

    public:
        Ros_KM_VS();
        virtual void Callback(const ImageConstPtr& image_color_msg, const ImageConstPtr& image_depth_msg);     
};

#endif