#ifndef ROS_HM_VS
#define ROS_HM_VS

#include "ros_VS.h"
#include "Hahn_moments_vs.h"

class Ros_HM_VS : public Ros_VS
{
    public:
        Hahn_Moments_VS *HM_VS;

    public:
        Ros_HM_VS();
        virtual void Callback(const ImageConstPtr& image_color_msg, const ImageConstPtr& image_depth_msg);     
};

#endif