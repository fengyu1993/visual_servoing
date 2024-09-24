#ifndef ROS_DCT_VS
#define ROS_DCT_VS

#include "ros_VS.h"
#include "discrete_cosine_transform_vs.h"

class Ros_DCT_VS : public Ros_VS
{
    public:
        Direct_Cosine_Transform_VS *DCT_VS;

    public:
        Ros_DCT_VS();
        virtual void Callback(const ImageConstPtr& image_color_msg, const ImageConstPtr& image_depth_msg);     
};

#endif