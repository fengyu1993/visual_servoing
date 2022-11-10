#ifndef DIRECT_COSINE_TRANSFORM_VS
#define DIRECT_COSINE_TRANSFORM_VS

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "visual_servoing.h"

using namespace cv;
using namespace std;

class Direct_Cosine_Transform_VS: public Visual_Servoing
{
    private:
        
    public: 
        Direct_Cosine_Transform_VS(int resolution_x, int resolution_y);

        virtual Mat get_feature_error();

        virtual Mat get_interaction_matrix();
};


#endif