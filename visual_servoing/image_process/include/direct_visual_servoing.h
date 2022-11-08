#ifndef DIRECT_VISUAL_SERVOING
#define DIRECT_VISUAL_SERVOING

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

using namespace cv;

class Direct_Visual_Servoing
{
    private:
        Mat image_desired;
        Mat image_initial;
        Mat image_current; 
        double lambda

    public:   
        // Discrete_Orthogonal_Moment_VS();

};


#endif