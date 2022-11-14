#ifndef TCHEBICHEF_MOMENTS_VS
#define TCHEBICHEF_MOMENTS_VS

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "discrete_orthogonal_moment_vs.h"

using namespace cv;
using namespace std;

class Techebichef_Moments_VS: public Discrete_Orthogonal_Moment_VS
{        
    public: 
        Techebichef_Moments_VS(int order_min, int order_max, int resolution_x, int resolution_y)
            : Discrete_Orthogonal_Moment_VS(order_min, order_max, resolution_x, resolution_y){};

        virtual void get_DOM_matrix(); //  DOM_x_ DOM_y_

        Mat get_orthogonal_polynomial_HM(int N, int order);

        virtual string get_method_name();

       
};


#endif
