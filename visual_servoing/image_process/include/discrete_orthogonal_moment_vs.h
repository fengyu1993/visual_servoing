#ifndef DISCRETE_ORTHOGONAL_MOMENT_VS
#define DISCRETE_ORTHOGONAL_MOMENT_VS

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "direct_visual_servoing.h"

using namespace cv;
using namespace std;

class Discrete_Orthogonal_Moment_VS: public Direct_Visual_Servoing
{
    protected:
        int N_;
        int M_;
        int order_min_;
        int order_max_;
        int order_;
        Mat DOM_x_;
        Mat DOM_y_;

        
    public: 
        Discrete_Orthogonal_Moment_VS(int order_min, int order_max, int resolution_x, int resolution_y);

        virtual void get_feature_error_interaction_matrix();

        virtual void get_DOM_matrix() = 0; // º∆À„ DOM_x_ DOM_y_

        int get_order_adaption();

        Mat get_interaction_matrix_DOM_once(Mat DOM_XY, Mat L_I);

};


#endif