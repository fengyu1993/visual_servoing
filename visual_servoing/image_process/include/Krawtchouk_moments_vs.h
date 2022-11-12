#ifndef KRAWTCHOUK_MOMENTS_VS
#define KRAWTCHOUK_MOMENTS_VS

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "discrete_orthogonal_moment_vs.h"

using namespace cv;
using namespace std;

class Krawtchouk_Moments_VS: public Discrete_Orthogonal_Moment_VS
{  
    private:
        double px;
        double py;

    public: 
        Krawtchouk_Moments_VS(int order_min, int order_max, int resolution_x, int resolution_y);
        
        virtual void get_DOM_matrix(); //  DOM_x_ DOM_y_

        Mat get_orthogonal_polynomial_KM(int N, int order, double p);

        void get_Krawtchouk_Moments_parameters();

        inline double w_x_x_1_Krawtchouk(int N, int x, double p);
       
        inline double w_x_x_2_Krawtchouk(int N, int x, double p);
};


#endif
