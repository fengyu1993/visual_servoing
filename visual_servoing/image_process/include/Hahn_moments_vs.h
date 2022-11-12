#ifndef Hahn_MOMENTS_VS
#define Hahn_MOMENTS_VS

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "discrete_orthogonal_moment_vs.h"

using namespace cv;
using namespace std;

class Hahn_Moments_VS: public Discrete_Orthogonal_Moment_VS
{  
    private:
        double ax;
        double bx;
        double ay;
        double by;

    public: 
        Hahn_Moments_VS(int order_min, int order_max, int resolution_x, int resolution_y);
        
        virtual void get_DOM_matrix(); //  DOM_x_ DOM_y_

        Mat get_orthogonal_polynomial_HM(int N, int order, double ax, double bx);

        void get_Hahn_Moments_parameters();

        void get_image_spread(Mat img, int xc, int yc, int& sx, int& sy);

        inline double w_x_x_1_Hahn(int N, int x, double p);
       
        inline double w_x_x_2_Hahn(int N, int x, double p);
};


#endif
