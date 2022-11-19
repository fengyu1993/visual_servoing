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
        int ax;
        int bx;
        int ay;
        int by;

        struct data_hm
        {
            Mat ax_list_;
            Mat bx_list_;
            Mat ay_list_;
            Mat by_list_;
        } data_hm;

    public: 
        Hahn_Moments_VS(int order_min, int order_max, double delta_epsilon, double lambda_order, int resolution_x, int resolution_y);
        
        virtual void get_DOM_matrix(); //  DOM_x_ DOM_y_

        Mat get_orthogonal_polynomial_HM(int N, int order, int a, int b);

        void get_Hahn_Moments_parameters();

        void get_image_spread(Mat& img, int xc, int yc, int& sx, int& sy);

        inline double w_x_x_1_Hahn(int N, int x, int a, int b);
       
        inline double w_x_x_2_Hahn(int N, int x, int a, int b);

        virtual void save_data_moments_parameter();

        virtual void write_data_moments(ofstream& oFile);

        virtual string get_method_name();
};


#endif
