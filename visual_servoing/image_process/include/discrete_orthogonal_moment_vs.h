#ifndef DISCRETE_ORTHOGONAL_MOMENT_VS
#define DISCRETE_ORTHOGONAL_MOMENT_VS

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "direct_visual_servoing.h"
#include <iostream>
#include <fstream>
#include <string>
#include <ctime> 
#include <chrono>

using namespace cv;
using namespace std;

class Discrete_Orthogonal_Moment_VS: public Direct_Visual_Servoing
{
    public:
        int N_;
        int M_;
        int order_min_;
        int order_max_;
        int order_;
        Mat DOM_x_;
        Mat DOM_y_;
        double error_pixel_ave_;
        double delta_epsilon_;
        double lambda_order_;

        struct data_dom
        {
            Mat order_list_;
            Mat error_pixel_ave_;
        } data_dom;
        
        
    public: 
        Discrete_Orthogonal_Moment_VS(int order_min, int order_max, double delta_epsilon, double lambda_order, int resolution_x, int resolution_y);

        virtual void get_feature_error_interaction_matrix();

        virtual void get_DOM_matrix() = 0; //  DOM_x_ DOM_y_

        int get_order_adaption();

        Mat get_interaction_matrix_DOM_once(Mat& DOM_XY, Mat& L_I);

        Mat linspace(double begin, double finish, int number);

        virtual void save_data_other_parameter();

        void save_data_order();

        void save_data_error_pixel_ave();

        virtual void save_data_moments_parameter() {};

        virtual void write_other_data(ofstream& oFile);

        void write_data_error_pixel_ave(ofstream& oFile);

        void write_data_order(ofstream& oFile);

        virtual void write_data_moments(ofstream& oFile){};

        virtual string get_method_name();

        virtual bool is_success();

};


#endif