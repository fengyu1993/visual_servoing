#ifndef DIRECT_MICROSCOPIC_VISUAL_SERVOING
#define DIRECT_MICROSCOPIC_VISUAL_SERVOING

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "microscopic_visual_servoing.h"

using namespace cv;
using namespace std;

class Direct_Microscopic_Visual_Servoing: public Microscopic_Visual_Servoing
{
    private:
        Mat div_col_;
        Mat div_row_;
        double Phi_;
        Mat Mat_x_;
        Mat Mat_y_;
        double A_;
        double B_;
        double C_;

    public: 
        Direct_Microscopic_Visual_Servoing(int resolution_x=640, int resolution_y=480);

        virtual void get_feature_error_interaction_matrix();

        void get_interaction_matrix_gray();

        void get_image_gradient_x(const Mat& image, Mat& I_x);

        void get_image_gradient_y(const Mat& image, Mat& I_y);

        virtual string get_method_name();
};


#endif