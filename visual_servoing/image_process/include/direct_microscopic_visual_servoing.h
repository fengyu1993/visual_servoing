#ifndef DIRECT_MICROSCOPIC_VISUAL_SERVOING
#define DIRECT_MICROSCOPIC_VISUAL_SERVOING

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "microscopic_visual_servoing.h"

using namespace cv;
using namespace std;

class Direct_Microscopic_Visual_Servoing: public Microscopic_Visual_Servoing
{
    public: 
        Direct_Microscopic_Visual_Servoing(int resolution_x=640, int resolution_y=480);

        virtual void get_feature_error_interaction_matrix();

        Mat get_interaction_matrix_gray();

        // void get_image_gradient(Mat& image, Mat& Camera_Intrinsic, Mat& I_x, Mat& I_y);

        // Mat get_image_gradient_x(Mat& image);

        // Mat get_image_gradient_y(Mat& image);

        virtual string get_method_name();
};


#endif