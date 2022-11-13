#ifndef VISUAL_SERVOING
#define VISUAL_SERVOING

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

class Visual_Servoing
{
    protected:
        Mat image_gray_desired_;
        Mat image_gray_current_; 
        Mat image_gray_initial_;
        Mat image_depth_desired_;
        Mat image_depth_current_; 
        Mat camera_intrinsic_;
        Mat camera_velocity_;
        double lambda_;
        double epsilon_; 
        int resolution_x_;
        int resolution_y_;
        struct data
        {
            Mat velocity_;
            Mat error_feature_ave_;
            Mat image_gray_init_;
            Mat image_gray_desired_;
        } data_vs;

    public:
        Mat L_e_;
        Mat error_s_;

    public: 
        Visual_Servoing(int resolution_x, int resolution_y);

        void init_VS(double lambda, double epsilon, Mat image_gray_desired, Mat image_depth_desired, Mat image_gray_initial, Mat camera_intrinsic);

        Mat get_camera_velocity();

        virtual void get_feature_error_interaction_matrix() = 0;

        void set_camera_intrinsic(Mat camera_intrinsic);

        void set_image_gray_desired(Mat image_gray_desired);

        void set_image_gray_current(Mat image_gray_current);

        void set_image_gray_initial(Mat image_gray_initial);

        void set_image_depth_desired(Mat image_depth_desired);

        void set_image_depth_current(Mat image_depth_current);

        void save_data_image();

        void save_data_image_gray_desired();

        void save_data_image_gray_initial();

        void save_data_camera_velocity();

        void save_data_error_feature_ave();
};


#endif