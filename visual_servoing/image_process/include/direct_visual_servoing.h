#ifndef DIRECT_VISUAL_SERVOING
#define DIRECT_VISUAL_SERVOING

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

using namespace cv;

class Direct_Visual_Servoing
{
    private:
        Mat image_gray_desired_;
        Mat image_gray_initial_;
        Mat image_gray_current_; 
        Mat image_depth_desired_;
        Mat image_depth_initial_;
        Mat image_depth_current_; 
        Mat L_e_;
        Mat error_s_;
        Mat camera_intrinsic_;
        double lambda_;
        double epsilon_; 
        int resolution_x_;
        int resolution_y_;
        struct sava_data
        {
            Mat error_pose_;
            Mat velocity_;
            Mat pose_camera_;
            Mat error_pixel_ave_;
            Mat error_feature_;
            Mat image_gray_init_;
            Mat image_gray_desired_;
        };

    public: 
        Direct_Visual_Servoing(double lambda, double epsilon, Mat camera_intrinsic, int resolution_x, int resolution_y);

        void get_FeaturesError_InteractionMatrix_DVS(Mat image_gray_new, Mat image_depth_new, 
                Mat image_gray_old, Mat image_depth_old, Mat Camera_Intrinsic, Mat& error_s, Mat& L_e);

        void get_feature_error_gray(Mat image_gray_new, Mat image_gray_old, Mat& error_s);

        void get_interaction_matrix_gray_depth(Mat image_gray, Mat image_depth, Mat Camera_Intrinsic, Mat& L_e);

        void get_image_gradient(Mat image, Mat Camera_Intrinsic, Mat& I_x, Mat& I_y);
};


#endif