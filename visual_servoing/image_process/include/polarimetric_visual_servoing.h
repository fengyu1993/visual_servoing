#ifndef POLARIMETRIC_VISUAL_SERVOING
#define POLARIMETRIC_VISUAL_SERVOING

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <time.h>
#include "geometry_msgs/Twist.h"
#include <vector>

using namespace cv;
using namespace std;

class Polarimetric_Visual_Servoing
{
    public:
        Mat image_I_0_desired_, image_I_45_desired_, image_I_90_desired_, image_I_135_desired_;
        Mat image_I_0_current_, image_I_45_current_, image_I_90_current_, image_I_135_current_;
        Mat image_I_0_initial_, image_I_45_initial_, image_I_90_initial_, image_I_135_initial_;
        Mat image_depth_desired_;
        Mat image_depth_current_; 
        Mat camera_intrinsic_;
        Mat camera_velocity_; // vx vy vz wx wy wz
        Mat pose_desired_;
        double lambda_;
        double epsilon_; 
        int resolution_x_;
        int resolution_y_;
        Mat L_e_;
        Mat error_s_;
        bool flag_first_;
        int iteration_num_;
        clock_t start_VS_time_;
        struct data
        {
            Mat velocity_;
            Mat pose_; // 平移+四元数wxyz
            Mat pose_desired_; // 平移+四元数wxyz
            Mat error_feature_;
            Mat image_I_0_initial_, image_I_45_initial_, image_I_90_initial_, image_I_135_initial_;
            Mat image_I_0_desired_, image_I_45_desired_, image_I_90_desired_, image_I_135_desired_;
            Mat time_vs_;
        } data_pvs;

    public: 
        Polarimetric_Visual_Servoing(int resolution_x, int resolution_y);

        void init_VS(double lambda, double epsilon, 
            Mat& image_I_0_desired_, Mat& image_I_45_desired_, Mat& image_I_90_desired_, Mat& image_I_135_desired_, Mat& image_depth_desired,  
            Mat& image_I_0_initial_, Mat& image_I_45_initial_, Mat& image_I_90_initial_, Mat& image_I_135_initial_, Mat camera_intrinsic, Mat pose_desired);

        Mat get_camera_velocity();

        bool is_success();

        void get_feature_error_interaction_matrix();

        void set_camera_intrinsic(Mat& camera_intrinsic);

        void set_image_gray_desired(Mat& image_I_0_desired_, Mat& image_I_45_desired_, Mat& image_I_90_desired_, Mat& image_I_135_desired_);

        void set_image_gray_current(Mat& image_I_0_current, Mat& image_I_45_current, Mat& image_I_90_current, Mat& image_I_135_current);

        void set_image_gray_initial(Mat& image_I_0_initial, Mat& image_I_45_initial, Mat& image_I_90_initial, Mat& image_I_135_initial);

        void set_image_depth_desired(Mat& image_depth_desired);

        void set_image_depth_current(Mat& image_depth_current);

        void set_pose_desired(Mat& pose_desired);

        void save_pose_desired();

        void save_data_image();

        void save_data_image_gray_desired();

        void save_data_image_gray_initial();

        void save_data_camera_velocity();

        void save_data_error_feature();

        void save_data_camera_pose(Mat& pose);

        void save_data_vs_time();

        void save_data(Mat pose);

        void write_data();

        void write_visual_servoing_data(ofstream& oFile);

        string get_save_file_name(); 

        string get_date_time();

        string get_method_name();

        void write_to_excel(Mat data, ofstream& oFile);
};


#endif