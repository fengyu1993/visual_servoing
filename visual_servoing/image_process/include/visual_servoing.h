#ifndef VISUAL_SERVOING
#define VISUAL_SERVOING

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <time.h>
# include "geometry_msgs/Twist.h"

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
        Mat camera_velocity_; // vx vy vz wx wy wz
        Mat pose_desired_;
        double lambda_;
        double epsilon_; 
        int resolution_x_;
        int resolution_y_;

    public:
        Mat L_e_;
        Mat error_s_;
        struct data
        {
            Mat velocity_;
            Mat pose_; // 平移+四元数wxyz
            Mat pose_desired_; // 平移+四元数wxyz
            Mat error_feature_;
            Mat image_gray_init_;
            Mat image_gray_desired_;
        } data_vs;

    public: 
        Visual_Servoing(int resolution_x, int resolution_y);

        void init_VS(double lambda, double epsilon, Mat& image_gray_desired, Mat& image_depth_desired, Mat image_gray_initial, Mat camera_intrinsic, Mat pose_desired);

        Mat get_camera_velocity();

        virtual bool is_success();

        virtual void get_feature_error_interaction_matrix() = 0;

        void set_camera_intrinsic(Mat& camera_intrinsic);

        void set_image_gray_desired(Mat& image_gray_desired);

        void set_image_gray_current(Mat& image_gray_current);

        void set_image_gray_initial(Mat& image_gray_initial);

        void set_image_depth_desired(Mat& image_depth_desired);

        void set_image_depth_current(Mat& image_depth_current);

        void set_pose_desired(Mat& pose_desired);

        void save_pose_desired();

        void save_data_image();

        void save_data_image_gray_desired();

        void save_data_image_gray_initial();

        void save_data_camera_velocity();

        virtual void save_data_error_feature();

        void save_data_camera_pose(Mat& pose);

        virtual void save_data_other_parameter() {};

        void save_data(Mat pose);

        void write_data();

        void write_visual_servoing_data(ofstream& oFile);

        virtual void write_other_data(ofstream& oFile){};

        string get_save_file_name(); 

        string get_date_time();

        virtual string get_method_name();

        void write_to_excel(Mat data, ofstream& oFile);
};


#endif