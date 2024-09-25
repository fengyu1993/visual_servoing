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
        Mat camera_intrinsic_inv_;
        Mat camera_velocity_; // vx vy vz wx wy wz
        Mat pose_desired_;
        Mat O_desired_;          Mat O_current_;
        Mat A_desired_;          Mat A_current_;
        Mat Phi_desired_;        Mat Phi_current_;
        Mat V_; 
        Mat n_desired_;          Mat n_current_;
        Mat S_desired_;          Mat S_current_;
        Mat us_desired_;         Mat us_current_;
        Mat ud_desired_;         Mat ud_current_;   
        Mat Is_desired_;         Mat Is_current_;
        Mat Id_desired_;         Mat Id_current_;
        Mat Iu_;     
        Mat rho_desired_;        Mat rho_current_;
        Mat theta_desired_;      Mat theta_current_;
        Mat phi_desired_;        Mat phi_current_;
        Mat rho_dp_desired_;     Mat rho_dp_current_;
        Mat rho_sp_desired_;     Mat rho_sp_current_;
        Mat d_rho_dp_theta_desired_;    Mat d_rho_dp_theta_current_;
        Mat d_rho_sp_theta_desired_;    Mat d_rho_sp_theta_current_;
        double lambda_;
        double epsilon_; 
        Mat eta_; 
        double phi_pol_; 
        double k_;
        double I_in_k_s_pi_;
        double I_in_k_d_;
        double I_a_k_a_;
        int resolution_x_;
        int resolution_y_;
        Mat L_e_;
        Mat error_s_;
        bool flag_first_;
        bool flag_a_or_b;
        int iteration_num_;
        clock_t start_VS_time_;
        Mat col_row_reshape_;
        Mat L_I_pol_a_real_desired_;        Mat L_I_pol_a_real_current_;
        Mat L_I_pol_b_real_desired_;        Mat L_I_pol_b_real_current_;       

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

        void init_VS(double lambda, double epsilon, double eta, double phi_pol, double k, 
            Mat& image_I_0_desired, Mat& image_I_45_desired, Mat& image_I_90_desired, Mat& image_I_135_desired, Mat& image_depth_desired,  
            Mat& image_I_0_initial, Mat& image_I_45_initial, Mat& image_I_90_initial, Mat& image_I_135_initial, Mat camera_intrinsic, Mat pose_desired);

        void init_VS(double lambda, double epsilon, double eta, double phi_pol, double k, 
            Mat& image_O_desired, Mat& image_A_desired, Mat& image_Phi_desired, Mat& image_depth_desired,  
            Mat& image_O_initial, Mat& image_A_initial, Mat& image_Phi_initial, Mat camera_intrinsic, Mat pose_desired);

        Mat get_L_kappa(Mat& camera_intrinsic);

        void get_O_A_Phi(Mat I_0, Mat I_45, Mat I_90, Mat I_135, Mat& O, Mat& A, Mat& Phi);
        
        void get_O_A_Phi_desired();
        void get_O_A_Phi_current();

        void get_Phong_model_init();

        void get_polar_data_desired();
        void get_polar_data_current();

        void get_Phong_us_ud_desired();
        void get_Phong_us_ud_current();

        void get_I_in_k_s_pi_I_in_k_d_I_a_k_a();

        void get_Is_Id_Iu_desired();
        void get_Is_Id_Iu_current();

        void get_rho_theta_phi_desired();
        void get_rho_theta_phi_current();

        void get_n_desired();
        void get_n_current(); 

        void get_rho_dp_sp_d_rho_sp_theta_d_rho_dp_theta(Mat theta, Mat eta, Mat& rho_dp, Mat& rho_sp, Mat& d_rho_dp_theta, Mat& d_rho_sp_theta);

        void get_rho_dp_sp_desired();
        
        void get_interaction_matrix_desired();
        void get_interaction_matrix_current();

        void get_interaction_matrix_desired_a();
        void get_interaction_matrix_current_a();

        void get_interaction_matrix_desired_b();
        void get_interaction_matrix_current_b();
        
        Mat get_P_desried();
        Mat get_P_current();
        
        void get_rho_dp_sp_current();
        
        Mat get_L_n(Mat n);

        Mat get_L_theta(double phi);

        Mat get_L_phi(double theta, double phi);

        Mat get_L_rho_sp(double d_rho_sp_d_theta, Mat L_theta);

        Mat get_L_rho_dp(double d_rho_dp_d_theta, Mat L_theta);

        Mat get_L_V(Mat P);

        Mat get_dV_dP(Mat P);

        Mat get_L_P(Mat P);

        Mat get_L_Sa(Mat S);
        Mat get_L_Sb(Mat P);

        Mat get_L_udb(Mat n, Mat S, Mat L_n, Mat L_Sb);

        Mat get_L_usa(double ud, Mat n, Mat S, Mat V, Mat L_n_desired, Mat L_V_desired, Mat L_Sa_desired);
        Mat get_L_usb(double ud, Mat n, Mat S, Mat V, Mat L_n_desired, Mat L_V_desired, Mat L_Sb_desired, Mat L_udb_desired);

        void get_T_perpendicular_parallel(Mat theta, Mat eta, Mat& T_s, Mat& T_p);  
        
        Mat get_theta(Mat K, Mat eta);

        pair<Mat,Mat> gradient(Mat & img, double spaceX, double spaceY);

        static Mat gradientX(Mat & mat, double spacing);

        static Mat gradientY(Mat & mat, double spacing);

        Mat cv_cos(Mat a);

        Mat cv_sin(Mat a);

        Mat cv_acos(Mat a);

        Mat cv_asin(Mat a);

        Mat cv_atan2(Mat a, Mat b);

        Mat VecToso3(Mat omg);

        Mat get_camera_velocity();

        bool is_success();

        void get_feature_error_interaction_matrix();

        void get_feature_error();

        void set_camera_intrinsic(Mat& camera_intrinsic);

        void set_image_gray_desired(Mat& image_I_0_desired_, Mat& image_I_45_desired_, Mat& image_I_90_desired_, Mat& image_I_135_desired_);
        
        void set_image_gray_current(Mat& image_I_0_current, Mat& image_I_45_current, Mat& image_I_90_current, Mat& image_I_135_current);

        void set_image_gray_initial(Mat& image_I_0_initial, Mat& image_I_45_initial, Mat& image_I_90_initial, Mat& image_I_135_initial);

        void set_image_depth_desired(Mat& image_depth_desired);

        void set_image_depth_current(Mat& image_depth_current);

        void set_image_polar_current(Mat& polar_O_current, Mat& polar_A_current, Mat& polar_Phi_current);

        void set_image_polar_desired(Mat& polar_O_desired, Mat& polar_A_desired, Mat& polar_Phi_desired);

        void set_image_polar_initial(Mat& image_O_initial, Mat& image_A_initial, Mat& image_Phi_initial);

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