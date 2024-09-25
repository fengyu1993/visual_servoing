#include "polarimetric_visual_servoing.h"
#include <opencv2/imgproc.hpp>
#include <math.h>
#include <iostream>
#include <fstream>
#include <string>
#include <ctime> 
#include <chrono>
#include <cmath>


Polarimetric_Visual_Servoing::Polarimetric_Visual_Servoing(int resolution_x=640, int resolution_y=480)
{
    this->resolution_x_ = resolution_x;
    this->resolution_y_ = resolution_y;

    this->image_I_0_desired_  = Mat::zeros(this->resolution_y_, this->resolution_x_, CV_64FC1);
    this->image_I_45_desired_ = Mat::zeros(this->resolution_y_, this->resolution_x_, CV_64FC1);
    this->image_I_90_desired_ = Mat::zeros(this->resolution_y_, this->resolution_x_, CV_64FC1);
    this->image_I_135_desired_= Mat::zeros(this->resolution_y_, this->resolution_x_, CV_64FC1);

    this->image_I_0_current_  = Mat::zeros(this->resolution_y_, this->resolution_x_, CV_64FC1);
    this->image_I_45_current_ = Mat::zeros(this->resolution_y_, this->resolution_x_, CV_64FC1);
    this->image_I_90_current_ = Mat::zeros(this->resolution_y_, this->resolution_x_, CV_64FC1);
    this->image_I_135_current_= Mat::zeros(this->resolution_y_, this->resolution_x_, CV_64FC1);
 
    this->image_depth_desired_ = Mat::zeros(this->resolution_y_, this->resolution_x_, CV_64FC1);
    this->image_depth_current_ = Mat::zeros(this->resolution_y_, this->resolution_x_, CV_64FC1);

    this->camera_intrinsic_ = Mat::zeros(3, 3, CV_64FC1);
    this->camera_intrinsic_inv_ = Mat::zeros(3, 3, CV_64FC1);
    this->camera_velocity_ = Mat::zeros(6, 1, CV_64FC1);
	this->flag_first_ = true;
    this->flag_a_or_b = true;
	this->iteration_num_ = 0;

    // 多维数组转为行存储
    this->O_desired_ = Mat::zeros(1, this->resolution_y_ * this->resolution_x_, CV_64FC1);
    this->O_current_ = Mat::zeros(1, this->resolution_y_ * this->resolution_x_, CV_64FC1);
    this->A_desired_ = Mat::zeros(1, this->resolution_y_ * this->resolution_x_, CV_64FC1);          
    this->A_current_ = Mat::zeros(1, this->resolution_y_ * this->resolution_x_, CV_64FC1);
    this->Phi_desired_ = Mat::zeros(1, this->resolution_y_ * this->resolution_x_, CV_64FC1);        
    this->Phi_current_ = Mat::zeros(1, this->resolution_y_ * this->resolution_x_, CV_64FC1);

    this->us_desired_ = Mat::zeros(1, this->resolution_y_ * this->resolution_x_, CV_64FC1);
    this->us_current_ = Mat::zeros(1, this->resolution_y_ * this->resolution_x_, CV_64FC1);
    this->ud_desired_ = Mat::zeros(1, this->resolution_y_ * this->resolution_x_, CV_64FC1);
    this->ud_current_ = Mat::zeros(1, this->resolution_y_ * this->resolution_x_, CV_64FC1); 

    this->Is_desired_ = Mat::zeros(1, this->resolution_y_ * this->resolution_x_, CV_64FC1); 
    this->Is_current_ = Mat::zeros(1, this->resolution_y_ * this->resolution_x_, CV_64FC1); 
    this->Id_desired_ = Mat::zeros(1, this->resolution_y_ * this->resolution_x_, CV_64FC1); 
    this->Id_current_ = Mat::zeros(1, this->resolution_y_ * this->resolution_x_, CV_64FC1); 
    this->Iu_ = Mat::zeros(1, this->resolution_y_ * this->resolution_x_, CV_64FC1);  

    this->rho_desired_ = Mat::zeros(1, this->resolution_y_ * this->resolution_x_, CV_64FC1); 
    this->rho_current_ = Mat::zeros(1, this->resolution_y_ * this->resolution_x_, CV_64FC1); 
    this->theta_desired_ = Mat::zeros(1, this->resolution_y_ * this->resolution_x_, CV_64FC1); 
    this->phi_desired_ = Mat::zeros(1, this->resolution_y_ * this->resolution_x_,  CV_64FC1); 
    this->phi_current_ = Mat::zeros(1, this->resolution_y_ * this->resolution_x_,  CV_64FC1); 

    this->rho_dp_desired_ = Mat::zeros(1, this->resolution_y_ * this->resolution_x_,  CV_64FC1); 
    this->rho_sp_desired_ = Mat::zeros(1, this->resolution_y_ * this->resolution_x_,  CV_64FC1);   
    this->rho_dp_current_ = Mat::zeros(1, this->resolution_y_ * this->resolution_x_,  CV_64FC1); 
    this->rho_sp_current_ = Mat::zeros(1, this->resolution_y_ * this->resolution_x_,  CV_64FC1); 

    this->d_rho_dp_theta_desired_ = Mat::zeros(1, this->resolution_y_ * this->resolution_x_,  CV_64FC1); 
    this->d_rho_sp_theta_desired_ = Mat::zeros(1, this->resolution_y_ * this->resolution_x_,  CV_64FC1);   
    this->d_rho_dp_theta_current_ = Mat::zeros(1, this->resolution_y_ * this->resolution_x_,  CV_64FC1); 
    this->d_rho_sp_theta_current_ = Mat::zeros(1, this->resolution_y_ * this->resolution_x_,  CV_64FC1); 

    this->V_ = Mat::zeros(3, this->resolution_y_ * this->resolution_x_, CV_64FC1);
    this->n_desired_ = Mat::zeros(3, this->resolution_y_ * this->resolution_x_, CV_64FC1);
    this->n_current_ = Mat::zeros(3, this->resolution_y_ * this->resolution_x_, CV_64FC1);
    this->S_desired_ = Mat::zeros(3, this->resolution_y_ * this->resolution_x_, CV_64FC1);
    this->S_current_ = Mat::zeros(3, this->resolution_y_ * this->resolution_x_, CV_64FC1);

    this->col_row_reshape_ = Mat::zeros(3, this->resolution_y_ * this->resolution_x_, CV_64FC1); 

    this->L_I_pol_a_real_desired_ = Mat::zeros(this->resolution_y_ * this->resolution_x_, 6, CV_64FC1);
    this->L_I_pol_a_real_current_ = Mat::zeros(this->resolution_y_ * this->resolution_x_, 6, CV_64FC1);

    this->L_I_pol_b_real_desired_ = Mat::zeros(this->resolution_y_ * this->resolution_x_, 6, CV_64FC1);
    this->L_I_pol_b_real_current_ = Mat::zeros(this->resolution_y_ * this->resolution_x_, 6, CV_64FC1);
}


// 初始化
void Polarimetric_Visual_Servoing::init_VS(double lambda, double epsilon, double eta, double phi_pol, double k, 
            Mat& image_I_0_desired, Mat& image_I_45_desired, Mat& image_I_90_desired, Mat& image_I_135_desired, Mat& image_depth_desired,   
            Mat& image_I_0_initial, Mat& image_I_45_initial, Mat& image_I_90_initial, Mat& image_I_135_initial, Mat camera_intrinsic, Mat pose_desired)
{
    this->lambda_ = lambda;
    this->epsilon_ = epsilon;
    set_camera_intrinsic(camera_intrinsic);
    set_image_depth_desired(image_depth_desired);
    set_image_gray_desired(image_I_0_desired, image_I_45_desired, image_I_90_desired, image_I_135_desired);
    set_image_gray_initial(image_I_0_initial, image_I_45_initial, image_I_90_initial, image_I_135_initial);
    set_pose_desired(pose_desired);
    save_data_image();
    save_pose_desired();
    this->eta_ = eta * Mat::ones(1, this->resolution_y_ * this->resolution_x_, CV_64FC1); 
    this->phi_pol_ = phi_pol; 
    this->k_ = k;
}

void Polarimetric_Visual_Servoing::init_VS(double lambda, double epsilon, double eta, double phi_pol, double k, 
            Mat& image_O_desired, Mat& image_A_desired, Mat& image_Phi_desired, Mat& image_depth_desired,  
            Mat& image_O_initial, Mat& image_A_initial, Mat& image_Phi_initial, Mat camera_intrinsic, Mat pose_desired)
{
    this->lambda_ = lambda;
    this->epsilon_ = epsilon;
    set_camera_intrinsic(camera_intrinsic);
    set_image_depth_desired(image_depth_desired);
    set_image_polar_desired(image_O_desired, image_A_desired, image_Phi_desired);
    set_image_polar_initial(image_O_initial, image_A_initial, image_Phi_initial);
    set_pose_desired(pose_desired);
    save_data_image();
    save_pose_desired();
    this->eta_ = eta * Mat::ones(1, this->resolution_y_ * this->resolution_x_, CV_64FC1); 
    this->phi_pol_ = phi_pol; 
    this->k_ = k;

}

// 初始化Phong模型 V_, n_desired_, n_current_, S_desired_, S_current_
// PVS.V_.col(row * PVS.resolution_x_ + col) << endl;
void Polarimetric_Visual_Servoing::get_Phong_model_init()
{
    double row = 1, col = 1;
    Mat temp = (Mat_<double>(3,1) << 0.0, 0.0, -1.0);

    for(int i = 0; i < V_.cols; i++)
    {
        this->V_.col(i) = this->camera_intrinsic_inv_ * cv::Vec3d(col, row, 1.0);
        this->V_.col(i) = this->V_.col(i) / norm(this->V_.col(i));
        temp.copyTo(this->n_desired_.col(i));
        temp.copyTo(this->n_current_.col(i));
        temp.copyTo(this->S_desired_.col(i));
        temp.copyTo(this->S_current_.col(i));
        if(++col > this->resolution_x_)
        {
            col = 1;
            ++row;
        }
    }
}

void Polarimetric_Visual_Servoing::get_polar_data_desired()
{
    get_O_A_Phi_desired();

    get_Phong_us_ud_desired(); 

    get_I_in_k_s_pi_I_in_k_d_I_a_k_a();

    get_Is_Id_Iu_desired();

    get_rho_theta_phi_desired();

    get_rho_dp_sp_desired();

    get_n_desired();       
}

void Polarimetric_Visual_Servoing::get_polar_data_current()
{
    get_O_A_Phi_current();
    
    get_Phong_us_ud_current();

    get_Is_Id_Iu_current();

    get_rho_theta_phi_current();    

    get_rho_dp_sp_current();

    get_n_current(); 
}

// 计算Phong模型 us_desired ud_desired
void Polarimetric_Visual_Servoing::get_Phong_us_ud_desired()
{
    // int cnt = 0;
    // for (int row = 0; row < this->us_desired_.rows; ++row) 
    // {
    //     double *us_d = (double *)(this->us_desired_.data + row*this->us_desired_.step[0]); 
    //     double *ud_d = (double *)(this->ud_desired_.data + row*this->ud_desired_.step[0]); 

    //     for (int col = 0; col < us_desired_.cols; ++col) 
    //     {          
    //         ud_d[col] = this->n_desired_.col(cnt).dot(this->S_desired_.col(cnt));
    //         us_d[col] = this->V_.col(cnt).dot(2*ud_d[col]*this->n_desired_.col(cnt) - this->S_desired_.col(cnt));
    //         cnt++;
    //     }
    // }


    this->ud_desired_ = this->n_desired_.row(0).mul(this->S_desired_.row(0)) + 
                this->n_desired_.row(1).mul(this->S_desired_.row(1)) +
                this->n_desired_.row(2).mul(this->S_desired_.row(2));

    Mat temp_1 = 2.0*this->ud_desired_.mul(this->n_desired_.row(0)) - this->S_desired_.row(0);
    Mat temp_2 = 2.0*this->ud_desired_.mul(this->n_desired_.row(1)) - this->S_desired_.row(1);
    Mat temp_3 = 2.0*this->ud_desired_.mul(this->n_desired_.row(2)) - this->S_desired_.row(2);

    this->us_desired_ =  this->V_.row(0).mul(temp_1) + 
                this->V_.row(1).mul(temp_2) + 
                this->V_.row(2).mul(temp_3);
}
// 计算Phong模型 us_current ud_current
void Polarimetric_Visual_Servoing::get_Phong_us_ud_current()
{
    this->ud_current_ = this->n_current_.row(0).mul(this->S_current_.row(0)) + 
                this->n_current_.row(1).mul(this->S_current_.row(1)) +
                this->n_current_.row(2).mul(this->S_current_.row(2));

    Mat temp_1 = 2*this->ud_current_.mul(this->n_current_.row(0)) - this->S_current_.row(0);
    Mat temp_2 = 2*this->ud_current_.mul(this->n_current_.row(1)) - this->S_current_.row(1);
    Mat temp_3 = 2*this->ud_current_.mul(this->n_current_.row(2)) - this->S_current_.row(2);

    this->us_current_ =  this->V_.row(0).mul(temp_1) + 
                this->V_.row(1).mul(temp_2) + 
                this->V_.row(2).mul(temp_3);
}

// 计算 I_in_k_s_pi I_in_k_d I_a_k_a
void Polarimetric_Visual_Servoing::get_I_in_k_s_pi_I_in_k_d_I_a_k_a()
{
    Mat US, UD, Jacob_pinv;
    cv::pow(this->us_desired_, this->k_, US);
    UD = this->ud_desired_;
    Mat Jacob = Mat::ones(this->resolution_y_ * this->resolution_x_, 3, CV_64FC1);
    Jacob.col(0) = US.t();
    Jacob.col(1) = UD.t();

    Mat sov = 2*this->O_desired_.t();
    invert(Jacob, Jacob_pinv, DECOMP_SVD);

    Mat temp = Jacob_pinv * sov;
    this->I_in_k_s_pi_ = temp.at<double>(0,0);
    this->I_in_k_d_ = temp.at<double>(1,0);
    this->I_a_k_a_ = temp.at<double>(2,0);
}

// 计算 Is_desired Id_desired Iu
void Polarimetric_Visual_Servoing::get_Is_Id_Iu_desired()
{
    Mat US; 
    cv::pow(this->us_desired_, this->k_, US);
    this->Is_desired_ = this->I_in_k_s_pi_ * US; 
    this->Id_desired_ = this->I_in_k_d_ * this->ud_desired_;
    this->Iu_ = this->I_a_k_a_ * Mat::ones(this->resolution_y_ * this->resolution_x_, 3, CV_64FC1);
}
// 计算 Is_current Id_current Iu
void Polarimetric_Visual_Servoing::get_Is_Id_Iu_current()
{
    Mat US; 
    cv::pow(this->us_current_, this->k_, US);
    this->Is_current_ = this->I_in_k_s_pi_ * US; 
    this->Id_current_ = this->I_in_k_d_ * this->ud_desired_;
 }

// 计算 rho_desired theta_desired phi_desired
void Polarimetric_Visual_Servoing::get_rho_theta_phi_desired()
{
    // 计算 rho
    this->rho_desired_ = this->A_desired_ / this->O_desired_;
    // 计算 zenith angle
    Mat T_s_desired = Mat::zeros(1, this->resolution_y_ * this->resolution_x_, CV_64FC1);
    Mat T_p_desired = Mat::zeros(1, this->resolution_y_ * this->resolution_x_, CV_64FC1);
    Mat K_desired = Mat::zeros(1, this->resolution_y_ * this->resolution_x_, CV_64FC1);
    Mat theta_desired_old = CV_PI/6 * Mat::ones(1, this->resolution_y_ * this->resolution_x_, CV_64FC1);
    for(int i = 0; i < 20; i++) 
    {
        get_T_perpendicular_parallel(theta_desired_old, this->eta_ , T_s_desired, T_p_desired);

        K_desired = 2.0*this->rho_desired_.mul(this->O_desired_) / (this->Id_desired_ - (this->Is_desired_ / (2 / (T_s_desired + T_p_desired) - 1)));

        this->theta_desired_ = get_theta(K_desired, this->eta_);

        if(norm(this->theta_desired_ - theta_desired_old) < 1e-6){
            break;
        }   
        else{
            theta_desired_old = this->theta_desired_;
        }     
    }
    // 计算 azimuth angle
    this->phi_desired_ = this->Phi_desired_ / 2.0;
    pair<Mat,Mat> grad = gradient(this->image_depth_desired_, 1.0, 1.0);
    Mat gradX = grad.first.reshape(0,1);
    Mat gradY = grad.second.reshape(0,1);
    Mat flag_desired = gradX.mul(cv_cos(this->phi_desired_)) + gradY.mul(cv_sin(this->phi_desired_));
    Mat id_desired = flag_desired < 0;
    Mat idx;
	cv::findNonZero(id_desired, idx); 
    for(int i = 0; i < idx.cols * idx.rows; i++)
    {
        this->phi_desired_.at<double>(idx.at<Point>(i).y, idx.at<Point>(i).x) = this->phi_desired_.at<double>(idx.at<Point>(i).y, idx.at<Point>(i).x) + CV_PI;
    }
}

// 计算 rho_current theta_current phi_current
void Polarimetric_Visual_Servoing::get_rho_theta_phi_current()
{
    // 计算 rho
    this->rho_current_ = this->A_current_ / this->O_current_;
    // 计算 zenith angle
    Mat T_s_current = Mat::zeros(1, this->resolution_y_ * this->resolution_x_, CV_64FC1);
    Mat T_p_current = Mat::zeros(1, this->resolution_y_ * this->resolution_x_, CV_64FC1);
    Mat K_current = Mat::zeros(1, this->resolution_y_ * this->resolution_x_, CV_64FC1);
    Mat theta_current_old = CV_PI/6 * Mat::ones(1, this->resolution_y_ * this->resolution_x_, CV_64FC1);
    for(int i = 0; i < 20; i++) 
    {
        get_T_perpendicular_parallel(theta_current_old, this->eta_ , T_s_current, T_p_current);
        K_current = 2*this->rho_current_.mul(this->O_current_) / (this->Id_current_ - (this->Is_current_ / (2 / (T_s_current + T_p_current) - 1)));
        this->theta_current_ = get_theta(K_current, this->eta_);
        if(norm(this->theta_current_ - theta_current_old) < 1e-6){
            break;
        }   
        else{
            theta_current_old = this->theta_current_;
        }     
    }
    // 计算 azimuth angle
    this->phi_current_ = this->Phi_current_ / 2;
    pair<Mat,Mat> grad = gradient(this->image_depth_current_, 1.0, 1.0);
    Mat gradX = grad.first.reshape(0,1);
    Mat gradY = grad.second.reshape(0,1);
    Mat flag_current = gradX.mul(cv_cos(this->phi_current_)) + gradY.mul(cv_sin(this->phi_current_));
    Mat id_current = flag_current < 0;
    Mat idx;
	cv::findNonZero(id_current, idx); 
    for(int i = 0; i < idx.cols * idx.rows; i++)
    {
        this->phi_current_.at<double>(idx.at<Point>(i).y, idx.at<Point>(i).x) = this->phi_current_.at<double>(idx.at<Point>(i).y, idx.at<Point>(i).x) + CV_PI;
    }    
}

// 计算 n_desired
void Polarimetric_Visual_Servoing::get_n_desired()
{
    Mat nx_desired = cv_cos(this->phi_desired_).mul(cv_sin(this->theta_desired_)); 
    Mat ny_desired = cv_sin(this->phi_desired_).mul(cv_sin(this->theta_desired_));
    Mat nz_desired = -cv_cos(this->theta_desired_);

   nx_desired.copyTo(this->n_desired_.row(0));
   ny_desired.copyTo(this->n_desired_.row(1));
   nz_desired.copyTo(this->n_desired_.row(2));
}

// 计算 n_current
void Polarimetric_Visual_Servoing::get_n_current()
{
    Mat nx_current = cv_cos(this->phi_current_).mul(cv_sin(this->theta_current_)); 
    Mat ny_current = cv_sin(this->phi_current_).mul(cv_sin(this->theta_current_));
    Mat nz_current = -cv_cos(this->theta_current_);

    nx_current.copyTo(this->n_current_.row(0));
    ny_current.copyTo(this->n_current_.row(1));
    nz_current.copyTo(this->n_current_.row(2));
}

// 计算 rho_dp rho_sp d_rho_sp_theta d_rho_dp_theta
void Polarimetric_Visual_Servoing::get_rho_dp_sp_d_rho_sp_theta_d_rho_dp_theta
                (Mat theta, Mat eta, Mat& rho_dp, Mat& rho_sp, Mat& d_rho_dp_theta, Mat& d_rho_sp_theta)
{
// rho_dp = ((eta_value - 1/eta_value).^2 .* sin(theta_value).^2) ./ ...
//     (2+2*eta_value.^2 - (eta_value + 1/eta_value).^2.*sin(theta_value).^2 + 4*cos(theta_value).*sqrt(eta_value.^2 - sin(theta_value).^2));
// rho_sp = (2*sin(theta_value).^2.*cos(theta_value).*sqrt(eta_value.^2 - sin(theta_value).^2)) ./ ...
//     (eta_value.^2 - sin(theta_value).^2 - eta_value.^2.*sin(theta_value).^2 + 2*sin(theta_value).^4);
// d_rho_sp_d_theta = (2*sin(theta).*(eta.^2 - sin(theta).^2 - eta.^2*sin(theta).^2) .* (2*eta.^2 - sin(theta).^2 - eta.^2.*sin(theta).^2)) ./ ...
//     (sqrt(eta.^2 - sin(theta).^2) .* (eta.^2 - sin(theta).^2 - eta.^2.*sin(theta).^2 + 2*sin(theta).^4).^2);
// d_rho_dp_d_theta = (eta.^2.*(eta.^2 - 1).^2 .* ((5*eta.^2 - 3).*sin(theta) + (eta.^2 + 1).*(sqrt(2).*sin(2*theta).*sqrt(2*eta.^2 + cos(2*theta) - 1) + sin(3*theta)))) ./ ...
//     (sqrt(eta.^2 - sin(theta).^2) .* (2*eta.^2 + 2*eta.^4 - (eta.^2 + 1).^2.*sin(theta).^2 + 4*eta.^2*cos(theta).*sqrt(eta.^2 - sin(theta).^2)).^2);
    Mat sin_theta = cv_sin(theta);
    Mat cos_theta = cv_cos(theta);
    Mat sin_theta_2 = sin_theta.mul(sin_theta);
    Mat eta_2 = eta.mul(eta);
    Mat eta_2_inv = 1.0 / eta_2;
    Mat temp_2 = eta_2 - sin_theta_2;
    Mat temp_sqrt;
    sqrt(temp_2, temp_sqrt);
    Mat d_rho_sp_theta_dec_temp = temp_2 - eta_2.mul(sin_theta_2) + 2*sin_theta_2.mul(sin_theta_2);
    Mat temp_d_rho_dp_theta_num_2 = 2*eta_2 + cv_cos(2*theta) - 1;
    Mat temp_d_rho_dp_theta_num;
    sqrt(temp_d_rho_dp_theta_num_2, temp_d_rho_dp_theta_num);
    Mat temp_d_rho_dp_theta_dec = 2*eta_2 + 2*eta_2.mul(eta_2) - (eta_2 + 1).mul(eta_2 + 1).mul(sin_theta_2) + 4*eta_2.mul(cv_cos(theta)).mul(temp_sqrt);
    // rho_sp
    Mat rho_sp_num = 2*sin_theta_2.mul(cos_theta).mul(temp_sqrt);
    Mat rho_sp_dec = eta_2 - sin_theta_2 - eta_2.mul(sin_theta_2) + 2*sin_theta_2.mul(sin_theta_2);
    rho_sp = rho_sp_num / rho_sp_dec;
    // rho_dp
    Mat rho_dp_num = (eta_2 + eta_2_inv - 2).mul(sin_theta_2);
    Mat rho_dp_dec = 2 + 2*eta_2 - (eta_2 + eta_2_inv + 2).mul(sin_theta_2) + 4*cos_theta.mul(temp_sqrt);
    rho_dp = rho_dp_num / rho_dp_dec;
    // d_rho_sp_theta
    Mat d_rho_sp_theta_num = 2*sin_theta.mul(temp_2 - eta_2.mul(sin_theta_2)).mul(2*eta_2 - sin_theta_2 - eta_2.mul(sin_theta_2));
    Mat d_rho_sp_theta_dec = temp_sqrt.mul(d_rho_sp_theta_dec_temp).mul(d_rho_sp_theta_dec_temp);
    d_rho_sp_theta = d_rho_sp_theta_num / d_rho_sp_theta_dec;
    // d_rho_dp_theta
    Mat d_rho_dp_theta_num = eta_2.mul(eta_2 - 1).mul(eta_2 - 1).mul((5*eta_2 - 3).mul(cv_sin(theta)) + (eta_2 + 1).mul(sqrt(2) * cv_sin(2*theta).mul(temp_d_rho_dp_theta_num) + cv_sin(3*theta)));
    Mat d_rho_dp_theta_dec = temp_sqrt.mul(temp_d_rho_dp_theta_dec).mul(temp_d_rho_dp_theta_dec);
    d_rho_dp_theta = d_rho_dp_theta_num / d_rho_dp_theta_dec;
}


// 计算 rho_dp_desired rho_sp_desired
void Polarimetric_Visual_Servoing::get_rho_dp_sp_desired()
{
    get_rho_dp_sp_d_rho_sp_theta_d_rho_dp_theta(this->theta_desired_, this->eta_, this->rho_dp_desired_, this->rho_sp_desired_, this->d_rho_dp_theta_desired_, this->d_rho_sp_theta_desired_);
}

// 计算 rho_dp_current rho_sp_current     
void Polarimetric_Visual_Servoing::get_rho_dp_sp_current()
{
    get_rho_dp_sp_d_rho_sp_theta_d_rho_dp_theta(this->theta_current_, this->eta_, this->rho_dp_current_, this->rho_sp_current_, this->d_rho_dp_theta_current_, this->d_rho_sp_theta_current_);
}

// 计算期望图像交互矩阵 
void Polarimetric_Visual_Servoing::get_interaction_matrix_desired()
{
    if(this->flag_a_or_b){
        get_interaction_matrix_desired_a();
    }
    else{
        get_interaction_matrix_desired_b();
    }
}

// 计算当前图像交互矩阵 
void Polarimetric_Visual_Servoing::get_interaction_matrix_current()
{
    if(this->flag_a_or_b){
        get_interaction_matrix_current_a();
    }
    else{
        get_interaction_matrix_current_b();
    }
}
// 计算期望图像交互矩阵
// the light source motionless with respect to the object frame and located at infinity
void Polarimetric_Visual_Servoing::get_interaction_matrix_desired_a()
{
    // 计算 L_I_desired 
    Mat L_I_desired = Mat::zeros(this->resolution_y_ * this->resolution_x_, 6, CV_64FC1);

    Mat P_desired = get_P_desried();
    Mat P_desired_x = P_desired.row(0) / P_desired.row(2);
    Mat P_desired_y = P_desired.row(1) / P_desired.row(2);
    Mat P_desired_z_inv = 1.0 / P_desired.row(2);

    Mat I;
    switch ((int)this->phi_pol_) 
    {
        case 0:
            I = this->image_I_0_desired_;
            break;
        case 45:
            I = this->image_I_45_desired_;
            break;
        case 90:
            I = this->image_I_90_desired_;
            break;
        case 135:
            I = this->image_I_135_desired_;
            break;
        default:
            I = this->image_I_0_desired_;
            break; 
    }
    pair<Mat,Mat> grad = gradient(I, 1.0, 1.0);
    Mat I_u_desired= Mat::zeros(6, this->resolution_y_ * this->resolution_x_, CV_64FC1);
    Mat I_v_desired= Mat::zeros(6, this->resolution_y_ * this->resolution_x_, CV_64FC1);
    grad.first.reshape(0,1).copyTo(I_u_desired.row(0));
    I_u_desired.row(0).copyTo(I_u_desired.row(1));
    I_u_desired.row(0).copyTo(I_u_desired.row(2));
    I_u_desired.row(0).copyTo(I_u_desired.row(3));
    I_u_desired.row(0).copyTo(I_u_desired.row(4));
    I_u_desired.row(0).copyTo(I_u_desired.row(5));
    grad.second.reshape(0,1).copyTo(I_v_desired.row(0));
    I_v_desired.row(0).copyTo(I_v_desired.row(1));
    I_v_desired.row(0).copyTo(I_v_desired.row(2));
    I_v_desired.row(0).copyTo(I_v_desired.row(3));
    I_v_desired.row(0).copyTo(I_v_desired.row(4));
    I_v_desired.row(0).copyTo(I_v_desired.row(5));

    Mat L_kappa = get_L_kappa(this->camera_intrinsic_);
    double ku = L_kappa.at<double>(0, 0);
    double kv = L_kappa.at<double>(1, 1);

    Mat temp = Mat::zeros(1, this->resolution_y_ * this->resolution_x_, CV_64FC1);

    Mat L_u_desired_x = Mat::zeros(6, this->resolution_y_ * this->resolution_x_, CV_64FC1);
    temp = ku*(-P_desired_z_inv);                       temp.copyTo(L_u_desired_x.row(0));
    temp = ku*(P_desired_x.mul(P_desired_z_inv));       temp.copyTo(L_u_desired_x.row(2));
    temp = ku*(P_desired_x.mul(P_desired_y));           temp.copyTo(L_u_desired_x.row(3));
    temp = ku*(-(1 + P_desired_x.mul(P_desired_x)));    temp.copyTo(L_u_desired_x.row(4));
    temp = ku*(P_desired_y);                            temp.copyTo(L_u_desired_x.row(5));

    Mat L_u_desired_y = Mat::zeros(6, this->resolution_y_ * this->resolution_x_, CV_64FC1);
    temp = kv*(-P_desired_z_inv);                        temp.copyTo(L_u_desired_y.row(1));
    temp = kv*(P_desired_y.mul(P_desired_z_inv));       temp.copyTo(L_u_desired_y.row(2));
    temp = kv*(1 + P_desired_y.mul(P_desired_y));       temp.copyTo(L_u_desired_y.row(3));
    temp = kv*(-P_desired_x.mul(P_desired_y));          temp.copyTo(L_u_desired_y.row(4));
    temp = kv*(-P_desired_x);                           temp.copyTo(L_u_desired_y.row(5));

    L_I_desired = (I_u_desired.mul(L_u_desired_x) + I_v_desired.mul(L_u_desired_y)).t();
    // 计算 L_I_pol_a_desired  
    Mat L_n_desired, L_theta_desired, L_phi_desired, L_rho_sp_desired, L_rho_dp_desired, 
        L_V_desired, L_Sa_desired, L_uda_desired, L_usa_desired, L_Ida_desired, L_Isa_desired, 
        L_Iu_desired, L_Oa_desired, L_Aa_desired, L_Phi_desired;

    double* prt_phi = (double*)(this->phi_desired_.data);
    double* prt_theta = (double*)(this->theta_desired_.data);
    double* prt_d_rho_sp_theta_desired = (double*)(this->d_rho_sp_theta_desired_.data);
    double* prt_d_rho_dp_theta_desired = (double*)(this->d_rho_dp_theta_desired_.data);
    double* prt_ud_desired = (double*)(this->ud_desired_.data);
    double* prt_us_desired = (double*)(this->us_desired_.data);
    double* prt_rho_dp_desired = (double*)(this->rho_dp_desired_.data);
    double* prt_rho_sp_desired = (double*)(this->rho_sp_desired_.data);
    double* prt_Id_desired = (double *)(this->Id_desired_.data);
    double* prt_Is_desired = (double *)(this->Is_desired_.data);
    double* prt_phi_desired = (double *)(this->phi_desired_.data);
    double* prt_A_desired = (double *)(this->A_desired_.data);

    Mat L_I_pol_a_desired = Mat::zeros(this->resolution_y_ * this->resolution_x_, 6, CV_64FC1);
    int row = 0, col = 0;
    for(int cnt = 0; cnt < L_I_pol_a_desired.rows; cnt++)
    {
        // L_n
        L_n_desired = get_L_n(this->n_desired_.col(cnt)); 
        // L_theta
        L_theta_desired = get_L_theta(*prt_phi);
        // L_phi
        L_phi_desired = get_L_phi(*prt_theta, *prt_phi);
        // L_rho_sp
        L_rho_sp_desired = get_L_rho_sp(*prt_d_rho_sp_theta_desired, L_theta_desired);
        // L_rho_dp 
        L_rho_dp_desired = get_L_rho_dp(*prt_d_rho_dp_theta_desired, L_theta_desired);
        // L_V
        L_V_desired = get_L_V(P_desired.col(cnt));
        // L_S
        L_Sa_desired = get_L_Sa(this->S_desired_.col(cnt));
        // L_ud
        L_uda_desired = Mat::zeros(1, 6, CV_64FC1);
        // L_us 
        L_usa_desired = get_L_usa(*prt_ud_desired, this->n_desired_.col(cnt), 
                            this->S_desired_.col(cnt), this->V_.col(cnt), L_n_desired, L_V_desired, L_Sa_desired);
        // L_Id
        L_Ida_desired = Mat::zeros(1, 6, CV_64FC1);
        // L_Is
        L_Isa_desired = this->I_in_k_s_pi_ * this->k_ * pow(*prt_us_desired, this->k_ - 1) * L_usa_desired;
        // L_Iu
        L_Iu_desired = Mat::zeros(1, 6, CV_64FC1);
        // L_O
        L_Oa_desired = 0.5 * (L_Isa_desired + L_Ida_desired + L_Iu_desired);
        // L_A
        L_Aa_desired = 0.5 * (*prt_rho_dp_desired * L_Ida_desired + *prt_Id_desired * L_rho_dp_desired 
                        - *prt_rho_sp_desired * L_Isa_desired - *prt_Is_desired * L_rho_sp_desired);
        // L_Phi
        L_Phi_desired = 2*L_phi_desired; 
        // L_I_pol
        L_I_pol_a_desired.row(cnt) = L_Oa_desired + cos(2*this->phi_pol_ - *prt_phi_desired) * L_Aa_desired + *prt_A_desired * sin(2*this->phi_pol_ - *prt_phi_desired) * L_Phi_desired;
        // 更新
        if(++col >= this->resolution_x_)
        {
            col = 0;
            ++row;
        }
        prt_phi++;
        prt_theta++;
        prt_d_rho_sp_theta_desired++;
        prt_d_rho_dp_theta_desired++;
        prt_ud_desired++;
        prt_us_desired++;
        prt_rho_dp_desired++;
        prt_rho_sp_desired++;
        prt_Id_desired++;
        prt_Is_desired++;
        prt_phi_desired++;
        prt_A_desired++;
    }
    // 计算 L_I_pol_a_real_desired 
    this->L_I_pol_a_real_desired_ = L_I_pol_a_desired - L_I_desired;
}

// 计算当前图像交互矩阵
// the light source motionless with respect to the object frame and located at infinity
void Polarimetric_Visual_Servoing::get_interaction_matrix_current_a()
{
    // 计算 L_I_current 
    Mat L_I_current = Mat::zeros(this->resolution_y_ * this->resolution_x_, 6, CV_64FC1);
    Mat P_current = get_P_current();
    Mat P_current_x = P_current.row(0) / P_current.row(2);
    Mat P_current_y = P_current.row(1) / P_current.row(2);
    Mat P_current_z_inv = 1.0 / P_current.row(2);

    Mat I;
    switch ((int)this->phi_pol_) 
    {
        case 0:
            I = this->image_I_0_current_;
            break;
        case 45:
            I = this->image_I_45_current_;
            break;
        case 90:
            I = this->image_I_90_current_;
            break;
        case 135:
            I = this->image_I_135_current_;
            break;
        default:
            I = this->image_I_0_current_;
            break; 
    }
    pair<Mat,Mat> grad = gradient(I, 1.0, 1.0);
    Mat I_u_current= Mat::zeros(6, this->resolution_y_ * this->resolution_x_, CV_64FC1);
    Mat I_v_current= Mat::zeros(6, this->resolution_y_ * this->resolution_x_, CV_64FC1);
    grad.first.reshape(0,1).copyTo(I_u_current.row(0));
    I_u_current.row(0).copyTo(I_u_current.row(1));
    I_u_current.row(0).copyTo(I_u_current.row(2));
    I_u_current.row(0).copyTo(I_u_current.row(3));
    I_u_current.row(0).copyTo(I_u_current.row(4));
    I_u_current.row(0).copyTo(I_u_current.row(5));
    grad.second.reshape(0,1).copyTo(I_v_current.row(0));
    I_v_current.row(0).copyTo(I_v_current.row(1));
    I_v_current.row(0).copyTo(I_v_current.row(2));
    I_v_current.row(0).copyTo(I_v_current.row(3));
    I_v_current.row(0).copyTo(I_v_current.row(4));
    I_v_current.row(0).copyTo(I_v_current.row(5));

    Mat L_kappa = get_L_kappa(this->camera_intrinsic_);
    double ku = L_kappa.at<double>(0, 0);
    double kv = L_kappa.at<double>(1, 1);

   Mat temp = Mat::zeros(1, this->resolution_y_ * this->resolution_x_, CV_64FC1);

    Mat L_u_current_x = Mat::zeros(6, this->resolution_y_ * this->resolution_x_, CV_64FC1);
    temp = ku*(-P_current_z_inv);                       temp.copyTo(L_u_current_x.row(0));
    temp = ku*(P_current_x.mul(P_current_z_inv));       temp.copyTo(L_u_current_x.row(2));
    temp = ku*(P_current_x.mul(P_current_y));           temp.copyTo(L_u_current_x.row(3));
    temp = ku*(-(1 + P_current_x.mul(P_current_x)));    temp.copyTo(L_u_current_x.row(4));
    temp = ku*(P_current_y);                            temp.copyTo(L_u_current_x.row(5));

    Mat L_u_current_y = Mat::zeros(6, this->resolution_y_ * this->resolution_x_, CV_64FC1);
    temp = kv*(-P_current_z_inv);                        temp.copyTo(L_u_current_y.row(1));
    temp = kv*(P_current_y.mul(P_current_z_inv));       temp.copyTo(L_u_current_y.row(2));
    temp = kv*(1 + P_current_y.mul(P_current_y));       temp.copyTo(L_u_current_y.row(3));
    temp = kv*(-P_current_x.mul(P_current_y));          temp.copyTo(L_u_current_y.row(4));
    temp = kv*(-P_current_x);                           temp.copyTo(L_u_current_y.row(5));

    L_I_current = (I_u_current.mul(L_u_current_x) + I_v_current.mul(L_u_current_y)).t();
    // 计算 L_I_pol_a_current  
    Mat L_n_current, L_theta_current, L_phi_current, L_rho_sp_current, L_rho_dp_current, 
        L_V_current, L_Sa_current, L_uda_current, L_usa_current, L_Ida_current, L_Isa_current, 
        L_Iu_current, L_Oa_current, L_Aa_current, L_Phi_current;

    double* prt_phi = (double*)(this->phi_current_.data);
    double* prt_theta = (double*)(this->theta_current_.data);
    double* prt_d_rho_sp_theta_current = (double*)(this->d_rho_sp_theta_current_.data);
    double* prt_d_rho_dp_theta_current = (double*)(this->d_rho_dp_theta_current_.data);
    double* prt_ud_current = (double*)(this->ud_current_.data);
    double* prt_us_current = (double*)(this->us_current_.data);
    double* prt_rho_dp_current = (double*)(this->rho_dp_current_.data);
    double* prt_rho_sp_current = (double*)(this->rho_sp_current_.data);
    double* prt_Id_current = (double *)(this->Id_current_.data);
    double* prt_Is_current = (double *)(this->Is_current_.data);
    double* prt_phi_current = (double *)(this->phi_current_.data);
    double* prt_A_current = (double *)(this->A_current_.data);

    Mat L_I_pol_a_current = Mat::zeros(this->resolution_y_ * this->resolution_x_, 6, CV_64FC1);
    int row = 0, col = 0;
    for(int cnt = 0; cnt < L_I_pol_a_current.rows; cnt++)
    {
        // L_n
        L_n_current = get_L_n(this->n_current_.col(cnt)); 
        // L_theta
        L_theta_current = get_L_theta(*prt_phi);
        // L_phi
        L_phi_current = get_L_phi(*prt_theta, *prt_phi);
        // L_rho_sp
        L_rho_sp_current = get_L_rho_sp(*prt_d_rho_sp_theta_current, L_theta_current);
        // L_rho_dp 
        L_rho_dp_current = get_L_rho_dp(*prt_d_rho_dp_theta_current, L_theta_current);
        // L_V
        L_V_current = get_L_V(P_current.col(cnt));
        // L_S
        L_Sa_current = get_L_Sa(this->S_current_.col(cnt));
        // L_ud
        L_uda_current = Mat::zeros(1, 6, CV_64FC1);
        // L_us 
        L_usa_current = get_L_usa(*prt_ud_current, this->n_current_.col(cnt), 
                            this->S_current_.col(cnt), this->V_.col(cnt), L_n_current, L_V_current, L_Sa_current);
        // L_Id
        L_Ida_current = Mat::zeros(1, 6, CV_64FC1);
        // L_Is
        L_Isa_current = this->I_in_k_s_pi_ * this->k_ * pow(*prt_us_current, this->k_ - 1) * L_usa_current;
        // L_Iu
        L_Iu_current = Mat::zeros(1, 6, CV_64FC1);
        // L_O
        L_Oa_current = 0.5 * (L_Isa_current + L_Ida_current + L_Iu_current);
        // L_A
        L_Aa_current = 0.5 * (*prt_rho_dp_current * L_Ida_current + *prt_Id_current * L_rho_dp_current 
                        - *prt_rho_sp_current * L_Isa_current - *prt_Is_current * L_rho_sp_current);
        // L_Phi
        L_Phi_current = 2*L_phi_current; 
        // L_I_pol
        L_I_pol_a_current.row(cnt) = L_Oa_current + cos(2*this->phi_pol_ - *prt_phi_current) * L_Aa_current + *prt_A_current * sin(2*this->phi_pol_ - *prt_phi_current) * L_Phi_current;
        // 更新
        if(++col >= this->resolution_x_)
        {
            col = 0;
            ++row;
        }
        prt_phi++;
        prt_theta++;
        prt_d_rho_sp_theta_current++;
        prt_d_rho_dp_theta_current++;
        prt_ud_current++;
        prt_us_current++;
        prt_rho_dp_current++;
        prt_rho_sp_current++;
        prt_Id_current++;
        prt_Is_current++;
        prt_phi_current++;
        prt_A_current++;
    }
    // 计算 L_I_pol_a_real_current 
    this->L_I_pol_a_real_current_ = L_I_pol_a_current - L_I_current;
}

// 计算期望图像交互矩阵 
// the light source mounted on the camera
void Polarimetric_Visual_Servoing::get_interaction_matrix_desired_b()
{
    // 计算 L_I_desired 
    Mat L_I_desired = Mat::zeros(this->resolution_y_ * this->resolution_x_, 6, CV_64FC1);
    Mat P_desired = get_P_desried();
    Mat P_desired_x = P_desired.row(0) / P_desired.row(2);
    Mat P_desired_y = P_desired.row(1) / P_desired.row(2);
    Mat P_desired_z_inv = 1.0 / P_desired.row(2);

    Mat I;
    switch ((int)this->phi_pol_) 
    {
        case 0:
            I = this->image_I_0_desired_;
            break;
        case 45:
            I = this->image_I_45_desired_;
            break;
        case 90:
            I = this->image_I_90_desired_;
            break;
        case 135:
            I = this->image_I_135_desired_;
            break;
        default:
            I = this->image_I_0_desired_;
            break; 
    }
    pair<Mat,Mat> grad = gradient(I, 1.0, 1.0);
    Mat I_u_desired= Mat::zeros(6, this->resolution_y_ * this->resolution_x_, CV_64FC1);
    Mat I_v_desired= Mat::zeros(6, this->resolution_y_ * this->resolution_x_, CV_64FC1);
    grad.first.reshape(0,1).copyTo(I_u_desired.row(0));
    I_u_desired.row(0).copyTo(I_u_desired.row(1));
    I_u_desired.row(0).copyTo(I_u_desired.row(2));
    I_u_desired.row(0).copyTo(I_u_desired.row(3));
    I_u_desired.row(0).copyTo(I_u_desired.row(4));
    I_u_desired.row(0).copyTo(I_u_desired.row(5));
    grad.second.reshape(0,1).copyTo(I_v_desired.row(0));
    I_v_desired.row(0).copyTo(I_v_desired.row(1));
    I_v_desired.row(0).copyTo(I_v_desired.row(2));
    I_v_desired.row(0).copyTo(I_v_desired.row(3));
    I_v_desired.row(0).copyTo(I_v_desired.row(4));
    I_v_desired.row(0).copyTo(I_v_desired.row(5));

    Mat L_kappa = get_L_kappa(this->camera_intrinsic_);
    double ku = L_kappa.at<double>(0, 0);
    double kv = L_kappa.at<double>(1, 1);

    Mat temp = Mat::zeros(1, this->resolution_y_ * this->resolution_x_, CV_64FC1);

    Mat L_u_desired_x = Mat::zeros(6, this->resolution_y_ * this->resolution_x_, CV_64FC1);
    temp = ku*(-P_desired_z_inv);                       temp.copyTo(L_u_desired_x.row(0));
    temp = ku*(P_desired_x.mul(P_desired_z_inv));       temp.copyTo(L_u_desired_x.row(2));
    temp = ku*(P_desired_x.mul(P_desired_y));           temp.copyTo(L_u_desired_x.row(3));
    temp = ku*(-(1 + P_desired_x.mul(P_desired_x)));    temp.copyTo(L_u_desired_x.row(4));
    temp = ku*(P_desired_y);                            temp.copyTo(L_u_desired_x.row(5));

    Mat L_u_desired_y = Mat::zeros(6, this->resolution_y_ * this->resolution_x_, CV_64FC1);
    temp = kv*(-P_desired_z_inv);                        temp.copyTo(L_u_desired_y.row(1));
    temp = kv*(P_desired_y.mul(P_desired_z_inv));       temp.copyTo(L_u_desired_y.row(2));
    temp = kv*(1 + P_desired_y.mul(P_desired_y));       temp.copyTo(L_u_desired_y.row(3));
    temp = kv*(-P_desired_x.mul(P_desired_y));          temp.copyTo(L_u_desired_y.row(4));
    temp = kv*(-P_desired_x);                           temp.copyTo(L_u_desired_y.row(5));

    L_I_desired = (I_u_desired.mul(L_u_desired_x) + I_v_desired.mul(L_u_desired_y)).t();

    // 计算 L_I_pol_b_desired
    Mat L_n_desired, L_theta_desired, L_phi_desired, L_rho_sp_desired, L_rho_dp_desired, 
        L_V_desired, L_Sb_desired, L_udb_desired, L_usb_desired, L_Idb_desired, L_Isb_desired, 
        L_Iu_desired, L_Ob_desired, L_Ab_desired, L_Phi_desired;
    
    double* prt_phi = (double*)(this->phi_desired_.data);
    double* prt_theta = (double*)(this->theta_desired_.data);
    double* prt_d_rho_sp_theta_desired = (double*)(this->d_rho_sp_theta_desired_.data);
    double* prt_d_rho_dp_theta_desired = (double*)(this->d_rho_dp_theta_desired_.data);
    double* prt_ud_desired = (double*)(this->ud_desired_.data);
    double* prt_us_desired = (double*)(this->us_desired_.data);
    double* prt_rho_dp_desired = (double*)(this->rho_dp_desired_.data);
    double* prt_rho_sp_desired = (double*)(this->rho_sp_desired_.data);
    double* prt_Id_desired = (double *)(this->Id_desired_.data);
    double* prt_Is_desired = (double *)(this->Is_desired_.data);
    double* prt_phi_desired = (double *)(this->phi_desired_.data);
    double* prt_A_desired = (double *)(this->A_desired_.data);

    Mat L_I_pol_b_desired = Mat::zeros(this->resolution_y_ * this->resolution_x_, 6, CV_64FC1);
    int row = 0, col = 0;
    for(int cnt = 0; cnt < L_I_pol_b_desired.rows; cnt++)
    {
        // L_n
        L_n_desired = get_L_n(this->n_desired_.col(cnt)); 
        // L_theta
        L_theta_desired = get_L_theta(*prt_phi);
        // L_phi
        L_phi_desired = get_L_phi(*prt_theta, *prt_phi);
        // L_rho_sp
        L_rho_sp_desired = get_L_rho_sp(*prt_d_rho_sp_theta_desired, L_theta_desired);
        // L_rho_dp 
        L_rho_dp_desired = get_L_rho_dp(*prt_d_rho_dp_theta_desired, L_theta_desired);
        // L_V
        L_V_desired = get_L_V(P_desired.col(cnt));
        // L_S
        L_Sb_desired = get_L_Sb(P_desired.col(cnt));
        // L_ud
        L_udb_desired =  get_L_udb(this->n_desired_.col(cnt), this->S_desired_.col(cnt), L_n_desired, L_Sb_desired);             
        // L_us 
        L_usb_desired = get_L_usb(*prt_ud_desired, this->n_desired_.col(cnt), 
                            this->S_desired_.col(cnt), this->V_.col(cnt), L_n_desired, L_V_desired, L_Sb_desired, L_udb_desired);
        // L_Id
        L_Idb_desired = this->I_in_k_d_ * L_udb_desired;
        // L_Is
        L_Isb_desired = this->I_in_k_s_pi_ * this->k_ * pow(*prt_us_desired, this->k_-1) * L_usb_desired;       
        // L_Iu
        L_Iu_desired = Mat::zeros(1, 6, CV_64FC1);
        // L_O
        L_Ob_desired = 0.5 * (L_Isb_desired + L_Idb_desired + L_Iu_desired);
        // L_A
        L_Ab_desired = 0.5 * (*prt_rho_dp_desired * L_Idb_desired + *prt_Id_desired * L_rho_dp_desired - *prt_rho_sp_desired * L_Isb_desired - *prt_Is_desired * L_rho_sp_desired);
        // L_Phi
        L_Phi_desired = 2*L_phi_desired; 
        // L_I_pol
        L_I_pol_b_desired.row(cnt) = L_Ob_desired + cos(2*this->phi_pol_ - *prt_phi_desired)*L_Ab_desired + *prt_A_desired*sin(2*this->phi_pol_ - *prt_phi_desired)*L_Phi_desired;
        // 更新       
        if(++col >= this->resolution_x_)
        {
            col = 0;
            ++row;
        }
        prt_phi++;
        prt_theta++;
        prt_d_rho_sp_theta_desired++;
        prt_d_rho_dp_theta_desired++;
        prt_ud_desired++;
        prt_us_desired++;
        prt_rho_dp_desired++;
        prt_rho_sp_desired++;
        prt_Id_desired++;
        prt_Is_desired++;
        prt_phi_desired++;
        prt_A_desired++;
    }
    // 计算 L_I_pol_b_real_desired 
    this->L_I_pol_b_real_desired_ = L_I_pol_b_desired - L_I_desired;       
}

// 计算当前图像交互矩阵 
// the light source mounted on the camera
void Polarimetric_Visual_Servoing::get_interaction_matrix_current_b()
{
    // 计算 L_I_current 
    Mat L_I_current = Mat::zeros(this->resolution_y_ * this->resolution_x_, 6, CV_64FC1);
    Mat P_current = get_P_current();
    Mat P_current_x = P_current.row(0) / P_current.row(2);
    Mat P_current_y = P_current.row(1) / P_current.row(2);
    Mat P_current_z_inv = 1.0 / P_current.row(2);

    Mat I;
    switch ((int)this->phi_pol_) 
    {
        case 0:
            I = this->image_I_0_current_;
            break;
        case 45:
            I = this->image_I_45_current_;
            break;
        case 90:
            I = this->image_I_90_current_;
            break;
        case 135:
            I = this->image_I_135_current_;
            break;
        default:
            I = this->image_I_0_current_;
            break; 
    }
    pair<Mat,Mat> grad = gradient(I, 1.0, 1.0);
    Mat I_u_current= Mat::zeros(6, this->resolution_y_ * this->resolution_x_, CV_64FC1);
    Mat I_v_current= Mat::zeros(6, this->resolution_y_ * this->resolution_x_, CV_64FC1);
    grad.first.reshape(0,1).copyTo(I_u_current.row(0));
    I_u_current.row(0).copyTo(I_u_current.row(1));
    I_u_current.row(0).copyTo(I_u_current.row(2));
    I_u_current.row(0).copyTo(I_u_current.row(3));
    I_u_current.row(0).copyTo(I_u_current.row(4));
    I_u_current.row(0).copyTo(I_u_current.row(5));
    grad.second.reshape(0,1).copyTo(I_v_current.row(0));
    I_v_current.row(0).copyTo(I_v_current.row(1));
    I_v_current.row(0).copyTo(I_v_current.row(2));
    I_v_current.row(0).copyTo(I_v_current.row(3));
    I_v_current.row(0).copyTo(I_v_current.row(4));
    I_v_current.row(0).copyTo(I_v_current.row(5));

    Mat L_kappa = get_L_kappa(this->camera_intrinsic_);
    double ku = L_kappa.at<double>(0, 0);
    double kv = L_kappa.at<double>(1, 1);

    Mat temp = Mat::zeros(1, this->resolution_y_ * this->resolution_x_, CV_64FC1);

    Mat L_u_current_x = Mat::zeros(6, this->resolution_y_ * this->resolution_x_, CV_64FC1);
    temp = ku*(-P_current_z_inv);                       temp.copyTo(L_u_current_x.row(0));
    temp = ku*(P_current_x.mul(P_current_z_inv));       temp.copyTo(L_u_current_x.row(2));
    temp = ku*(P_current_x.mul(P_current_y));           temp.copyTo(L_u_current_x.row(3));
    temp = ku*(-(1 + P_current_x.mul(P_current_x)));    temp.copyTo(L_u_current_x.row(4));
    temp = ku*(P_current_y);                            temp.copyTo(L_u_current_x.row(5));

    Mat L_u_current_y = Mat::zeros(6, this->resolution_y_ * this->resolution_x_, CV_64FC1);
    temp = kv*(-P_current_z_inv);                        temp.copyTo(L_u_current_y.row(1));
    temp = kv*(P_current_y.mul(P_current_z_inv));       temp.copyTo(L_u_current_y.row(2));
    temp = kv*(1 + P_current_y.mul(P_current_y));       temp.copyTo(L_u_current_y.row(3));
    temp = kv*(-P_current_x.mul(P_current_y));          temp.copyTo(L_u_current_y.row(4));
    temp = kv*(-P_current_x);                           temp.copyTo(L_u_current_y.row(5));  

    L_I_current = (I_u_current.mul(L_u_current_x) + I_v_current.mul(L_u_current_y)).t();
    // 计算 L_I_pol_b_current
    Mat L_n_current, L_theta_current, L_phi_current, L_rho_sp_current, L_rho_dp_current, 
        L_V_current, L_Sb_current, L_udb_current, L_usb_current, L_Idb_current, L_Isb_current, 
        L_Iu_current, L_Ob_current, L_Ab_current, L_Phi_current;
    
    double* prt_phi = (double*)(this->phi_current_.data);
    double* prt_theta = (double*)(this->theta_current_.data);
    double* prt_d_rho_sp_theta_current = (double*)(this->d_rho_sp_theta_current_.data);
    double* prt_d_rho_dp_theta_current = (double*)(this->d_rho_dp_theta_current_.data);
    double* prt_ud_current = (double*)(this->ud_current_.data);
    double* prt_us_current = (double*)(this->us_current_.data);
    double* prt_rho_dp_current = (double*)(this->rho_dp_current_.data);
    double* prt_rho_sp_current = (double*)(this->rho_sp_current_.data);
    double* prt_Id_current = (double *)(this->Id_current_.data);
    double* prt_Is_current = (double *)(this->Is_current_.data);
    double* prt_phi_current = (double *)(this->phi_current_.data);
    double* prt_A_current = (double *)(this->A_current_.data);

    Mat L_I_pol_b_current = Mat::zeros(this->resolution_y_ * this->resolution_x_, 6, CV_64FC1);
    int row = 0, col = 0;
    for(int cnt = 0; cnt < L_I_pol_b_current.rows; cnt++)
    {
        // L_n
        L_n_current = get_L_n(this->n_current_.col(cnt)); 
        // L_theta
        L_theta_current = get_L_theta(*prt_phi);
        // L_phi
        L_phi_current = get_L_phi(*prt_theta, *prt_phi);
        // L_rho_sp
        L_rho_sp_current = get_L_rho_sp(*prt_d_rho_sp_theta_current, L_theta_current);
        // L_rho_dp 
        L_rho_dp_current = get_L_rho_dp(*prt_d_rho_dp_theta_current, L_theta_current);
        // L_V
        L_V_current = get_L_V(P_current.col(cnt));
        // L_S
        L_Sb_current = get_L_Sb(P_current.col(cnt));
         // L_ud
        L_udb_current =  get_L_udb(this->n_current_.col(cnt), this->S_current_.col(cnt), L_n_current, L_Sb_current);
        // L_us 
        L_usb_current = get_L_usb(*prt_ud_current, this->n_current_.col(cnt), 
                            this->S_current_.col(cnt), this->V_.col(cnt), L_n_current, L_V_current, L_Sb_current, L_udb_current);
        // L_Id
        L_Idb_current = this->I_in_k_d_ * L_udb_current;
        // L_Is
        L_Isb_current = this->I_in_k_s_pi_ * this->k_ * pow(*prt_us_current, this->k_-1) * L_usb_current;
        // L_Iu
        L_Iu_current = Mat::zeros(1, 6, CV_64FC1);
        // L_O
        L_Ob_current = 0.5 * (L_Isb_current + L_Idb_current + L_Iu_current);
        // L_A
        L_Ab_current = 0.5 * (*prt_rho_dp_current * L_Idb_current + *prt_Id_current * L_rho_dp_current - *prt_rho_sp_current * L_Isb_current - *prt_Is_current * L_rho_sp_current);
        // L_Phi
        L_Phi_current = 2*L_phi_current; 
        // L_I_pol
        L_I_pol_b_current.row(cnt) = L_Ob_current + cos(2*this->phi_pol_ - *prt_phi_current)*L_Ab_current + *prt_A_current*sin(2*this->phi_pol_ - *prt_phi_current)*L_Phi_current;
        // 更新       
        if(++col >= this->resolution_x_)
        {
            col = 0;
            ++row;
        }
        prt_phi++;
        prt_theta++;
        prt_d_rho_sp_theta_current++;
        prt_d_rho_dp_theta_current++;
        prt_ud_current++;
        prt_us_current++;
        prt_rho_dp_current++;
        prt_rho_sp_current++;
        prt_Id_current++;
        prt_Is_current++;
        prt_phi_current++;
        prt_A_current++;
    }
    // 计算 L_I_pol_b_real_current 
    this->L_I_pol_b_real_current_ = L_I_pol_b_current - L_I_current;
}

// % 计算期望图像空间点坐标
Mat Polarimetric_Visual_Servoing::get_P_desried()
{
    int row = 1, col = 1;
    Mat temp_col = Mat::zeros(3, 1, CV_64FC1);

    for(int i = 0; i < V_.cols; i++)
    {
        temp_col = (Mat_<double>(3,1) << col, row, 1.0);
        temp_col.copyTo(this->col_row_reshape_.col(i)); 
        if(++col > this->resolution_x_)
        {
            col = 1;
            ++row;
        }
    }

    Mat image_Z = image_depth_desired_.reshape(0, 1);
    Mat temp = Mat::zeros(3, this->resolution_x_ * this->resolution_y_, CV_64FC1);
    image_Z.copyTo(temp.row(0));
    image_Z.copyTo(temp.row(1));
    image_Z.copyTo(temp.row(2));
    Mat P = (this->camera_intrinsic_inv_ * this->col_row_reshape_).mul(temp);

    return P;
}

// % 计算当前图像空间点坐标
Mat Polarimetric_Visual_Servoing::get_P_current()
{
    Mat image_Z = image_depth_current_.reshape(0, 1);
    Mat temp = Mat::zeros(3, this->resolution_x_ * this->resolution_y_, CV_64FC1);
    image_Z.copyTo(temp.row(0));
    image_Z.copyTo(temp.row(1));
    image_Z.copyTo(temp.row(2));
    Mat P = (this->camera_intrinsic_inv_ * this->col_row_reshape_).mul(temp);

    return P;
}

// 计算 theta
Mat Polarimetric_Visual_Servoing::get_theta(Mat K, Mat eta)
{
// cos_theta = sqrt(((eta_value.^2 - 1).^2 + 2*(eta_value.^2 + 1).*K + (-eta_value.^4 + 4*eta_value.^2 + 1).*K.^2 - 4*eta_value.^3.*K.*sqrt(1 - K.^2)) ./ ...
//     ((K + 1).*((eta_value.^2 - 1).^2 + K.*(eta_value.^4 + 6*eta_value.^2 + 1))));
    Mat eta_2 = eta.mul(eta);
    Mat eta_3 = eta_2.mul(eta);
    Mat eta_4 = eta_3.mul(eta); 
    Mat K_2 = K.mul(K);
    Mat temp_K = 1 - K_2;
    Mat temp_K_sqrt;
    sqrt(temp_K, temp_K_sqrt);
    Mat cos_theta_Num = (eta_2 - 1).mul(eta_2 - 1) + 2*(eta_2 + 1).mul(K) + (-eta_4 + 4*eta_2 + 1).mul(K_2) - 4*eta_3.mul(K).mul(temp_K_sqrt);
    Mat cos_theta_Den = (K + 1).mul((eta_2 - 1).mul(eta_2 - 1) + K.mul(eta_4 + 6*eta_2 + 1));
    Mat cos_theta_2 = cos_theta_Num / cos_theta_Den;
    Mat cos_theta;
    sqrt(cos_theta_2, cos_theta);
    Mat theta = cv_acos(cos_theta);
    return theta;
}


// 计算 L_n eq(51)
Mat Polarimetric_Visual_Servoing::get_L_n(Mat n)
{
    return (Mat_<double>(3,6) << 0.0, 0.0, 0.0, 0, -n.at<double>(2,0), n.at<double>(1,0), 
                                 0.0, 0.0, 0.0, n.at<double>(2,0), 0, -n.at<double>(0,0),
                                 0.0, 0.0, 0.0, -n.at<double>(1,0), n.at<double>(0,0), 0);
}

// 计算 L_theta eq(53)
Mat Polarimetric_Visual_Servoing::get_L_theta(double phi)
{
    return (Mat_<double>(1,6) << 0, 0, 0, -sin(phi), cos(phi), 0);
}

// 计算 L_phi eq(53)
Mat Polarimetric_Visual_Servoing::get_L_phi(double theta, double phi)
{
    return (Mat_<double>(1,6) << 0, 0, 0, -cos(phi)/tan(theta), -sin(phi)/tan(theta), -1);
}

// 计算 L_phi eq(55)
Mat Polarimetric_Visual_Servoing::get_L_rho_sp(double d_rho_sp_d_theta, Mat L_theta)
{
    return d_rho_sp_d_theta * L_theta;
}

// 计算 L_phi eq(55)
Mat Polarimetric_Visual_Servoing::get_L_rho_dp(double d_rho_dp_d_theta, Mat L_theta)
{
    return d_rho_dp_d_theta * L_theta;
}

// 计算 L_V eq(59)
Mat Polarimetric_Visual_Servoing::get_L_V(Mat P)
{
    Mat dV_dP = get_dV_dP(P);
    Mat L_P = get_L_P(P);
    return dV_dP * L_P; 
}

// 计算 L_Sa eq(60)
Mat Polarimetric_Visual_Servoing::get_L_Sa(Mat S)
{
    return (Mat_<double>(3,6) << 0.0, 0.0, 0.0, 0, -S.at<double>(2,0), S.at<double>(1,0), 
                                 0.0, 0.0, 0.0, S.at<double>(2,0), 0, -S.at<double>(0,0),
                                 0.0, 0.0, 0.0, -S.at<double>(1,0), S.at<double>(0,0), 0);
}

// 计算 L_Sa eq(61)
Mat Polarimetric_Visual_Servoing::get_L_Sb(Mat P)
{
    return get_L_V(P); 
}

// 计算 L_udb eq(42)
Mat Polarimetric_Visual_Servoing::get_L_udb(Mat n, Mat S, Mat L_n, Mat L_Sb)
{
    return n.t() * L_Sb + S.t() * L_n; 
}

// 计算 L_usa eq(45)
Mat Polarimetric_Visual_Servoing::get_L_usa(double ud, Mat n, Mat S, Mat V, Mat L_n_desired, Mat L_V_desired, Mat L_Sa_desired)
{
    return (2*ud*n - S).t() * L_V_desired + V.t()*(2*ud*L_n_desired - L_Sa_desired); 
}

// 计算 L_usa eq(47)
Mat Polarimetric_Visual_Servoing::get_L_usb(double ud, Mat n, Mat S, Mat V, Mat L_n_desired, Mat L_V_desired, Mat L_Sb_desired, Mat L_udb_desired)
{
    return (2*ud*n - S).t() * L_V_desired + V.t()*(2*n*L_udb_desired + 2*ud*L_n_desired - L_Sb_desired);
}



// 计算 dV_dP eq(59)
Mat Polarimetric_Visual_Servoing::get_dV_dP(Mat P)
{
    Mat dV_dP = P * P.t() / pow(norm(P),3) - Mat::eye(3, 3, CV_64FC1) / (norm(P));
    return dV_dP;
}

// 计算 L_P eq(56)
Mat Polarimetric_Visual_Servoing::get_L_P(Mat P)
{
    return (Mat_<double>(3,6) << -1.0, 0.0, 0.0, 0, -P.at<double>(2,0), P.at<double>(1,0), 
                                 0.0, -1.0, 0.0, P.at<double>(2,0), 0, -P.at<double>(0,0),
                                 0.0, 0.0, -1.0, -P.at<double>(1,0), P.at<double>(0,0), 0);
}


// 计算 T_s T_p
void Polarimetric_Visual_Servoing::get_T_perpendicular_parallel(Mat theta, Mat eta, Mat& T_s, Mat& T_p)
{
//  T_s = (4*cos(theta_value).*sqrt(eta_value.^2 - sin(theta_value).^2)) ./ ...
//        (1 + eta_value.^2 - 2*sin(theta_value).^2 + 2*cos(theta_value).*sqrt(eta_value.^2 - sin(theta_value).^2));
// T_p = (4*cos(theta_value).*sqrt(eta_value.^2 - sin(theta_value).^2)) ./ ...
//     (1 + eta_value.^2 - (eta_value.^2 + 1/eta_value.^2).*sin(theta_value).^2 + 2*cos(theta_value).*sqrt(eta_value.^2 - sin(theta_value).^2));
    Mat cv_sin_theta = cv_sin(theta);
    Mat cv_cos_theta = cv_cos(theta);
    Mat cv_sin_theta_2 = cv_sin_theta.mul(cv_sin_theta);
    Mat eta_2 = eta.mul(eta);
    Mat temp = eta_2 - cv_sin_theta_2;
    Mat temp_sqrt;
    sqrt(temp, temp_sqrt);
    Mat cos_theta_temp = 2*cv_cos_theta.mul(temp_sqrt);

    Mat Numerator = 4*cv_cos(theta).mul(temp_sqrt);
    Mat Denominator_T_s = 1 + eta_2 - 2*cv_sin_theta_2 + cos_theta_temp;
    T_s = Numerator / Denominator_T_s;
    Mat Denominator_T_p = 1 + eta_2 - (eta_2 + 1/eta_2).mul(cv_sin_theta_2) + cos_theta_temp;
    T_p = Numerator / Denominator_T_p;
}

pair<Mat,Mat> Polarimetric_Visual_Servoing::gradient(Mat & img, double spaceX, double spaceY) {

    Mat gradY = gradientY(img,spaceY);
    Mat gradX = gradientX(img,spaceX);
    pair<Mat,Mat> retValue(gradX,gradY);
    return retValue;
}


Mat Polarimetric_Visual_Servoing::gradientX(Mat & mat, double spacing) {

    Mat grad = Mat::zeros(mat.rows,mat.cols,CV_64FC1);

    /*  last row */
    int maxCols = mat.cols;
    int maxRows = mat.rows;

    /* get gradients in each border */
    /* first row */
    Mat col = (-mat.col(0) + mat.col(1))/(double)spacing;
    col.copyTo(grad(Rect(0,0,1,maxRows)));

    col = (-mat.col(maxCols-2) + mat.col(maxCols-1))/(double)spacing;
    col.copyTo(grad(Rect(maxCols-1,0,1,maxRows)));

    /* centered elements */
    Mat centeredMat = mat(Rect(0,0,maxCols-2,maxRows));
    Mat offsetMat = mat(Rect(2,0,maxCols-2,maxRows));
    Mat resultCenteredMat = (-centeredMat + offsetMat)/(((double)spacing)*2.0);

    resultCenteredMat.copyTo(grad(Rect(1,0,maxCols-2, maxRows)));
    return grad;
}


Mat Polarimetric_Visual_Servoing::gradientY(Mat & mat, double spacing) {
    Mat grad = Mat::zeros(mat.rows,mat.cols,CV_64FC1);

    /*  last row */
    const int maxCols = mat.cols;
    const int maxRows = mat.rows;

    /* get gradients in each border */
    /* first row */
    Mat row = (-mat.row(0) + mat.row(1))/(double)spacing;
    row.copyTo(grad(Rect(0,0,maxCols,1)));

    row = (-mat.row(maxRows-2) + mat.row(maxRows-1))/(double)spacing;
    row.copyTo(grad(Rect(0,maxRows-1,maxCols,1)));

    /* centered elements */
    Mat centeredMat = mat(Rect(0,0,maxCols,maxRows-2));
    Mat offsetMat = mat(Rect(0,2,maxCols,maxRows-2));
    Mat resultCenteredMat = (-centeredMat + offsetMat)/(((double)spacing)*2.0);

    resultCenteredMat.copyTo(grad(Rect(0,1,maxCols, maxRows-2)));
    return grad;
}



// 计算L_kappa
Mat Polarimetric_Visual_Servoing::get_L_kappa(Mat& camera_intrinsic)
{
    Mat L_kappa = (Mat_<double>(2,2) << camera_intrinsic.at<double>(0,0), 0, 0, camera_intrinsic.at<double>(1,1));
    return L_kappa;
}

// 计算偏振参数
void Polarimetric_Visual_Servoing::get_O_A_Phi(Mat I_0, Mat I_45, Mat I_90, Mat I_135, Mat& O, Mat& A, Mat& Phi)
{
    O = (I_0 + I_45 + I_90 + I_135) / 4;

    Mat err_I0 = I_0 - O;
    Mat err_I45 = I_45 - O;
    Mat err_I_90 = I_90 - O;
    Mat err_I135 = I_135 - O;
    sqrt((err_I0.mul(err_I0) + err_I45.mul(err_I45) + err_I_90.mul(err_I_90) + err_I135.mul(err_I135)) / 2, A);
    
    Phi = 0.5 * cv_atan2(I_45 - I_135, I_0 - I_90);

    O = O.reshape(0, 1);
    A = A.reshape(0, 1);
    Phi = Phi.reshape(0, 1);
}

void Polarimetric_Visual_Servoing::get_O_A_Phi_desired()
{
    get_O_A_Phi(this->image_I_0_desired_, this->image_I_45_desired_, this->image_I_90_desired_, this->image_I_135_desired_, this->O_desired_, this->A_desired_, this->Phi_desired_);
}
        
void Polarimetric_Visual_Servoing::get_O_A_Phi_current()
{
    get_O_A_Phi(this->image_I_0_current_, this->image_I_45_current_, this->image_I_90_current_, this->image_I_135_current_, this->O_current_, this->A_current_, this->Phi_current_);
}



Mat Polarimetric_Visual_Servoing::cv_cos(Mat a)
{
    Mat dst = Mat::zeros(a.size(), a.type());
    int rows = a.rows;
    int cols = a.cols;
    for ( int row = 0; row < rows; row++)
    {
        double* current_a = a.ptr<double>(row);
        double* current_dst = dst.ptr<double>(row);
        for (int col = 0; col< cols; col++)
        {
            current_dst[col] = std::cos(current_a[col]);
        }
    }
    return dst;
}

Mat Polarimetric_Visual_Servoing::cv_sin(Mat a)
{
    Mat dst = Mat::zeros(a.size(), a.type());
    int rows = a.rows;
    int cols = a.cols;
    for ( int row = 0; row < rows; row++)
    {
        double* current_a = a.ptr<double>(row);
        double* current_dst = dst.ptr<double>(row);
        for (int col = 0; col< cols; col++)
        {
            current_dst[col] = std::sin(current_a[col]);
        }
    }
    return dst;
}


Mat Polarimetric_Visual_Servoing::cv_acos(Mat a)
{
    Mat dst = Mat::zeros(a.size(), a.type());
    int rows = a.rows;
    int cols = a.cols;
    for ( int row = 0; row < rows; row++)
    {
        double* current_a = a.ptr<double>(row);
        double* current_dst = dst.ptr<double>(row);
        for (int col = 0; col< cols; col++)
        {
            current_dst[col] = std::acos(current_a[col]);
        }
    }
    return dst;
}

Mat Polarimetric_Visual_Servoing::cv_asin(Mat a)
{
    Mat dst = Mat::zeros(a.size(), a.type());
    int rows = a.rows;
    int cols = a.cols;
    for ( int row = 0; row < rows; row++)
    {
        double* current_a = a.ptr<double>(row);
        double* current_dst = dst.ptr<double>(row);
        for (int col = 0; col< cols; col++)
        {
            current_dst[col] = std::asin(current_a[col]);
        }
    }
    return dst;
}

Mat Polarimetric_Visual_Servoing::cv_atan2(Mat a, Mat b)
{
    Mat dst = Mat::zeros(a.size(), a.type());
    int rows = a.rows;
    int cols = a.cols;
    for ( int row = 0; row < rows; row++)
    {
        double* current_a = a.ptr<double>(row);
        double* current_b = b.ptr<double>(row);
        double* current_dst = dst.ptr<double>(row);
        for (int col = 0; col< cols; col++)
        {
            current_dst[col] = std::atan2(current_a[col], current_b[col]);
        }
    }

    return dst;    
}

// 计算so3
Mat Polarimetric_Visual_Servoing::VecToso3(Mat omg)
{
    return (Mat_<double>(3,3) << 0, -omg.at<double>(2,0), omg.at<double>(1,0), omg.at<double>(2,0), 0, -omg.at<double>(0,0), -omg.at<double>(1,0), omg.at<double>(0,0), 0);
}

// 计算相机速度
Mat Polarimetric_Visual_Servoing::get_camera_velocity()
{
	// 计算相机速度
    Mat L_e_inv;
	this->iteration_num_++;    
    get_feature_error_interaction_matrix(); 
    invert(this->L_e_, L_e_inv, DECOMP_SVD);  
    this->camera_velocity_ = -this->lambda_ * L_e_inv * this->error_s_;
    return this->camera_velocity_;
}

// 判断是否伺服成功
bool Polarimetric_Visual_Servoing::is_success()
{
	Mat error_ave = this->error_s_.t() * this->error_s_ / (this->error_s_.rows*this->error_s_.cols);

	if(error_ave.at<double>(0,0) < this->epsilon_)
	{
		cout << "Visual Servoing Success" << endl;
		return true;
	}
	else
	{
		return false;
	}
}

// 设置相机内参
void Polarimetric_Visual_Servoing::set_camera_intrinsic(Mat& camera_intrinsic)
{
    camera_intrinsic.copyTo(this->camera_intrinsic_);
    this->camera_intrinsic_inv_ = this->camera_intrinsic_.inv();
}

// 设置期望灰度图像
void Polarimetric_Visual_Servoing::set_image_gray_desired(Mat& image_I_0_desired, Mat& image_I_45_desired, Mat& image_I_90_desired, Mat& image_I_135_desired)
{
    image_I_0_desired.copyTo(this->image_I_0_desired_); 
    image_I_45_desired.copyTo(this->image_I_45_desired_); 
    image_I_90_desired.copyTo(this->image_I_90_desired_); 
    image_I_135_desired.copyTo(this->image_I_135_desired_); 
}

// 设置当前灰度图像
void Polarimetric_Visual_Servoing::set_image_gray_current(Mat& image_I_0_current, Mat& image_I_45_current, Mat& image_I_90_current, Mat& image_I_135_current)

{
    image_I_0_current.copyTo(this->image_I_0_current_);
    image_I_45_current.copyTo(this->image_I_45_current_);
    image_I_90_current.copyTo(this->image_I_90_current_);
    image_I_135_current.copyTo(this->image_I_135_current_);
}

// 设置初始图像
void Polarimetric_Visual_Servoing::set_image_gray_initial(Mat& image_I_0_initial, Mat& image_I_45_initial, Mat& image_I_90_initial, Mat& image_I_135_initial)
{
    image_I_0_initial.copyTo(this->image_I_0_initial_);
    image_I_45_initial.copyTo(this->image_I_45_initial_);
    image_I_90_initial.copyTo(this->image_I_90_initial_);
    image_I_135_initial.copyTo(this->image_I_135_initial_);
}

// 设置偏振图像
void Polarimetric_Visual_Servoing::set_image_polar_current(Mat& polar_O_current, Mat& polar_A_current, Mat& polar_Phi_current)
{
    polar_O_current.reshape(0,1).copyTo(this->O_current_);
    polar_A_current.reshape(0,1).copyTo(this->A_current_);
    polar_Phi_current.reshape(0,1).copyTo(this->Phi_current_);
    this->image_I_0_current_ = polar_O_current + polar_A_current * cv_cos(polar_Phi_current);
}

void Polarimetric_Visual_Servoing::set_image_polar_desired(Mat& polar_O_desired, Mat& polar_A_desired, Mat& polar_Phi_desired)
{
    polar_O_desired.reshape(0,1).copyTo(this->O_desired_);
    polar_A_desired.reshape(0,1).copyTo(this->A_desired_);
    polar_Phi_desired.reshape(0,1).copyTo(this->Phi_desired_);
    this->image_I_0_desired_ = polar_O_desired + polar_A_desired * cv_cos(-polar_Phi_desired);
    this->image_I_45_desired_ = polar_O_desired + polar_A_desired * cv_cos(CV_PI/2.0 - polar_Phi_desired);
    this->image_I_90_desired_ = polar_O_desired + polar_A_desired * cv_cos(CV_PI - polar_Phi_desired);
    this->image_I_135_desired_ = polar_O_desired + polar_A_desired * cv_cos(-CV_PI/2.0 - polar_Phi_desired);  
}

void Polarimetric_Visual_Servoing::set_image_polar_initial(Mat& image_O_initial, Mat& image_A_initial, Mat& image_Phi_initial)
{
    this->image_I_0_initial_ = image_O_initial + image_A_initial * cv_cos(-image_Phi_initial);
    this->image_I_45_initial_ = image_O_initial + image_A_initial * cv_cos(CV_PI/2.0 - image_Phi_initial);
    this->image_I_90_initial_ = image_O_initial + image_A_initial * cv_cos(CV_PI - image_Phi_initial);
    this->image_I_135_initial_ = image_O_initial + image_A_initial * cv_cos(-CV_PI/2.0 - image_Phi_initial);  
}

// // 设置期望深度图像
void Polarimetric_Visual_Servoing::set_image_depth_desired(Mat& image_depth_desired)
{
    image_depth_desired.copyTo(this->image_depth_desired_);
}

// 设置当前深度图像
void Polarimetric_Visual_Servoing::set_image_depth_current(Mat& image_depth_current)
{
    image_depth_current.copyTo(this->image_depth_current_);
}

// // 保存期望位姿
void Polarimetric_Visual_Servoing::set_pose_desired(Mat& pose_desired)
{
	pose_desired.copyTo(this->pose_desired_);
}


// 保存图像数据
void Polarimetric_Visual_Servoing::save_pose_desired()
{
    this->pose_desired_.copyTo(this->data_pvs.pose_desired_);   
}

// 保存图像数据
void Polarimetric_Visual_Servoing::save_data_image()
{
    save_data_image_gray_desired();
    save_data_image_gray_initial();
}

// 保存期望图像
void Polarimetric_Visual_Servoing::save_data_image_gray_desired()
{
    this->image_I_0_desired_.copyTo(this->data_pvs.image_I_0_desired_);
    this->image_I_45_desired_.copyTo(this->data_pvs.image_I_45_desired_);
    this->image_I_90_desired_.copyTo(this->data_pvs.image_I_90_desired_);
    this->image_I_135_desired_.copyTo(this->data_pvs.image_I_135_desired_);
}

// 保存初始图像
void Polarimetric_Visual_Servoing::save_data_image_gray_initial()
{
    this->image_I_0_initial_.copyTo(this->data_pvs.image_I_0_initial_);
    this->image_I_45_initial_.copyTo(this->data_pvs.image_I_45_initial_);
    this->image_I_90_initial_.copyTo(this->data_pvs.image_I_90_initial_);
    this->image_I_135_initial_.copyTo(this->data_pvs.image_I_135_initial_);
}

// 保存相机速度
void Polarimetric_Visual_Servoing::save_data_camera_velocity()
{
    this->data_pvs.velocity_.push_back(this->camera_velocity_.t());
}

// // 保存特征误差
// void Polarimetric_Visual_Servoing::save_data_error_feature()
// {
//     this->data_pvs.error_feature_.push_back(this->error_s_);
// }

// 保存相机位姿
void Polarimetric_Visual_Servoing::save_data_camera_pose(Mat& pose)
{
    this->data_pvs.pose_.push_back(pose);
}

// 保存所有数据
void Polarimetric_Visual_Servoing::save_data(Mat pose)
{
    save_data_camera_velocity();
    save_data_camera_pose(pose);
    save_data_error_feature(); 
	save_data_vs_time();
}

// 记录时间
void Polarimetric_Visual_Servoing::save_data_vs_time()
{
	
	if (this->iteration_num_ == 1)
	{
		this->start_VS_time_ = clock();
		this->data_pvs.time_vs_.push_back(0.0);
	}
	else
	{
		this->data_pvs.time_vs_.push_back((double)(clock() - this->start_VS_time_) / CLOCKS_PER_SEC);
	}
}


// 将数据保存在文件中
void Polarimetric_Visual_Servoing::write_data()
{
    string file_name = get_save_file_name();
    string location = "/home/cyh/Work/visual_servoing_ws/src/visual_servoing/image_process/resource/data/";
    // 保存图像
    string saveImage_desired_I0 = location + file_name + "_desired_image_I0.png";
    string saveImage_desired_I45 = location + file_name + "_desired_image_I45.png";
    string saveImage_desired_I90 = location + file_name + "_desired_image_I90.png";
    string saveImage_desired_I135 = location + file_name + "_desired_image_I135.png";
    string saveImage_initial_I0 = location + file_name + "_initial_image_I0.png";
    string saveImage_initial_I45 = location + file_name + "_initial_image_I45.png";
    string saveImage_initial_I90 = location + file_name + "_initial_image_I90.png";
    string saveImage_initial_I135 = location + file_name + "_initial_image_I135.png";
    imwrite(saveImage_desired_I0, this->data_pvs.image_I_0_desired_*255);
    imwrite(saveImage_desired_I45, this->data_pvs.image_I_45_desired_*255);
    imwrite(saveImage_desired_I90, this->data_pvs.image_I_90_desired_*255);
    imwrite(saveImage_desired_I135, this->data_pvs.image_I_135_desired_*255);
    imwrite(saveImage_initial_I0, this->data_pvs.image_I_0_initial_*255);
    imwrite(saveImage_initial_I45, this->data_pvs.image_I_45_initial_*255);
    imwrite(saveImage_initial_I90, this->data_pvs.image_I_90_initial_*255);
    imwrite(saveImage_initial_I135, this->data_pvs.image_I_135_initial_*255);
    // 保存数据
    ofstream oFile;
	string excel_name = location + file_name + "_data.xls";
    oFile.open(excel_name, ios::out|ios::trunc);
    write_visual_servoing_data(oFile);
    // 关闭文件
    oFile.close();
}

// 写入基本视觉伺服数据到文件
void Polarimetric_Visual_Servoing::write_visual_servoing_data(ofstream& oFile)
{
    oFile << "camera velocity" << endl;
    write_to_excel(this->data_pvs.velocity_, oFile);
    oFile << "camera pose" << endl;
    write_to_excel(this->data_pvs.pose_, oFile);
    oFile << "camera desired pose" << endl;
    write_to_excel(this->data_pvs.pose_desired_, oFile);   
    oFile << "error feature" << endl;
    write_to_excel(this->data_pvs.error_feature_, oFile);
    oFile << "visual servoing time" << endl;
    write_to_excel(this->data_pvs.time_vs_, oFile);	
}

// 存储数据文件命名
string Polarimetric_Visual_Servoing::get_save_file_name()
{
    return get_date_time() + "_" + get_method_name();
}

// 视觉伺服方法名字
string Polarimetric_Visual_Servoing::get_method_name()
{
    return "Polarimetric_Visual_Servoing";
}

// 获取当前计算机时间
string Polarimetric_Visual_Servoing::get_date_time()
{
	auto to_string = [](const std::chrono::system_clock::time_point& t)->std::string
	{
		auto as_time_t = std::chrono::system_clock::to_time_t(t);
		struct tm tm;
#if defined(WIN32) || defined(_WINDLL)
		localtime_s(&tm, &as_time_t);  //win api，线程安全，而std::localtime线程不安全
#else
		localtime_r(&as_time_t, &tm);//linux api，线程安全
#endif

		std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(t.time_since_epoch());
		char buf[128];
		snprintf(buf, sizeof(buf), "%04d_%02d_%02d_%02d_%02d_%02d",
			tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
		return buf;
	};

	std::chrono::system_clock::time_point t = std::chrono::system_clock::now();
	return to_string(t);
}

void Polarimetric_Visual_Servoing::write_to_excel(Mat data, ofstream& oFile)
{
		int channels = data.channels();            //获取图像channel  
		int nrows = data.rows;                     //矩阵的行数  
		int ncols = data.cols*channels;            //矩阵的总列数=列数*channel分量数  
 
		//循环用变量
		int i = 0;
		int j = 0;
 
		if (data.depth() == CV_8U)//uchar
		{
			for (i = 0; i<nrows; i++)
			{
				for (j = 0; j<ncols; j++)
				{
					int tmpVal = (int)data.ptr<uchar>(i)[j];
					oFile << tmpVal << '\t';
				}
				oFile << endl;
			}
		}
		else if (data.depth() == CV_16S)//short
		{
			for (i = 0; i<nrows; i++)
			{
				for (j = 0; j<ncols; j++)
				{
					oFile << (short)data.ptr<short>(i)[j] << '\t';
				}
				oFile << endl;
			}
		}
		else if (data.depth() == CV_16U)//unsigned short
		{
			for (i = 0; i<nrows; i++)
			{
				for (j = 0; j<ncols; j++)
				{
					oFile << (unsigned short)data.ptr<unsigned short>(i)[j] << '\t';
				}
				oFile << endl;
			}
		}
		else if (data.depth() == CV_32S)//int 
		{
			for (i = 0; i<nrows; i++)
			{
				for (j = 0; j<ncols; j++)
				{
					oFile << (int)data.ptr<int>(i)[j] << '\t';
				}
				oFile << endl;
			}
		}
		else if (data.depth() == CV_32F)//double
		{
			for (i = 0; i<nrows; i++)
			{
				for (j = 0; j<ncols; j++)
				{
					oFile << (double)data.ptr<double>(i)[j] << '\t';
				}
				oFile << endl;
			}
		}
		else//CV_64F double
		{
			for (i = 0; i < nrows; i++)
			{
				for (j = 0; j < ncols; j++)
				{
					oFile << (double)data.ptr<double>(i)[j] << '\t';
				}
				oFile << endl;
			}
		}
}

void Polarimetric_Visual_Servoing::get_feature_error_interaction_matrix()
{
    Mat temp;

    get_feature_error();

    get_interaction_matrix_current();

    if(this->flag_a_or_b){
        this->L_e_  = 0.5 * (this->L_I_pol_a_real_current_ + this->L_I_pol_a_real_desired_);
    }
    else{
        this->L_e_  = 0.5 * (this->L_I_pol_b_real_current_ + this->L_I_pol_b_real_desired_);
    }
}

void Polarimetric_Visual_Servoing::get_feature_error()
{
    switch ((int)this->phi_pol_) 
    {
        case 0:
            this->error_s_ = this->image_I_0_current_ - this->image_I_0_desired_;
            break;
        case 45:
            this->error_s_ = this->image_I_45_current_ - this->image_I_45_desired_;
            break;
        case 90:
            this->error_s_ = this->image_I_90_current_ - this->image_I_90_desired_;
            break;
        case 135:
            this->error_s_ = this->image_I_135_current_ - this->image_I_135_desired_;
            break;
        default:
            this->error_s_ = this->image_I_0_current_ - this->image_I_0_desired_;
            break; 
    }   
    this->error_s_ = this->error_s_.reshape(0, this->resolution_x_ * this->resolution_y_); 
}

void Polarimetric_Visual_Servoing::save_data_error_feature()
{
    Mat error_ave = (this->error_s_.t() * this->error_s_) / (this->error_s_.rows * this->error_s_.cols);
    this->data_pvs.error_feature_.push_back(error_ave);
}