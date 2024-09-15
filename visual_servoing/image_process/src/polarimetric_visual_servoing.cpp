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
	this->iteration_num_ = 0;

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
    this->Id_desired_ = Mat::zeros(1, this->resolution_y_ * this->resolution_x_, CV_64FC1); 
    this->Iu_ = Mat::zeros(1, this->resolution_y_ * this->resolution_x_, CV_64FC1);  

    this->rho_desired_ = Mat::zeros(1, this->resolution_y_ * this->resolution_x_, CV_64FC1); 
    this->theta_desired_ = Mat::zeros(1, this->resolution_y_ * this->resolution_x_, CV_64FC1); 
    this->phi_desired_ = Mat::zeros(1, this->resolution_y_ * this->resolution_x_,  CV_64FC1); 

    this->rho_dp_desired_ = Mat::zeros(1, this->resolution_y_ * this->resolution_x_,  CV_64FC1); 
    this->rho_sp_desired_ = Mat::zeros(1, this->resolution_y_ * this->resolution_x_,  CV_64FC1);   
    this->rho_dp_current_ = Mat::zeros(1, this->resolution_y_ * this->resolution_x_,  CV_64FC1); 
    this->rho_sp_current_ = Mat::zeros(1, this->resolution_y_ * this->resolution_x_,  CV_64FC1); 

    // 多维数组转为行存储
    this->V_ = Mat::zeros(3, this->resolution_y_ * this->resolution_x_, CV_64FC1);
    this->n_desired_ = Mat::zeros(3, this->resolution_y_ * this->resolution_x_, CV_64FC1);
    this->n_current_ = Mat::zeros(3, this->resolution_y_ * this->resolution_x_, CV_64FC1);
    this->S_desired_ = Mat::zeros(3, this->resolution_y_ * this->resolution_x_, CV_64FC1);
    this->S_current_ = Mat::zeros(3, this->resolution_y_ * this->resolution_x_, CV_64FC1);
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
    this->eta_ = eta * Mat::ones(this->resolution_y_, this->resolution_x_, CV_64FC1); 
    this->phi_pol_ = phi_pol; 
    this->k_ = k;
}

// 初始化Phong模型 V_, n_desired_, n_current_, S_desired_, S_current_
// PVS.V_.col(row * PVS.resolution_x_ + col) << endl;
void Polarimetric_Visual_Servoing::get_Phong_model_init()
{
    double row = 1, col = 1;

    for(int i = 0; i < V_.cols; i++)
    {
        this->V_.col(i) = this->camera_intrinsic_inv_ * cv::Vec3d(col, row, 1.0);
        this->V_.col(i) = this->V_.col(i) / norm(this->V_.col(i));
        this->n_desired_.col(i)  = cv::Vec3d(0.0, 0.0, -1.0);
        this->n_current_.col(i)  = cv::Vec3d(0.0, 0.0, -1.0);
        this->S_desired_.col(i)  = cv::Vec3d(0.0, 0.0, -1.0);
        this->S_current_.col(i)  = cv::Vec3d(0.0, 0.0, -1.0);
        if(++col > this->resolution_x_)
        {
            col = 1;
            ++row;
        }
    }
}

void Polarimetric_Visual_Servoing::get_polar_data_desired()
{
    cout << "cyh_a" << endl;
    get_Phong_us_ud_desired();

cout << "cyh_b" << endl;
    get_I_in_k_s_pi_I_in_k_d_I_a_k_a();

cout << "cyh_c" << endl;
    get_Is_Id_Iu_desired();

cout << "cyh_d" << endl;
    // get_rho_theta_phi_desired();

cout << "cyh_e" << endl;
    get_n_desired();    
}

// 计算Phong模型 us_desired ud_desired
void Polarimetric_Visual_Servoing::get_Phong_us_ud_desired()
{
    int cnt = 0;
    for (int row = 0; row < this->us_desired_.rows; ++row) 
    {
        double *us_d = (double *)(this->us_desired_.data + row*this->us_desired_.step[0]); 
        double *ud_d = (double *)(this->ud_desired_.data + row*this->ud_desired_.step[0]); 

        for (int col = 0; col < us_desired_.cols; ++col) 
        {          
            ud_d[col] = this->n_desired_.col(cnt).dot(this->S_desired_.col(cnt));
            us_d[col] = this->V_.col(cnt).dot(2*ud_d[col]*this->n_desired_.col(cnt) - this->S_desired_.col(cnt));
            cnt++;
        }
    }
}

// 计算 I_in_k_s_pi I_in_k_d I_a_k_a
void Polarimetric_Visual_Servoing::get_I_in_k_s_pi_I_in_k_d_I_a_k_a()
{
    Mat US; Mat UD; Mat Jcaob_pinv;
    cv::pow(this->us_desired_, this->k_, US);
    UD = this->us_desired_;
    Mat Jacob = Mat::ones(this->resolution_y_ * this->resolution_x_, 3, CV_64FC1);
    Jacob.col(0) = US.reshape(0,1).t();
    Jacob.col(1) = UD.reshape(0,1).t();
    Mat sov = 2*this->O_desired_.reshape(0,1).t();
    invert(Jacob, Jcaob_pinv, DECOMP_SVD);
    Mat temp = Jcaob_pinv * sov;
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
        K_desired = 2*this->rho_desired_.mul(this->O_desired_) / (this->Id_desired_ - (this->Is_desired_ / (2 / (T_s_desired + T_p_desired) - 1)));
        this->theta_desired_ = get_theta(K_desired, this->eta_);
        if(norm(this->theta_desired_ - theta_desired_old) < 1e-6){
            break;
        }   
        else{
            theta_desired_old = this->theta_desired_;
        }     
    }
    // 计算 azimuth angle
    Mat phi_desired = this->Phi_desired_ / 2;
    pair<Mat,Mat> grad = gradient(this->image_depth_desired_, 1.0, 1.0);

    Mat flag_desired = grad.first.mul(cv_cos(phi_desired)) + grad.second.mul(cv_sin(phi_desired));
    Mat id_desired = flag_desired < 0;
    Mat idx;
	cv::findNonZero(id_desired, idx);    
    for(int i = 0; i < idx.cols * idx.rows; i++)
    {
        phi_desired.at<double>(idx.at<Point>(i).y, idx.at<Point>(i).x) = phi_desired.at<double>(idx.at<Point>(i).y, idx.at<Point>(i).x) + CV_PI;
    }
}

// 计算 n
void Polarimetric_Visual_Servoing::get_n_desired()
{

    Mat nx_desired = cv_cos(this->phi_desired_).mul(cv_sin(this->theta_desired_)); 
    Mat ny_desired = cv_sin(this->phi_desired_).mul(cv_sin(this->theta_desired_));
    Mat nz_desired = -cv_cos(this->theta_desired_);
    this->n_desired_.row(0) = nx_desired.reshape(0, 1);
    this->n_desired_.row(1) = ny_desired.reshape(0, 1);
    this->n_desired_.row(2) = nz_desired.reshape(0, 1);
}

// 计算 rho_dp rho_sp
void Polarimetric_Visual_Servoing::get_rho_dp_sp(Mat theta, Mat eta, Mat& rho_dp, Mat& rho_sp)
{
// rho_dp = ((eta_value - 1/eta_value).^2 .* sin(theta_value).^2) ./ ...
//     (2+2*eta_value.^2 - (eta_value + 1/eta_value).^2.*sin(theta_value).^2 + 4*cos(theta_value).*sqrt(eta_value.^2 - sin(theta_value).^2));
// rho_sp = (2*sin(theta_value).^2.*cos(theta_value).*sqrt(eta_value.^2 - sin(theta_value).^2)) ./ ...
//     (eta_value.^2 - sin(theta_value).^2 - eta_value.^2.*sin(theta_value).^2 + 2*sin(theta_value).^4);
    Mat sin_theta = cv_sin(theta);
    Mat cos_theta = cv_cos(theta);
    Mat sin_theta_2 = sin_theta.mul(sin_theta);
    Mat eta_2 = eta.mul(eta);
    Mat eta_2_inv = 1/eta_2_inv;
    Mat temp_2 = eta_2 - sin_theta_2;
    Mat temp_sqrt;
    sqrt(temp_2, temp_sqrt);
    
    Mat rho_sp_num = 2*sin_theta_2.mul(cos_theta).mul(temp_sqrt);
    Mat rho_sp_dec = eta_2 - sin_theta_2 - eta_2.mul(sin_theta_2) + 2*sin_theta_2.mul(sin_theta_2);
    
    Mat rho_dp_num = (eta_2 + eta_2_inv - 2).mul(sin_theta_2);
    Mat rho_dp_dec = 2 + 2*eta_2 - (eta_2 + eta_2_inv + 2).mul(sin_theta_2) + 4*cos_theta.mul(temp_sqrt);
 
    rho_sp = rho_sp_num / rho_sp_dec;
    rho_dp = rho_dp_num / rho_dp_dec;
}


// 计算 rho_dp_desired rho_sp_desired
void Polarimetric_Visual_Servoing::get_rho_dp_sp_desired()
{
    get_rho_dp_sp(this->theta_desired_, this->eta_, this->rho_dp_desired_, this->rho_sp_desired_);
}

// 计算 rho_dp_current rho_sp_current     
void Polarimetric_Visual_Servoing::get_rho_dp_sp_current()
{
    get_rho_dp_sp(this->theta_current_, this->eta_, this->rho_dp_current_, this->rho_sp_current_);
}

// 计算期望图像交互矩阵和特征 
// the light source motionless with respect to the object frame and located at infinity
void Polarimetric_Visual_Servoing::get_feature_error_interaction_matrix_desired_a()
{
    Mat L_n_desired, L_theta_desired, L_phi_desired, L_rho_sp_desired, L_rho_dp_desired, 
        L_V_desired, L_Sa_desired, L_udb_desired, L_usa_desired, L_Ida_desired, L_Isa_desired, 
        L_Iu_desired, L_Oa_desired, L_Aa_desired, L_Phi_desired;
    double* prt_phi = (double*)(this->phi_desired_.data);

    Mat P_desired = get_P_desried();
    Mat L_I_pol_a_desired = Mat::zeros(this->resolution_y_ * this->resolution_x_, 6, CV_64FC1);
    int row = 0, col = 0;

    for(int cnt = 0; cnt < L_I_pol_a_desired.rows; cnt++)
    {
        // L_n
        L_n_desired = get_L_n(this->n_desired_.col(cnt)); 
        // L_theta
        // L_theta_desired = get_L_theta(*prt_phi++);


        
        if(++col >= this->resolution_x_)
        {
            col = 0;
            ++row;
        }
    }

//     for i = 1 : row
//     for j = 1 : col
  

//         % L_phi
//         L_phi_desired = get_L_phi(theta_desired(i,j), phi_desired(i,j));
//         % L_rho_sp
//         L_rho_sp_desired = get_L_rho_sp(theta_desired(i,j), phi_desired(i,j), eta);
//         % L_rho_dp 
//         L_rho_dp_desired = get_L_rho_dp(theta_desired(i,j), phi_desired(i,j), eta);
//         % L_V
//         L_V_desired = get_L_V(reshape(P_desired(i,j,:),[],1));
//         % L_S
//         L_Sa_desired = get_L_Sa(reshape(S(i,j,:),[],1));
//         L_Sb_desired = get_L_Sb(reshape(P_desired(i,j,:),[],1));
//         % L_ud
//         L_udb_desired = get_L_udb(reshape(n_desired(i,j,:),[],1), reshape(S(i,j,:),[],1), L_n_desired, L_Sb_desired);
//         % L_us 
//         L_usa_desired = get_L_usa(ud_desired(i,j), reshape(n_desired(i,j,:),[],1), reshape(S(i,j,:),[],1), reshape(V(i,j,:),[],1), L_n_desired, L_V_desired, L_Sa_desired);
//         L_usb_desired = get_L_usb(ud_desired(i,j), reshape(n_desired(i,j,:),[],1), reshape(S(i,j,:),[],1), reshape(V(i,j,:),[],1), L_n_desired, L_V_desired, L_Sb_desired, L_udb_desired);
//         % L_Id
//         L_Ida_desired = zeros(1, 6);
//         L_Idb_desired = I_in_k_d * L_udb_desired;
//         % L_Is
//         L_Isa_desired = I_in_k_s_pi * k * us_desired(i,j)^(k-1) * L_usa_desired;
//         L_Isb_desired = I_in_k_s_pi * k * us_desired(i,j)^(k-1) * L_usb_desired;
//         % L_Iu
//         L_Iu_desired = zeros(1,6);
//         % L_O
//         L_Oa_desired = 1/2 * (L_Isa_desired + L_Ida_desired + L_Iu_desired);
//         L_Ob_desired = 1/2 * (L_Isb_desired + L_Idb_desired + L_Iu_desired);
//         % L_A
//         L_Aa_desired = 1/2 * (rho_dp_desired(i,j) * L_Ida_desired + Id_desired(i,j)*L_rho_dp_desired - rho_sp_desired(i,j)*L_Isa_desired - Is_desired(i,j)*L_rho_sp_desired);
//         L_Ab_desired = 1/2 * (rho_dp_desired(i,j) * L_Idb_desired + Id_desired(i,j)*L_rho_dp_desired - rho_sp_desired(i,j)*L_Isb_desired - Is_desired(i,j)*L_rho_sp_desired);
//         % L_Phi
//         L_Phi_desired = 2*L_phi_desired;
//         % L_I_pol
//         L_I_pol_a_desired(cnt, :) = L_Oa_desired + cos(2*phi_pol - phi_desired(i,j))*L_Aa_desired + A_desired(i,j)*sin(2*phi_pol - phi_desired(i,j))*L_Phi_desired;
//         L_I_pol_b_desired(cnt, :) = L_Ob_desired + cos(2*phi_pol - phi_desired(i,j))*L_Ab_desired + A_desired(i,j)*sin(2*phi_pol - phi_desired(i,j))*L_Phi_desired;
//         cnt = cnt + 1;
//     end
// end
}


// 计算期望图像交互矩阵和特征 
// the light source mounted on the camera
void Polarimetric_Visual_Servoing::get_feature_error_interaction_matrix_desired_b()
{
    Mat P_desired = get_P_desried();


}

// % 计算期望图像空间点坐标
Mat Polarimetric_Visual_Servoing::get_P_desried()
{
    Mat P = Mat::zeros(3, this->resolution_y_ * this->resolution_x_, CV_64FC1);
    int row = 1, col = 1;
    Mat image_Z = image_depth_desired_.reshape(0, 1);
    double *ptr = image_Z.ptr<double>(0, 0);

    for(int i = 0; i < V_.cols; i++)
    {
        P.col(i) = this->camera_intrinsic_inv_ * cv::Vec3d(col, row, 1.0) 
                            * ptr[i];
        if(++col > this->resolution_x_)
        {
            col = 1;
            ++row;
        }
    }
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
    Mat temp_1 = (I_0 - O) / A;
    Mat temp_2 = (I_45 - O) / A;
    Mat temp_3 = (O - I_90) / A;
    Mat temp_4 = (O - I_135) / A;
    Phi = 1/4 * (cv_acos(temp_1) + cv_asin(temp_2) + cv_acos(temp_3) + cv_asin(temp_4));
}

void Polarimetric_Visual_Servoing::get_O_A_Phi_desired(Mat I_0, Mat I_45, Mat I_90, Mat I_135)
{
    get_O_A_Phi(I_0, I_45, I_90, I_135, O_desired_, A_desired_, Phi_desired_);
}
        
void Polarimetric_Visual_Servoing::get_O_A_Phi_current(Mat I_0, Mat I_45, Mat I_90, Mat I_135)
{
    get_O_A_Phi(I_0, I_45, I_90, I_135, O_current_, A_current_, Phi_current_);
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
    // // 计算特征误差 交互矩阵
    // void Discrete_Orthogonal_Moment_VS::get_feature_error_interaction_matrix()
    // {
    //     // 准备
    //     get_order_adaption();
    //     int num = int((double(this->order_) + 2) * (double(this->order_) + 1) / 2.0);
    //     this->L_e_ = Mat::zeros(num, 6, CV_64FC1); 
    //     this->error_s_ = Mat::zeros(num, 1, CV_64FC1); 
    //     Mat feature_new= Mat::zeros(num, 1, CV_64FC1); 
    //     Mat feature_old= Mat::zeros(num, 1, CV_64FC1); 
    //     Mat Le_new =  Mat::zeros(num, 6, CV_64FC1);
    //     Mat Le_old =  Mat::zeros(num, 6, CV_64FC1);
    //     Mat DOM_XY;
    //     int cnt = 0;
    //     // 计算离散正交矩所需矩阵 DOM_x_ DOM_y_
    //     get_DOM_matrix();
    //     // 计算灰度交互矩阵
    //     Mat L_I_new = get_interaction_matrix_gray(this->image_gray_current_, this->image_depth_current_, this->camera_intrinsic_);
    //     Mat L_I_old = get_interaction_matrix_gray(this->image_gray_desired_, this->image_depth_desired_, this->camera_intrinsic_);
    //     // 计算特征 交互矩阵
    //     for(int l = 0; l <= this->order_; l++)
    //     {
    //         for(int k = 0; k <= this->order_; k++)
    //         {
    //             if(l + k > this->order_){
    //                 continue;
    //             }else{
    //                 // 准备            
    //                 DOM_XY = repeat(this->DOM_x_.row(l), this->M_, 1).mul(
    //                     repeat(this->DOM_y_.row(k).t(), 1, this->N_));                     
    //                 // 计算特征
    //                 ((double*)feature_new.data)[cnt] = sum(DOM_XY.mul(this->image_gray_current_))[0];
    //                 ((double*)feature_old.data)[cnt] = sum(DOM_XY.mul(this->image_gray_desired_))[0];
    //                 // 计算交互矩阵
    //                 get_interaction_matrix_DOM_once(DOM_XY, L_I_new).copyTo(Le_new.row(cnt));
    //                 get_interaction_matrix_DOM_once(DOM_XY, L_I_old).copyTo(Le_old.row(cnt));
    //                 // 计数
    //                 cnt++;
    //             }
    //         }
    //     }   
    //     // 计算特征误差 交互矩阵
    //     this->error_s_ = feature_new.rowRange(0, cnt) - feature_old.rowRange(0, cnt);
    //     this->L_e_ = 0.5*(Le_new.rowRange(0, cnt) + Le_old.rowRange(0, cnt));
    // }
}

void Polarimetric_Visual_Servoing::save_data_error_feature()
{
    // void Direct_Visual_Servoing::save_data_error_feature()
    // {
    //     Mat error_ave = (this->error_s_.t() * this->error_s_) / (this->error_s_.rows * this->error_s_.cols);
    //     this->data_vs.error_feature_.push_back(error_ave);
    // }
}