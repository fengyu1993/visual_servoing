#include "visual_servoing.h"
#include <opencv2/imgproc.hpp>
#include <math.h>

Visual_Servoing::Visual_Servoing(int resolution_x=640, int resolution_y=480)
{
    this->resolution_x_ = resolution_x;
    this->resolution_y_ = resolution_y;
    this->image_gray_desired_ = Mat::zeros(this->resolution_y_, this->resolution_x_, CV_64FC1);
    this->image_gray_current_ = Mat::zeros(this->resolution_y_, this->resolution_x_, CV_64FC1);
    this->image_depth_desired_ = Mat::zeros(this->resolution_y_, this->resolution_x_, CV_64FC1);
    this->image_depth_current_ = Mat::zeros(this->resolution_y_, this->resolution_x_, CV_64FC1);
    this->camera_intrinsic_ = Mat::zeros(3, 3, CV_64FC1);
    this->camera_velocity_ = Mat::zeros(6, 1, CV_64FC1);
}

// 初始化
void Visual_Servoing::init_VS(double lambda, double epsilon, Mat image_gray_desired, Mat image_depth_desired, Mat image_gray_initial, Mat camera_intrinsic)
{
    this->lambda_ = lambda;
    this->epsilon_ = epsilon;
    set_camera_intrinsic(camera_intrinsic);
    set_image_depth_desired(image_depth_desired);
    set_image_gray_desired(image_gray_desired);
    set_image_gray_initial(image_gray_initial);
}

// 计算相机速度
Mat Visual_Servoing::get_camera_velocity()
{
    Mat L_e_inv;
    get_feature_error_interaction_matrix();
    invert(this->L_e_, L_e_inv, DECOMP_SVD);
    this->camera_velocity_ = -this->lambda_ * L_e_inv * this->error_s_;
    return this->camera_velocity_;
}

// 设置相机内参
void Visual_Servoing::set_camera_intrinsic(Mat camera_intrinsic)
{
    camera_intrinsic.copyTo(this->camera_intrinsic_);
}

// 设置期望灰度图像
void Visual_Servoing::set_image_gray_desired(Mat image_gray_desired)
{
    image_gray_desired.copyTo(this->image_gray_desired_);
}

// 设置当前灰度图像
void Visual_Servoing::set_image_gray_current(Mat image_gray_current)
{
    // image_gray_current.copyTo(this->image_gray_current_);
    this->image_gray_current_ = image_gray_current;
}

// 设置初始图像
void Visual_Servoing::set_image_gray_initial(Mat image_gray_initial)
{
    image_gray_initial.copyTo(this->image_gray_initial_);
}

// 设置期望深度图像
void Visual_Servoing::set_image_depth_desired(Mat image_depth_desired)
{
    image_depth_desired.copyTo(this->image_depth_desired_);
}

// 设置当前深度图像
void Visual_Servoing::set_image_depth_current(Mat image_depth_current)
{
    // image_depth_current.copyTo(this->image_depth_current_);
    this->image_depth_current_ = image_depth_current;
}

