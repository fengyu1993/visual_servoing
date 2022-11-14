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

// ��ʼ��
void Visual_Servoing::init_VS(double lambda, double epsilon, Mat image_gray_desired, Mat image_depth_desired, Mat image_gray_initial, Mat camera_intrinsic)
{
    this->lambda_ = lambda;
    this->epsilon_ = epsilon;
    set_camera_intrinsic(camera_intrinsic);
    set_image_depth_desired(image_depth_desired);
    set_image_gray_desired(image_gray_desired);
    set_image_gray_initial(image_gray_initial);
    save_data_image();
}

// ��������ٶ�
Mat Visual_Servoing::get_camera_velocity()
{
    Mat L_e_inv;
    get_feature_error_interaction_matrix();
    invert(this->L_e_, L_e_inv, DECOMP_SVD);
    this->camera_velocity_ = -this->lambda_ * L_e_inv * this->error_s_;
    return this->camera_velocity_;
}

// ��������ڲ�
void Visual_Servoing::set_camera_intrinsic(Mat camera_intrinsic)
{
    camera_intrinsic.copyTo(this->camera_intrinsic_);
}

// ���������Ҷ�ͼ��
void Visual_Servoing::set_image_gray_desired(Mat image_gray_desired)
{
    image_gray_desired.copyTo(this->image_gray_desired_);
}

// ���õ�ǰ�Ҷ�ͼ��
void Visual_Servoing::set_image_gray_current(Mat image_gray_current)
{
    // image_gray_current.copyTo(this->image_gray_current_);
    this->image_gray_current_ = image_gray_current;
}

// ���ó�ʼͼ��
void Visual_Servoing::set_image_gray_initial(Mat image_gray_initial)
{
    image_gray_initial.copyTo(this->image_gray_initial_);
}

// �����������ͼ��
void Visual_Servoing::set_image_depth_desired(Mat image_depth_desired)
{
    image_depth_desired.copyTo(this->image_depth_desired_);
}

// ���õ�ǰ���ͼ��
void Visual_Servoing::set_image_depth_current(Mat image_depth_current)
{
    // image_depth_current.copyTo(this->image_depth_current_);
    this->image_depth_current_ = image_depth_current;
}

// 
void Visual_Servoing::save_data_image()
{
    save_data_image_gray_desired();
    save_data_image_gray_initial();
}

void Visual_Servoing::save_data_image_gray_desired()
{
    this->image_gray_desired_.copyTo(this->data_vs.image_gray_desired_);
}

void Visual_Servoing::save_data_image_gray_initial()
{
    this->image_gray_initial_.copyTo(this->data_vs.image_gray_init_);
}

void Visual_Servoing::save_data_camera_velocity()
{
    this->data_vs.velocity_.push_back(this->camera_velocity_);
}

void Visual_Servoing::save_data_error_feature()
{
    this->data_vs.error_feature_.push_back(this->error_s_);
}

void Visual_Servoing::save_data_camera_pose(Mat pose)
{
    this->data_vs.pose_.push_back(pose);
}

void Visual_Servoing::save_data(Mat pose)
{
    save_data_camera_velocity();
    save_data_camera_pose(pose);
    save_data_error_feature(); 
    save_date_other_parameter();
}