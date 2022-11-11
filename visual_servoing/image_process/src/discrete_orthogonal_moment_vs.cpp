#include "discrete_orthogonal_moment_vs.h"
#include "direct_visual_servoing.h"
#include <opencv2/imgproc.hpp>
#include <math.h>

Discrete_Orthogonal_Moment_VS::Discrete_Orthogonal_Moment_VS(int order_min, int order_max, int resolution_x=640, int resolution_y=480):Direct_Visual_Servoing(resolution_x, resolution_y)
{
    this->N_ = resolution_y;
    this->M_ = resolution_x;
    this->order_min_ = order_min;
    this->order_max_ = order_max;   
}

// ����������� ��������
void Discrete_Orthogonal_Moment_VS::get_feature_error_interaction_matrix()
{
    // ׼��
    get_order_adaption();
    int num = int((double(this->order_) + 2) * (double(this->order_) + 1) / 2.0);
    this->L_e_ = Mat::zeros(num, 6, CV_64FC1); 
    this->error_s_ = Mat::zeros(num, 1, CV_64FC1); 
    Mat feature_new= Mat::zeros(num, 1, CV_64FC1); 
    Mat feature_old= Mat::zeros(num, 1, CV_64FC1); 
    Mat Le_new =  Mat::zeros(num, 6, CV_64FC1);
    Mat Le_old =  Mat::zeros(num, 6, CV_64FC1);
    Mat DOM_XY = Mat::zeros(this->N_, this->M_, CV_64FC1);
    int cnt = 0;
    // ������ɢ������������� DOM_x_ DOM_y_
    get_DOM_matrix();
    // ����ҶȽ�������
    Mat L_I_new = get_interaction_matrix_gray(this->image_gray_current_, this->image_depth_current_, this->camera_intrinsic_);
    Mat L_I_old = get_interaction_matrix_gray(this->image_gray_desired_, this->image_depth_desired_, this->camera_intrinsic_);
    // �������� ��������
    for(int l = 0; l <= this->order_; l++)
    {
        for(int k = 0; k <= this->order_; k++)
        {
            if(l + k > this->order_){
                continue;
            }else{
                // ׼��  
                Mat M_xx = repeat(this->DOM_x_.row(l), this->M_, 1);
                Mat M_yy = repeat(this->DOM_y_.row(k).t(), 1, this->N_);
                DOM_XY = repeat(this->DOM_x_.row(l), this->M_, 1).mul(
                    repeat(this->DOM_y_.row(k).t(), 1, this->N_));       
                // ��������
                feature_new.at<double>(cnt, 0) = sum(DOM_XY.mul(this->image_gray_current_))[0];
                feature_old.at<double>(cnt, 0) = sum(DOM_XY.mul(this->image_gray_desired_))[0];
                // ���㽻������
                get_interaction_matrix_DOM_once(DOM_XY, L_I_new).copyTo(Le_new.row(cnt));
                get_interaction_matrix_DOM_once(DOM_XY, L_I_old).copyTo(Le_old.row(cnt));
                // ����
                cnt++;
            }
        }
    }
    // ����������� ��������
    this->error_s_ = feature_new.rowRange(0, cnt) - feature_old.rowRange(0, cnt);
    this->L_e_ = 0.5*(Le_new.rowRange(0, cnt) + Le_old.rowRange(0, cnt));
}

// ����ÿ�������ͽ�������
Mat Discrete_Orthogonal_Moment_VS::get_interaction_matrix_DOM_once(Mat DOM_XY, Mat L_I)
{
    Mat DOM_XY_Vec = DOM_XY.reshape(0, DOM_XY.rows*DOM_XY.cols);
    Mat L_once = Mat::zeros(1, L_I.cols, CV_64FC1);
    for(int i = 0; i < L_I.rows; i++)
    {
        L_once = L_once + DOM_XY_Vec.at<double>(i,0) * L_I.row(i);
    }
    return L_once;
}

// ������ɢ�������������
int Discrete_Orthogonal_Moment_VS::get_order_adaption()
{
    Mat vec_image_new = this->image_gray_current_.reshape(0, this->N_*this->M_);
    Mat vec_image_old = this->image_gray_desired_.reshape(0, this->N_*this->M_);
    Mat vec_image_init = this->image_gray_initial_.reshape(0, this->N_*this->M_);
    
    Mat err = vec_image_new - vec_image_old;
    Mat err_0 = vec_image_init - vec_image_old;

    Mat error_ave = err.t() * err / (err.rows * err.cols);
    double error_pixel_ave = error_ave.at<double>(0,0);
    Mat error_ave_0 = err_0.t() * err_0 / (err_0.rows * err_0.cols);
    double error_pixel_ave_0 = error_ave_0.at<double>(0,0);

    double k = 10.0;
    
    if (error_pixel_ave > error_pixel_ave_0)
        error_pixel_ave = error_pixel_ave_0;

    double t = exp(-k*(error_pixel_ave / (error_pixel_ave_0 - error_pixel_ave)));
    double order_ = (this->order_max_ - this->order_min_)*t + this->order_min_;
    this->order_ = round(order_);  

    return this->order_;
}

