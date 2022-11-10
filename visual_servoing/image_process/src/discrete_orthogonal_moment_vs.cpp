#include "discrete_orthogonal_moment_vs.h"
#include "direct_visual_servoing.h"
#include <opencv2/imgproc.hpp>
#include <math.h>

Discrete_Orthogonal_Moment_VS::Discrete_Orthogonal_Moment_VS(int order_min, int order_max, int resolution_x, int resolution_y):Direct_Visual_Servoing(resolution_x, resolution_y)
{
    this->N_ = resolution_y;
    this->M_ = resolution_x;
    this->order_min_ = order_min;
    this->order_max_ = order_max;   
}

// 计算特征误差 交互矩阵
void Discrete_Orthogonal_Moment_VS::get_feature_error_interaction_matrix()
{
    // 准备
    get_order_adaption();
    int num = int((double(this->order_) + 2) * (double(this->order_) + 1) / 2.0);
    this->L_e_ = Mat::zeros(num, 6, CV_64FC1); 
    this->error_s_ = Mat::zeros(num, 1, CV_64FC1); 
    Mat DOM_new= Mat::zeros(num, 1, CV_64FC1); 
    Mat DOM_old= Mat::zeros(num, 1, CV_64FC1); 
    Mat Le_new =  Mat::zeros(num, 6, CV_64FC1);
    Mat Le_old =  Mat::zeros(num, 6, CV_64FC1);
    int cnt = 1;
    // 计算离散正交矩所需矩阵 DOM_X_ DOM_Y_
    get_DOM_matrix();
    // 计算灰度交互矩阵
    Mat L_I_new = get_interaction_matrix_gray(this->image_gray_current_, this->image_depth_current_, this->camera_intrinsic_);
    Mat L_I_old = get_interaction_matrix_gray(this->image_gray_desired_, this->image_depth_desired_, this->camera_intrinsic_);
    // 计算特征 交互矩阵
    for(int l = 0; l <= this->order_; l++)
    {
        for(int k = 0; k <= this->order_; k++)
        {
            if(l + k > this->order_){
                continue;
            }else{
                // 
            }

        }
    }
    // 计算特征误差 交互矩阵
    this->error_s_ = DOM_new - DOM_old;
    this->L_e_ = 0.5*(Le_new + Le_old);
}

// for l = 0 : order
//     for k = 0 : order
//         if l + k > order
//             continue;
//         else 
//             %%
//             % 准备
//             DnX = repmat(Dnx(l+1,:)', 1, M); % 图像横坐标为X,纵坐标为Y
//             DnY = repmat(Dny(k+1,:), N, 1);
//             DnXY = DnX .* DnY;
//             % 计算特征
//             Product_new = DnXY .* image_gray_new;
//             Product_old = DnXY .* image_gray_old;
//             Dnm_new(cnt, 1) = sum(Product_new(:));
//             Dnm_old(cnt, 1) = sum(Product_old(:));
//             % 计算交互矩阵
//             L_e_new(cnt, :) = sum(DnXY(:) .* L_I_new);
//             L_e_old(cnt, :) = sum(DnXY(:) .* L_I_old);
//             % 计数
//             cnt = cnt + 1;            
//         end
//     end
// end

// 计算离散正交矩所需阶数
int Discrete_Orthogonal_Moment_VS::get_order_adaption()
{
    Mat vec_image_new = this->image_gray_current_.reshape(0, this->image_gray_current_.rows*this->image_gray_current_.cols);
    Mat vec_image_old = this->image_gray_desired_.reshape(0, this->image_gray_desired_.rows*this->image_gray_desired_.cols);
    Mat vec_image_init = this->image_gray_initial_.reshape(0, this->image_gray_initial_.rows*this->image_gray_initial_.cols);
    
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

