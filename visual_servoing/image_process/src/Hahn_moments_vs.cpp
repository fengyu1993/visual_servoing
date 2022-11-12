#include "Hahn_moments_vs.h"
#include <opencv2/imgproc.hpp>
#include <math.h>
#include <algorithm>


Hahn_Moments_VS::Hahn_Moments_VS(int order_min, int order_max, int resolution_x, int resolution_y)
            : Discrete_Orthogonal_Moment_VS(order_min, order_max, resolution_x, resolution_y)
{
    this->ax = 10;
    this->bx = 10;
    this->ay = 10;
    this->by = 10;
}


void Hahn_Moments_VS::get_DOM_matrix()
{
    get_Hahn_Moments_parameters();

    this->DOM_x_ = get_orthogonal_polynomial_HM(this->N_, this->order_, this->ax, this->bx);
    this->DOM_y_ = get_orthogonal_polynomial_HM(this->M_, this->order_, this->ay, this->by);
}

// 自适应计算Hahn矩参数
void Hahn_Moments_VS::get_Hahn_Moments_parameters()
{
    // 准备
    Mat image_gray_new_x, image_gray_new_y, image_gray_old_x, image_gray_old_y;   
    Mat X = linspace(0, this->image_gray_current_.cols-1, this->image_gray_current_.cols);
    Mat Y = linspace(0, this->image_gray_current_.rows-1, this->image_gray_current_.rows);
    reduce(this->image_gray_current_, image_gray_new_x, 0, REDUCE_SUM);
    reduce(this->image_gray_current_, image_gray_new_y, 1, REDUCE_SUM);
    reduce(this->image_gray_desired_, image_gray_old_x, 0, REDUCE_SUM);
    reduce(this->image_gray_desired_, image_gray_old_y, 1, REDUCE_SUM);
    // 计算图片重心
    Mat xc_new = image_gray_new_x * X.t() / sum(image_gray_new_x)[0];
    Mat yc_new = image_gray_new_y.t() * Y.t() / sum(image_gray_new_x)[0] ;
    Mat xc_old = image_gray_old_x * X.t() / sum(image_gray_old_x)[0];
    Mat yc_old = image_gray_old_y.t() * Y.t() / sum(image_gray_old_x)[0] ;  
    double xc = (xc_new.at<double>(0,0) + xc_old.at<double>(0,0)) / (2*this->image_gray_current_.cols);
    double yc = (yc_new.at<double>(0,0) + yc_old.at<double>(0,0)) / (2*this->image_gray_current_.rows);
    // 计算散度
    int sx_new, sy_new, sx_old, sy_old;
    get_image_spread(this->image_gray_current_, round(xc*this->image_gray_current_.cols), 
                            round(yc*this->image_gray_current_.rows), sx_new, sy_new);
    get_image_spread(this->image_gray_desired_, round(xc*this->image_gray_desired_.cols), 
                            round(yc*this->image_gray_desired_.rows), sx_old, sy_old);
    // 计算new图像的Hahn参数a,b



}

// 计算散度
void Hahn_Moments_VS::get_image_spread(Mat img, int xc, int yc, int& sx, int& sy)
{
    // 准备
    Mat image_gray_x, image_gray_y; 
    Mat p_gray_x, p_gray_y;
    double p_n_sigma = 0.9973; 
    reduce(img, image_gray_x, 0, REDUCE_SUM);
    reduce(img, image_gray_y, 1, REDUCE_SUM);
    p_gray_x = image_gray_x / sum(image_gray_x)[0];
    p_gray_y = image_gray_y / sum(image_gray_y)[0];
    // x方向
    int sx_right = 0, sx_left = 0, num_right_x = 0, num_left_x = 0;
    double p = p_gray_x.at<double>(0, xc);
    for(int i = 1; i <= img.cols; i++)
    {
        if(p <= p_n_sigma)
        {
            num_right_x = xc + i;
            if(num_right_x < img.cols)
            {
                p = p + p_gray_x.at<double>(0, num_right_x);
                sx_right = sx_right + 1;
            }
            num_left_x = xc - i;
            if(num_left_x >= 0)
            {
                p = p + p_gray_x.at<double>(0, num_left_x);
                sx_left = sx_left + 1;
            }
        }
        else
        {
            break;
        }
    }
    sx = std::max(sx_right, sx_left); 
    // y方向
    int sy_right = 0, sy_left = 0, num_right_y = 0, num_left_y = 0;
    p = p_gray_y.at<double>(yc, 0);
    for(int i = 1; i <= img.cols; i++)
    {
        if(p <= p_n_sigma)
        {
            num_right_y = yc + i;
            if(num_right_y < img.rows)
            {
                p = p + p_gray_y.at<double>(num_right_y, 0);
                sy_right = sy_right + 1;
            }
            num_left_y = yc - i;
            if(num_left_y >= 0)
            {
                p = p + p_gray_y.at<double>(num_right_y, 0);
                sy_left = sy_left + 1;
            }
        }
        else
        {
            break;
        }
    }
    sy = std::max(sx_right, sx_left); 
}



// 计算Hahn正交多项式 
// Hn = [0 1 2 ... N-1
//       1   .
//       2     .
//       .       .
//       .
//       .
//       n     ...   ]
Mat Hahn_Moments_VS::get_orthogonal_polynomial_HM(int N, int order, double ax, double bx)
{
  
}

double Hahn_Moments_VS::w_x_x_1_Hahn(int N, int x, double p)
{
    
}

double Hahn_Moments_VS::w_x_x_2_Hahn(int N, int x, double p)
{
    
}

