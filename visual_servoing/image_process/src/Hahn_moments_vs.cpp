#include "Hahn_moments_vs.h"
#include <opencv2/imgproc.hpp>
#include <math.h>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <string>
#include <ctime> 
#include <chrono>

Hahn_Moments_VS::Hahn_Moments_VS(int order_min=4, int order_max=8, double delta_epsilon=0.1, double lambda_order=1.2, int resolution_x=640, int resolution_y=480)
            : Discrete_Orthogonal_Moment_VS(order_min, order_max, delta_epsilon, lambda_order, resolution_x, resolution_y)
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
    Mat X = linspace(0, this->N_-1, this->N_);
    Mat Y = linspace(0, this->M_-1, this->M_);
    reduce(this->image_gray_current_, image_gray_new_x, 0, REDUCE_SUM);
    reduce(this->image_gray_current_, image_gray_new_y, 1, REDUCE_SUM);
    reduce(this->image_gray_desired_, image_gray_old_x, 0, REDUCE_SUM);
    reduce(this->image_gray_desired_, image_gray_old_y, 1, REDUCE_SUM);
    // 计算图片重心
    Mat xc_new = image_gray_new_x * X.t() / sum(image_gray_new_x)[0];
    Mat yc_new = image_gray_new_y.t() * Y.t() / sum(image_gray_new_x)[0] ;
    Mat xc_old = image_gray_old_x * X.t() / sum(image_gray_old_x)[0];
    Mat yc_old = image_gray_old_y.t() * Y.t() / sum(image_gray_old_x)[0] ;  
    double xc = (xc_new.at<double>(0,0) + xc_old.at<double>(0,0)) / (2*this->N_);
    double yc = (yc_new.at<double>(0,0) + yc_old.at<double>(0,0)) / (2*this->M_);
    // 计算散度
    int sx_new, sy_new, sx_old, sy_old;
    get_image_spread(this->image_gray_current_, round(xc*this->N_), 
                            round(yc*this->M_), sx_new, sy_new);
    get_image_spread(this->image_gray_desired_, round(xc*this->N_), 
                            round(yc*this->M_), sx_old, sy_old);
    // 计算new图像的Hahn参数a,b
    double multiple = 20.0;
    double infinitesimal = 0.01;
    Mat temp;
    // x
    Mat A_x_new = (cv::Mat_<double>(2, 2) << 
            3.0*sqrt(this->N_*xc*(1.0-xc)), 1.0, this->N_-1.0, 1.0);
    Mat k_x_new = A_x_new.inv() * (cv::Mat_<double>(2, 1) << log(multiple*(N_-1)), log(infinitesimal));
    temp = k_x_new.t() * (cv::Mat_<double>(2, 1) << double(sx_new), 1.0);
    double tx_new = exp(temp.at<double>(0,0));
    // y
    Mat A_y_new = (cv::Mat_<double>(2, 2) << 
            3.0*sqrt(this->M_*yc*(1.0-yc)), 1.0, this->M_-1.0, 1.0);
    Mat k_y_new = A_y_new.inv() * (cv::Mat_<double>(2, 1) << log(multiple*(M_-1)), log(infinitesimal));
    temp = k_y_new.t() * (cv::Mat_<double>(2, 1) << double(sy_new), 1.0);
    double ty_new = exp(temp.at<double>(0,0));
    // 
    double b_x_new = xc * tx_new;
    double a_x_new = (1.0 - xc) * tx_new;
    double b_y_new = yc * ty_new;
    double a_y_new = (1.0 - yc) * ty_new;
    // 计算old图像的Hahn参数a,b
    // x
    Mat A_x_old = (cv::Mat_<double>(2, 2) << 
            3.0*sqrt(this->N_*xc*(1.0-xc)), 1.0, this->N_-1.0, 1.0);
    Mat k_x_old = A_x_old.inv() * (cv::Mat_<double>(2, 1) << log(multiple*(N_-1)), log(infinitesimal));
    temp = k_x_old.t() * (cv::Mat_<double>(2, 1) << double(sx_old), 1.0);
    double tx_old = exp(temp.at<double>(0,0));
    // y
    Mat A_y_old = (cv::Mat_<double>(2, 2) << 
            3.0*sqrt(this->M_*yc*(1.0-yc)), 1.0, this->M_-1.0, 1.0);
    Mat k_y_old = A_y_old.inv() * (cv::Mat_<double>(2, 1) << log(multiple*(M_-1)), log(infinitesimal));
    temp = k_y_old.t() * (cv::Mat_<double>(2, 1) << double(sy_old), 1.0);
    double ty_old = exp(temp.at<double>(0,0));
    //
    double b_x_old = xc * tx_old;
    double a_x_old = (1.0 - xc) * tx_old;
    double b_y_old = yc * ty_old;
    double a_y_old = (1.0 - yc) * ty_old;
    // 输出
    this->ax = round((a_x_new + a_x_old) / 2.0);
    this->bx = round((b_x_new + b_x_old) / 2.0);
    this->ay = round((a_y_new + a_y_old) / 2.0);
    this->by = round((b_y_new + b_y_old) / 2.0);
}


// 计算散度
void Hahn_Moments_VS::get_image_spread(Mat& img, int xc, int yc, int& sx, int& sy)
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
                p = p + ((double*)p_gray_x.data)[num_right_x];
                sx_right = sx_right + 1;
            }
            num_left_x = xc - i;
            if(num_left_x >= 0)
            {
                p = p + ((double*)p_gray_x.data)[num_left_x];
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
    for(int i = 1; i <= img.rows; i++)
    {
        if(p <= p_n_sigma)
        {
            num_right_y = yc + i;
            if(num_right_y < img.rows)
            {
                p = p + ((double*)p_gray_y.data)[num_right_y*p_gray_y.cols];
                sy_right = sy_right + 1;
            }
            num_left_y = yc - i;
            if(num_left_y >= 0)
            {
                p = p + ((double*)p_gray_y.data)[num_left_y*p_gray_y.cols];
                sy_left = sy_left + 1;
            }
        }
        else
        {
            break;
        }
    }
    sy = std::max(sy_right, sy_left); 
}



// 计算Hahn正交多项式 
// Hn = [0 1 2 ... N-1
//       1   .
//       2     .
//       .       .
//       .
//       .
//       n     ...   ]
Mat Hahn_Moments_VS::get_orthogonal_polynomial_HM(int N, int order, int a, int b)
{
    // 初始化
    Mat Hn = Mat::zeros(order+1, N, CV_64FC1);
    double lambda=0.0, sigma=0.0, tao=0.0;
    double w_x_x_1, w_x_x_2;
    double e = 1e-6, h, dh, ddh;
    bool flag_1, flag_2;  
    double temp;
    // 计算第1列
    temp = 1;
    for(int i = 1; i <= b+1; i++)
    {
        temp = temp * double(a+i)/double(N+a-1.0+i);
    }    
    ((double*)Hn.data)[0] = sqrt(temp);
    for(int n = 1; n <= order; n++)
    {
        ((double*)Hn.data)[n*Hn.cols] = -sqrt(double(N-n)*(n+b)*(a+b+n) / double(n*(a+n)*(a+b+N+n))) * sqrt(double(2*n+1+a+b) / double(2*n-1+a+b)) * ((double*)Hn.data)[(n-1)*Hn.cols];
    }
    // 计算第2列
    for(int n = 0; n <= order; n++)
    {
        ((double*)Hn.data)[n*Hn.cols+1] = (1 - double(n*(n+a+b+1)) / double((b+1)*(N-1))) * sqrt(w_x_x_1_Hahn(N, 1, a, b)) * ((double*)Hn.data)[n*Hn.cols];
    } 
    // 计算第3...N列
    for(int n = 0; n <= order; n++)
    {
        lambda =  n * (a + b + n + 1);
        for(int x = 2; x < N; x++)
        {
            sigma = double((x-1) * (N +a - x + 1));
            tao = double((b+1)*(N-1) - (2+a+b)*(x-1));
            w_x_x_1 = w_x_x_1_Hahn(N, x, a, b);
            w_x_x_2 = w_x_x_2_Hahn(N, x, a, b);
            ((double*)Hn.data)[n*Hn.cols+x] = 1.0 / (sigma + tao) * 
                    ((2*sigma + tao - lambda) * sqrt(w_x_x_1) * ((double*)Hn.data)[n*Hn.cols+x-1]
                        - sigma * sqrt(w_x_x_2) * ((double*)Hn.data)[n*Hn.cols+x-2]);    
            // 抑制数值不稳定 
            h = ((double*)Hn.data)[n*Hn.cols+x]; 
            dh = ((double*)Hn.data)[n*Hn.cols+x] - ((double*)Hn.data)[n*Hn.cols+x-1];
            ddh = ((double*)Hn.data)[n*Hn.cols+x] - 2*((double*)Hn.data)[n*Hn.cols+x-1] + ((double*)Hn.data)[n*Hn.cols+x-2];
            flag_1 = (h>0) && (h<e) && (dh>-e) && (dh<0) && (ddh>0) && (ddh<e); // 0<h<e, -e<dh<0, 0<ddh<e
            flag_2 = (h>-e) && (h<0) && (dh>0) && (dh<e) && (ddh>-e) && (ddh<0);// -e<h<0, 0<dh<e, -e<ddh< 0
            if (flag_1 || flag_2)
            {
                Hn.rowRange(n, n+1).colRange(x, N) = 0;
                break;
            }
        }
        temp = temp + 1;
    }
    return Hn;  
}


double Hahn_Moments_VS::w_x_x_1_Hahn(int N, int x, int a, int b)
{
    return double(b+x) * double(N-x) / double((N+a-x)*x);
}

double Hahn_Moments_VS::w_x_x_2_Hahn(int N, int x, int a, int b)
{
    return double((b+x)*(b+x-1)) / double((x*(x-1))) * double((N-x)*(N-x+1)) / double((N-x+a)*(N-x+a+1));
}

void Hahn_Moments_VS::save_data_moments_parameter()
{
    this->data_hm.ax_list_.push_back(this->ax);
    this->data_hm.ay_list_.push_back(this->ay);
    this->data_hm.bx_list_.push_back(this->bx);
    this->data_hm.by_list_.push_back(this->by);
}

void Hahn_Moments_VS::write_data_moments(ofstream& oFile)
{
    oFile << "ax" << endl;
    write_to_excel(this->data_hm.ax_list_, oFile);     
    oFile << "bx" << endl;
    write_to_excel(this->data_hm.bx_list_, oFile);  
    oFile << "ay" << endl;
    write_to_excel(this->data_hm.ay_list_, oFile);  
    oFile << "by" << endl;
    write_to_excel(this->data_hm.by_list_, oFile);  
}

string Hahn_Moments_VS::get_method_name()
{
    return "HM_VS";
}