#include "Krawtchouk_moments_vs.h"
#include <opencv2/imgproc.hpp>
#include <math.h>
#include <iostream>
#include <fstream>
#include <string>
#include <ctime> 
#include <chrono>

Krawtchouk_Moments_VS::Krawtchouk_Moments_VS(int order_min=4, int order_max=8, double delta_epsilon=0.1, double lambda_order=1.2, int resolution_x=640, int resolution_y=480)
            : Discrete_Orthogonal_Moment_VS(order_min, order_max, delta_epsilon, lambda_order, resolution_x, resolution_y)
{
    this->px = 0.5;
    this->py = 0.5;
}


void Krawtchouk_Moments_VS::get_DOM_matrix()
{
    get_Krawtchouk_Moments_parameters();

    this->DOM_x_ = get_orthogonal_polynomial_KM(this->N_, this->order_, this->px);
    this->DOM_y_ = get_orthogonal_polynomial_KM(this->M_, this->order_, this->py);
}

void Krawtchouk_Moments_VS::get_Krawtchouk_Moments_parameters()
{
    Mat image_gray_new_x, image_gray_new_y, image_gray_old_x, image_gray_old_y;
    
    Mat X = linspace(0, this->N_-1, this->N_);
    Mat Y = linspace(0, this->M_-1, this->M_);

    reduce(this->image_gray_current_, image_gray_new_x, 0, REDUCE_SUM);
    reduce(this->image_gray_current_, image_gray_new_y, 1, REDUCE_SUM);
    reduce(this->image_gray_desired_, image_gray_old_x, 0, REDUCE_SUM);
    reduce(this->image_gray_desired_, image_gray_old_y, 1, REDUCE_SUM);

    Mat xc_new = image_gray_new_x * X.t() / sum(image_gray_new_x)[0];
    Mat yc_new = image_gray_new_y.t() * Y.t() / sum(image_gray_new_x)[0] ;
    Mat xc_old = image_gray_old_x * X.t() / sum(image_gray_old_x)[0];
    Mat yc_old = image_gray_old_y.t() * Y.t() / sum(image_gray_old_x)[0] ;

    this->px = (((double*)xc_new.data)[0] + ((double*)xc_old.data)[0]) / (2*this->N_);
    this->py = (((double*)yc_new.data)[0] + ((double*)yc_old.data)[0]) / (2*this->M_);
}

// 计算Krawtchouk正交多项式 
// Kn = [0 1 2 ... N-1
//       1   .
//       2     .
//       .       .
//       .
//       .
//       n     ...   ]
Mat Krawtchouk_Moments_VS::get_orthogonal_polynomial_KM(int N, int order, double p)
{
    // 初始化
    Mat Kn = Mat::zeros(order+1, N, CV_64FC1);
    double lambda=0.0, sigma=0.0, tao=0.0;
    double w_x_x_1, w_x_x_2;
    double e = 1e-6, k, dk, ddk;
    bool flag_1, flag_2;
    // 计算第1列
    Kn.at<double>(0,0) = pow((1.0 - p), (double(N)/2.0));
    for(int n = 1; n <= order; n++)
    {
        ((double*)Kn.data)[n*Kn.cols] = -sqrt(double(N-n+1) / double(n)) * sqrt((p) / (1.0-p)) * ((double*)Kn.data)[(n-1)*Kn.cols];
        
    }
    // 计算第2列
    for(int n = 0; n <= order; n++)
    {
        ((double*)Kn.data)[n*Kn.cols+1] = (1.0 - double(n)/double(N*p))  * sqrt(w_x_x_1_Krawtchouk(N, 1, p)) * ((double*)Kn.data)[n*Kn.cols];
    }    
    // 计算第3...N列
    for(int n = 0; n <= order; n++)
    {
        lambda = n / (1.0 - p);
        for(int x = 2; x < N; x++)
        {
            sigma = double(x-1);
            tao = double(N * p - x + 1) / (1.0 - p);
            w_x_x_1 = w_x_x_1_Krawtchouk(N, x, p);
            w_x_x_2 = w_x_x_2_Krawtchouk(N, x, p);
            ((double*)Kn.data)[n*Kn.cols+x] = 1.0 / (sigma + tao) * 
                    ((2*sigma + tao - lambda) * sqrt(w_x_x_1) * ((double*)Kn.data)[n*Kn.cols+x-1] 
                        - sigma * sqrt(w_x_x_2) * ((double*)Kn.data)[n*Kn.cols+x-2]);    
            // 抑制数值不稳定 
            k = ((double*)Kn.data)[n*Kn.cols+x]; 
            dk = ((double*)Kn.data)[n*Kn.cols+x] - ((double*)Kn.data)[n*Kn.cols+x-1];
            ddk = ((double*)Kn.data)[n*Kn.cols+x] - 2*((double*)Kn.data)[n*Kn.cols+x-1] + ((double*)Kn.data)[n*Kn.cols+x-2];
            flag_1 = (k>0) && (k<e) && (dk>-e) && (dk<0) && (ddk>0) && (ddk<e); // 0<h<e, -e<dh<0, 0<ddh<e
            flag_2 = (k>-e) && (k<0) && (dk>0) && (dk<e) && (ddk>-e) && (ddk<0);// -e<h<0, 0<dh<e, -e<ddh< 0
            if (flag_1 || flag_2)
            {
                Kn.rowRange(n, n+1).colRange(x, N) = 0;
                break;
            }
        }
    }
    return Kn;    
}

double Krawtchouk_Moments_VS::w_x_x_1_Krawtchouk(int N, int x, double p)
{
    return p / (1.0 - p) * double(N - x + 1) / x;
}

double Krawtchouk_Moments_VS::w_x_x_2_Krawtchouk(int N, int x, double p)
{
    return double(pow((p / (1.0 - p)), 2) * (N - x + 1)*(N - x + 2)) / double(x*(x-1));
}


void Krawtchouk_Moments_VS::save_data_moments_parameter()
{
    this->data_km.px_list_.push_back(this->px);
    this->data_km.py_list_.push_back(this->py);
}

void Krawtchouk_Moments_VS::write_data_moments(ofstream& oFile)
{
    oFile << "px" << endl;
    write_to_excel(this->data_km.px_list_, oFile);    
    oFile << "py" << endl;
    write_to_excel(this->data_km.py_list_, oFile); 
}

string Krawtchouk_Moments_VS::get_method_name()
{
    return "KM_VS";
}
