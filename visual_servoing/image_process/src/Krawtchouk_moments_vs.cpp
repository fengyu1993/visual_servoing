#include "Krawtchouk_moments_vs.h"
#include <opencv2/imgproc.hpp>
#include <math.h>


Krawtchouk_Moments_VS::Krawtchouk_Moments_VS(int order_min, int order_max, int resolution_x, int resolution_y)
            : Discrete_Orthogonal_Moment_VS(order_min, order_max, resolution_x, resolution_y)
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

    this->px = (xc_new.at<double>(0,0) + xc_old.at<double>(0,0)) / (2*this->N_);
    this->py = (yc_new.at<double>(0,0) + yc_old.at<double>(0,0)) / (2*this->M_);
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
        Kn.at<double>(n,0) = -sqrt(double(N-n+1) / double(n)) * sqrt((p) / (1.0-p)) * Kn.at<double>(n-1,0);
    }
    // 计算第2列
    for(int n = 0; n <= order; n++)
    {
        Kn.at<double>(n,1) = (1.0 - double(n)/double(N*p))  * sqrt(w_x_x_1_Krawtchouk(N, 1, p)) * Kn.at<double>(n,0);
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
            Kn.at<double>(n,x) = 1.0 / (sigma + tao) * 
                    ((2*sigma + tao - lambda) * sqrt(w_x_x_1) * Kn.at<double>(n,x-1) 
                        - sigma * sqrt(w_x_x_2) * Kn.at<double>(n,x-2));    
            // 抑制数值不稳定 
            k = Kn.at<double>(n,x); 
            dk = Kn.at<double>(n,x) - Kn.at<double>(n,x-1);
            ddk = Kn.at<double>(n,x) - 2*Kn.at<double>(n,x-1) + Kn.at<double>(n,x-2);
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

void Krawtchouk_Moments_VS::write_data_moments()
{
    
}
