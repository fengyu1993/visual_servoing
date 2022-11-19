#include "Tchebichef_moments_vs.h"
#include <opencv2/imgproc.hpp>
#include <math.h>
#include <iostream>
#include <fstream>
#include <string>
#include <ctime> 
#include <chrono>


void Techebichef_Moments_VS::get_DOM_matrix()
{
    this->DOM_x_ = get_orthogonal_polynomial_HM(this->N_, this->order_);
    this->DOM_y_ = get_orthogonal_polynomial_HM(this->M_, this->order_);
}

// 计算Tchebichef正交多项式 
// Tn = [0 1 2 ... N-1
//       1   .
//       2     .
//       .       .
//       .
//       .
//       n     ...   ]
Mat Techebichef_Moments_VS::get_orthogonal_polynomial_HM(int N, int order)
{
    // 初始化
    Mat Tn = Mat::zeros(order+1, N, CV_64FC1);
    double lambda=0.0, sigma=0.0, tao=0.0;
    // 计算第1列
    Tn.at<double>(0,0) = sqrt(1.0/N);
    for(int n = 1; n <= order; n++)
    {
        ((double*)Tn.data)[n*Tn.cols] = -sqrt(double(N-n) / double(N+n)) * sqrt(double(2*n+1) / double(2*n-1)) * ((double*)Tn.data)[(n-1)*Tn.cols]; 
    }
    // 计算第2列
    for(int n = 0; n <= order; n++)
    {
        ((double*)Tn.data)[n*Tn.cols+1] = (1 + double(n*(n+1)) / double(1 - N)) * ((double*)Tn.data)[n*Tn.cols];
    }    
    // 计算第3...N列
    for(int n = 0; n <= order; n++)
    {
        lambda = n * (n + 1);
        for(int x = 2; x < N; x++)
        {
            sigma = (x-1) * (N - x + 1);
            tao = N - 1 - 2 * (x-1);
            ((double*)Tn.data)[n*Tn.cols+x] = 1.0 / (sigma + tao) * ((2*sigma + tao - lambda) 
                            * ((double*)Tn.data)[n*Tn.cols+x-1] - sigma * ((double*)Tn.data)[n*Tn.cols+x-2]);           
        }
    }
    return Tn;
}

string Techebichef_Moments_VS::get_method_name()
{
    return "TM_VS";
}