#include "discrete_cosine_transform_vs.h"
#include <opencv2/imgproc.hpp>
#include <math.h>
#include <iostream>
#include <fstream>
#include <string>
#include <ctime> 
#include <chrono>

void Direct_Cosine_Transform_VS::get_DOM_matrix()
{
    this->DOM_x_ = get_orthogonal_polynomial_DCT(this->N_, this->order_);
    this->DOM_y_ = get_orthogonal_polynomial_DCT(this->M_, this->order_);
}

// 计算DCT正交多项式 
// Dn = [0 1 2 ... N-1
//       1   .
//       2     .
//       .       .
//       .
//       .
//       n     ...   ]
Mat Direct_Cosine_Transform_VS::get_orthogonal_polynomial_DCT(int N, int order)
{
    Mat Dn = Mat::zeros(order+1, N, CV_64FC1);

    for(int u = 0; u <= order; u++)
    {
        for(int x = 0; x < N; x++)
        {
            ((double*)Dn.data)[u*Dn.cols+x] = 1.0/sqrt(2.0) * cos(double(u * (2*x + 1)*M_PI) / double(2*N));
        }
        Dn.row(u) = Dn.row(u) / norm(Dn.row(u));
    }
    return Dn;
}


string Direct_Cosine_Transform_VS::get_method_name()
{
    return "DCT_VS";
}