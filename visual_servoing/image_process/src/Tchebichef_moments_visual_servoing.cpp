#include "Tchebichef_moments_visual_servoing.h"
#include <opencv2/imgproc.hpp>
#include <math.h>


void Techebichef_Moments_VS::get_DOM_matrix()
{
    this->DOM_x_ = get_orthogonal_polynomial_HM(this->N_, this->order_);
    this->DOM_y_ = get_orthogonal_polynomial_HM(this->M_, this->order_);
}

// 计算Tchebichef正交多项式 
// Hn = [0 1 2 ... N-1
//       1   .
//       2     .
//       .       .
//       .
//       .
//       n     ...   ]
Mat Techebichef_Moments_VS::get_orthogonal_polynomial_HM(int N, int order)
{
    Mat Hn = Mat::zeros(order+1, N, CV_64FC1);

    return Hn;
}
