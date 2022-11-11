#include "discrete_cosine_transform_vs.h"
#include <opencv2/imgproc.hpp>
#include <math.h>


void Direct_Cosine_Transform_VS::get_DOM_matrix()
{
    this->DOM_x_ = get_orthogonal_polynomial_DCT(this->M_, this->order_);
    this->DOM_y_ = get_orthogonal_polynomial_DCT(this->N_, this->order_);
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
    Mat temp;

    for(int u = 0; u <= order; u++)
    {
        for(int x = 0; x < N; x++)
        {
            Dn.at<double>(u,x) = 1/sqrt(2) * cos(u * (2*x + 1)*M_PI / (2*N));
        }
        temp = Dn.row(u) / norm(Dn.row(u));
        temp.copyTo(Dn.row(u));
    }
    return Dn;
}
