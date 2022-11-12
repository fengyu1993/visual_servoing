#include "Hahn_moments_vs.h"
#include <opencv2/imgproc.hpp>
#include <math.h>


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

}

void Hahn_Moments_VS::get_Hahn_Moments_parameters()
{
 
}

// 计算Hahn正交多项式 
// Hn = [0 1 2 ... N-1
//       1   .
//       2     .
//       .       .
//       .
//       .
//       n     ...   ]
Mat Hahn_Moments_VS::get_orthogonal_polynomial_HM(int N, int order, double p)
{
  
}

double Hahn_Moments_VS::w_x_x_1_Hahn(int N, int x, double p)
{
    
}

double Hahn_Moments_VS::w_x_x_2_Hahn(int N, int x, double p)
{
    
}

