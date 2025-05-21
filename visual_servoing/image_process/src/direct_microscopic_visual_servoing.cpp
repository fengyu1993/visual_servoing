#include "direct_microscopic_visual_servoing.h"
#include <opencv2/imgproc.hpp>
#include <math.h>
#include <iostream>
#include <fstream>
#include <string>
#include <ctime> 
#include <chrono>

Direct_Microscopic_Visual_Servoing::Direct_Microscopic_Visual_Servoing(int resolution_x, int resolution_y) : Microscopic_Visual_Servoing(resolution_x, resolution_y)
{
    this->L_e_ = Mat::zeros(resolution_x*resolution_y, 6, CV_64FC1); 
    this->error_s_ = Mat::zeros(resolution_x*resolution_y, 1, CV_64FC1);
}

// 计算直接显微视觉伺服特征误差 交互矩阵
void Direct_Microscopic_Visual_Servoing::get_feature_error_interaction_matrix()
{
    this->error_s_ = this->image_gray_current_.reshape(0, this->image_gray_current_.rows*this->image_gray_current_.cols)
                - this->image_gray_desired_.reshape(0, this->image_gray_desired_.rows*this->image_gray_desired_.cols);  
    this->L_e_ = get_interaction_matrix_gray();
}

Mat Direct_Microscopic_Visual_Servoing::get_interaction_matrix_gray()
{
    Mat I_x, I_y;
    int cnt = 0;
    Mat point_image = Mat::ones(3, 1, CV_64FC1);
    Mat xy = Mat::zeros(3, 1, CV_64FC1);
    double x, y, I_x_temp, I_y_temp;
    double Z_inv;
    Mat L_e = Mat::zeros(this->image_gray_current_.rows * this->image_gray_current_.cols, 6, CV_64FC1); 

    // get_image_gradient(image_gray, Camera_Intrinsic, I_x, I_y);

    // for(int i = 0; i < image_gray.rows; i++)
    // {
    //     point_image.at<double>(1,0) = i;
    //     for(int j = 0; j < image_gray.cols; j++)
    //     {
    //         point_image.at<double>(0,0) = j;
    //         xy = Camera_Intrinsic.inv() * point_image;
    //         x = ((double*)xy.data)[0];
    //         y = ((double*)xy.data)[1];
    //         Z_inv = 1.0/image_depth.at<double>(i, j);
    //         I_x_temp = ((double*)I_x.data)[i*I_x.cols+j];
    //         I_y_temp = ((double*)I_y.data)[i*I_y.cols+j];
    //         ((double*)L_e.data)[cnt*6+0] = I_x_temp*Z_inv;
    //         ((double*)L_e.data)[cnt*6+1] = I_y_temp*Z_inv;
    //         ((double*)L_e.data)[cnt*6+2] = -(x*I_x_temp + y*I_y_temp)*Z_inv;
    //         ((double*)L_e.data)[cnt*6+3] = -x*y*I_x_temp - (1+y*y)*I_y_temp;
    //         ((double*)L_e.data)[cnt*6+4] = (1+x*x)*I_x_temp + x*y*I_y_temp;
    //         ((double*)L_e.data)[cnt*6+5] = -y*I_x_temp + x*I_y_temp;  
    //         cnt++;        
    //     }
    // }

    return L_e;
}

// // 计算图像梯度
// void Direct_Microscopic_Visual_Servoing::get_image_gradient(Mat& image, Mat& Camera_Intrinsic, Mat& I_x, Mat& I_y)
// {
//     I_x = get_image_gradient_x(image) * Camera_Intrinsic.at<double>(0, 0);
//     I_y = get_image_gradient_y(image) * Camera_Intrinsic.at<double>(1, 1);
// }

// // 计算矩阵x方向上的梯度
// Mat Direct_Microscopic_Visual_Servoing::get_image_gradient_x(Mat& image)
// {
//     Mat I_x = Mat::zeros(image.rows, image.cols, CV_64FC1);
//     int up, down;
//     for(int i = 0; i < image.cols; i++)
//     {
//         up = i+1;
//         down = i-1;
//         if (up > image.cols-1) 
//             up = image.cols-1;
//         if (down < 0) 
//             down = 0;
//         I_x.col(i) = (image.col(up) - image.col(down)) / (up - down); 
//     }
//     return I_x;
// }

// // 计算矩阵y方向上的梯度
// Mat Direct_Microscopic_Visual_Servoing::get_image_gradient_y(Mat& image)
// {
//     Mat I_y = Mat::zeros(image.rows, image.cols, CV_64FC1);
//     int up, down;
//     for(int i = 0; i < image.rows; i++)
//     {
//         up = i+1;
//         down = i-1;
//         if (up > image.rows-1) 
//             up = image.rows-1;
//         if (down < 0) 
//             down = 0;
//         I_y.row(i) = (image.row(up) - image.row(down)) / (up - down); 
//     }
//     return I_y;
// }


string Direct_Microscopic_Visual_Servoing::get_method_name()
{
    return "Direct_Microscopic_Visual_Servoing";
}





