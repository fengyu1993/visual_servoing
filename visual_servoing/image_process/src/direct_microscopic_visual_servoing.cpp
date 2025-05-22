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

    this->div_col_ = Mat::ones(resolution_y, resolution_x, CV_64FC1) * 2;
    this->div_col_.col(0).setTo(cv::Scalar(1.0));
    this->div_col_.col(this->div_col_.cols-1).setTo(cv::Scalar(1.0));

    this->div_row_ = Mat::ones(resolution_y, resolution_x, CV_64FC1) * 2;
    this->div_row_.row(0).setTo(cv::Scalar(1.0));
    this->div_row_.row(this->div_row_.rows-1).setTo(cv::Scalar(1.0));

    this->Phi_ = (pow(this->camera_intrinsic_.R_f,2) * this->camera_intrinsic_.D_f) / (9*this->camera_intrinsic_.Z_f);
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
    int cnt = 0;
    Mat point_image = Mat::ones(3, 1, CV_64FC1);
    Mat xy = Mat::zeros(3, 1, CV_64FC1);
    double x, y, I_x_temp, I_y_temp;
    double Z_inv;
    Mat L_e = Mat::zeros(this->image_gray_current_.rows * this->image_gray_current_.cols, 6, CV_64FC1); 

    Mat I_x, I_y, I_xx, I_yy, Delta_I;
    get_image_gradient_x(this->image_gray_current_, I_x);
    get_image_gradient_y(this->image_gray_current_, I_y);
    get_image_gradient_x(I_x, I_xx);
    get_image_gradient_y(I_y, I_yy);  
    cv::add(I_xx, I_yy, Delta_I);


    

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

// 计算矩阵x方向上的梯度
void Direct_Microscopic_Visual_Servoing::get_image_gradient_x(const Mat& image, Mat& I_x)
{
    I_x = cv::Mat::zeros(image.rows, image.cols, CV_64FC1);
    // 中间部分：(i+1)列 - (i-1)列
    cv::subtract(image.colRange(2, image.cols), image.colRange(0, image.cols - 2), I_x.colRange(1, image.cols - 1)); // 中间部分直接相减
    cv::subtract(image.col(1), image.col(0), I_x.col(0));
    cv::subtract(image.col(image.cols - 1), image.col(image.cols - 2), I_x.col(image.cols - 1));
    cv::divide(I_x, this->div_col_, I_x);
}

// 计算矩阵y方向上的梯度
void Direct_Microscopic_Visual_Servoing::get_image_gradient_y(const Mat& image, Mat& I_y)
{
    I_y = cv::Mat::zeros(image.rows, image.cols, CV_64FC1);
    // 中间部分：(i+1)行 - (i-1)行
    cv::subtract(image.rowRange(2, image.rows), image.rowRange(0, image.rows - 2), I_y.rowRange(1, image.rows - 1)); // 中间部分直接相减
    cv::subtract(image.row(1), image.row(0), I_y.row(0));
    cv::subtract(image.row(image.rows - 1), image.row(image.rows - 2), I_y.row(image.rows - 1));
    cv::divide(I_y, this->div_row_, I_y);
}


string Direct_Microscopic_Visual_Servoing::get_method_name()
{
    return "Direct_Microscopic_Visual_Servoing";
}





