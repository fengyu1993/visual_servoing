#include "direct_visual_servoing.h"
#include <opencv2/imgproc.hpp>
#include <math.h>

Direct_Visual_Servoing::Direct_Visual_Servoing(int resolution_x, int resolution_y):Visual_Servoing(resolution_x, resolution_y)
{
    this->L_e_ = Mat::zeros(resolution_x*resolution_y, 6, CV_64FC1); 
    this->error_s_ = Mat::zeros(resolution_x*resolution_y, 1, CV_64FC1);
}

// 计算直接视觉伺服特征误差 交互矩阵
void Direct_Visual_Servoing::get_feature_error_interaction_matrix()
{
    this->error_s_ = this->image_gray_current_.reshape(0, this->image_gray_current_.rows*this->image_gray_current_.cols)
                - this->image_gray_desired_.reshape(0, this->image_gray_desired_.rows*this->image_gray_desired_.cols);
    
    Mat Le_new = get_interaction_matrix_gray(this->image_gray_current_, this->image_depth_current_, this->camera_intrinsic_);  
    Mat Le_old = get_interaction_matrix_gray(this->image_gray_desired_, this->image_depth_desired_, this->camera_intrinsic_);
    this->L_e_ = 0.5*(Le_new + Le_old);
}

Mat Direct_Visual_Servoing::get_interaction_matrix_gray(Mat image_gray, Mat image_depth, Mat Camera_Intrinsic)
{
    Mat I_x, I_y;
    int cnt = 0;
    Mat point_image = Mat::ones(3, 1, CV_64FC1);
    Mat xy = Mat::zeros(3, 1, CV_64FC1);
    double x, y;
    double Z_inv;
    Mat L_e = Mat::zeros(image_gray.rows*image_gray.cols, 6, CV_64FC1); 

    get_image_gradient(image_gray, Camera_Intrinsic, I_x, I_y);

    for(int i = 0; i < image_gray.rows; i++)
    {
        point_image.at<double>(1,0) = i;
        for(int j = 0; j < image_gray.cols; j++)
        {
            point_image.at<double>(0,0) = j;
            xy = Camera_Intrinsic.inv() * point_image;
            x = xy.at<double>(0,0);
            y = xy.at<double>(1,0);
            Z_inv = 1.0/image_depth.at<double>(i, j);
            L_e.at<double>(cnt, 0) = I_x.at<double>(i, j)*Z_inv;
            L_e.at<double>(cnt, 1) = I_y.at<double>(i, j)*Z_inv;
            L_e.at<double>(cnt, 2) = -(x*I_x.at<double>(i, j) + y*I_y.at<double>(i, j))*Z_inv;
            L_e.at<double>(cnt, 3) = -x*y*I_x.at<double>(i, j) - (1+y*y)*I_y.at<double>(i, j);
            L_e.at<double>(cnt, 4) = (1+x*x)*I_x.at<double>(i, j) + x*y*I_y.at<double>(i, j);
            L_e.at<double>(cnt, 5) = -y*I_x.at<double>(i, j) + x*I_y.at<double>(i, j);   
            cnt++;        
        }
    }

    return L_e;
}

// 计算图像梯度
void Direct_Visual_Servoing::get_image_gradient(Mat image, Mat Camera_Intrinsic, Mat& I_x, Mat& I_y)
{
    I_x = get_image_gradient_x(image) * Camera_Intrinsic.at<double>(0, 0);
    I_y = get_image_gradient_y(image) * Camera_Intrinsic.at<double>(1, 1);
}

// 计算矩阵x方向上的梯度
Mat Direct_Visual_Servoing::get_image_gradient_x(Mat image)
{
    Mat I_x = Mat::zeros(image.rows, image.cols, CV_64FC1);
    Mat temp = Mat::zeros(image.rows, 1, CV_64FC1);
    int up, down;
    for(int i = 0; i < image.cols; i++)
    {
        up = i+1;
        down = i-1;
        if (up > image.cols-1) 
            up = image.cols-1;
        if (down < 0) 
            down = 0;
        temp = (image.col(up) - image.col(down)) / (up - down); 
        temp.copyTo(I_x.col(i));
    }

    return I_x;
}

// 计算矩阵y方向上的梯度
Mat Direct_Visual_Servoing::get_image_gradient_y(Mat image)
{
    Mat I_y = Mat::zeros(image.rows, image.cols, CV_64FC1);
    Mat temp = Mat::zeros(1, image.cols, CV_64FC1);
    int up, down;
    for(int i = 0; i < image.rows; i++)
    {
        up = i+1;
        down = i-1;
        if (up > image.rows-1) 
            up = image.rows-1;
        if (down < 0) 
            down = 0;
        temp = (image.row(up) - image.row(down)) / (up - down); 
        temp.copyTo(I_y.row(i));
    }
    return I_y;
}


void Direct_Visual_Servoing::save_data_error_feature()
{
    Mat error_ave = (this->error_s_.t() * this->error_s_) / (this->error_s_.rows * this->error_s_.cols);
    this->data_vs.error_feature_.push_back(error_ave);
}

