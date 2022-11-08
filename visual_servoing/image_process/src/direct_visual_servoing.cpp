#include "direct_visual_servoing.h"
#include <opencv2/imgproc.hpp>
#include <math.h>

Direct_Visual_Servoing::Direct_Visual_Servoing(double lambda, double epsilon, Mat camera_intrinsic, int resolution_x=640, int resolution_y=480)
{
    this->lambda_ = lambda;
    this->epsilon_ = epsilon;
    this->resolution_x_ = resolution_x;
    this->resolution_y_ = resolution_y;
    this->image_gray_desired_ = Mat::zeros(this->resolution_x_, this->resolution_y_, CV_64FC1);
    this->image_gray_initial_ = Mat::zeros(this->resolution_x_, this->resolution_y_, CV_64FC1);
    this->image_gray_current_ = Mat::zeros(this->resolution_x_, this->resolution_y_, CV_64FC1);
    this->image_depth_desired_ = Mat::zeros(this->resolution_x_, this->resolution_y_, CV_64FC1);
    this->image_depth_initial_ = Mat::zeros(this->resolution_x_, this->resolution_y_, CV_64FC1);
    this->image_depth_current_ = Mat::zeros(this->resolution_x_, this->resolution_y_, CV_64FC1);
    this->L_e_ = Mat::zeros(6, this->resolution_x_*this->resolution_y_, CV_64FC1); 
    this->error_s_ = Mat::zeros(1, this->resolution_x_*this->resolution_y_, CV_64FC1);  
    this->camera_intrinsic_ = camera_intrinsic;
}

// 计算特征误差和交互矩阵 直接伺服伺服(DVS)
void Direct_Visual_Servoing::get_FeaturesError_InteractionMatrix_DVS
        (Mat image_gray_new, Mat image_depth_new, Mat image_gray_old, Mat image_depth_old, 
        Mat Camera_Intrinsic, Mat &error_s, Mat &L_e)
{
    Mat L_e_new;
    Mat L_e_old;
    // 计算特征误差
    get_feature_error_gray(image_gray_new, image_gray_old, error_s);
    // 计算交互矩阵
    get_interaction_matrix_gray_depth(image_gray_new, image_depth_new, Camera_Intrinsic, L_e_new);
    get_interaction_matrix_gray_depth(image_gray_old, image_depth_old, Camera_Intrinsic, L_e_old);
    L_e = 0.5*(L_e_new + L_e_old);
}

// 计算直接视觉伺服特征误差
void Direct_Visual_Servoing::get_feature_error_gray(Mat image_gray_new, Mat image_gray_old, Mat &error_s)
{
    error_s = image_gray_new.reshape(0, image_gray_new.rows*image_gray_new.cols)
                - image_gray_old.reshape(0, image_gray_old.rows*image_gray_old.cols);
}

// 计算灰度误差的交互矩阵（直接视觉伺服DVS）
void Direct_Visual_Servoing::get_interaction_matrix_gray_depth(Mat image_gray, Mat image_depth, Mat Camera_Intrinsic, Mat& L_e)
{
    Mat I_x, I_y;
    int cnt = 0;
    Mat point_image = Mat::ones(1, 3, CV_64FC1);
    Mat xy = Mat::zeros(1, 3, CV_64FC1);
    double x, y;
    double Z_inv;

    get_image_gradient(image_gray, Camera_Intrinsic, I_x, I_y);
    for(int i = 0; i < image_gray.rows; i++)
    {
        point_image.at<double>(0,1) = i;
        for(int j = 0; j < image_gray.cols; j++)
        {
            point_image.at<double>(0,0) = j;
            xy = Camera_Intrinsic.inv() * point_image;
            x = xy.at<double>(0,0);
            y = xy.at<double>(0,1);
            Z_inv = 1.0/image_depth.at<double>(j, i);
            L_e.at<double>(0, cnt) = I_x.at<double>(j, i)*Z_inv;
            L_e.at<double>(1, cnt) = I_y.at<double>(j, i)*Z_inv;
            L_e.at<double>(2, cnt) = -(x*I_x.at<double>(j, i) + y*I_y.at<double>(j, i))*Z_inv;
            L_e.at<double>(3, cnt) = -x*y*I_x.at<double>(j, i) - (1+y*y)*I_y.at<double>(j, i);
            L_e.at<double>(4, cnt) = (1+x*x)*I_x.at<double>(j, i) + x*y*I_y.at<double>(j, i);
            L_e.at<double>(5, cnt) = -y*I_x.at<double>(j, i) + x*I_y.at<double>(j, i);   
            cnt++;        
        }
    }
}
// %% 计算灰度误差的交互矩阵（直接视觉伺服DVS）
// for i = 1 : col
//     for j = 1 : row
//         point = [i; j; 1];
//         corn_right_norm = Para_Camera_Intrinsic \ point;
//         x = corn_right_norm(1);
//         y = corn_right_norm(2);
//         Z_inv = 1 / image_depth(j,i);
//         L_I(cnt, :) = [I_x(j,i)*Z_inv, ...
//                        I_y(j,i)*Z_inv, ...
//                        -(x*I_x(j,i) + y*I_y(j,i))*Z_inv, ...
//                        -x*y*I_x(j,i) - (1+y^2)*I_y(j,i), ...
//                        (1+x^2)*I_x(j,i) + x*y*I_y(j,i), ...
//                        -y*I_x(j,i) + x*I_y(j,i)];
//         cnt = cnt + 1;
//     end
// end




// 计算图像梯度
void Direct_Visual_Servoing::get_image_gradient(Mat image, Mat Camera_Intrinsic, Mat& I_x, Mat& I_y)
{
    Sobel(image, I_x, -1, 1, 0);
    Sobel(image, I_y, -1, 0, 1);
    I_x = I_x * Camera_Intrinsic.at<double>(0, 0);
    I_y = I_y * Camera_Intrinsic.at<double>(1, 1);
}
