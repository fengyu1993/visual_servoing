#include "direct_visual_servoing.h"
#include <opencv2/imgproc.hpp>
#include <math.h>

using namespace std;

Direct_Visual_Servoing::Direct_Visual_Servoing(double lambda, double epsilon, Mat camera_intrinsic, int resolution_x=640, int resolution_y=480)
{
    this->lambda_ = lambda;
    this->epsilon_ = epsilon;
    this->resolution_x_ = resolution_x;
    this->resolution_y_ = resolution_y;
    this->image_gray_desired_ = Mat::zeros(this->resolution_y_, this->resolution_x_, CV_64FC1);
    this->image_gray_initial_ = Mat::zeros(this->resolution_y_, this->resolution_x_, CV_64FC1);
    this->image_gray_current_ = Mat::zeros(this->resolution_y_, this->resolution_x_, CV_64FC1);
    this->image_depth_desired_ = Mat::zeros(this->resolution_y_, this->resolution_x_, CV_64FC1);
    this->image_depth_initial_ = Mat::zeros(this->resolution_y_, this->resolution_x_, CV_64FC1);
    this->image_depth_current_ = Mat::zeros(this->resolution_y_, this->resolution_x_, CV_64FC1);
    this->L_e_ = Mat::zeros(this->resolution_x_*this->resolution_y_, 6, CV_64FC1); 
    this->error_s_ = Mat::zeros(this->resolution_x_*this->resolution_y_, 1, CV_64FC1);  
    this->camera_intrinsic_ = camera_intrinsic;
    this->camera_velocity_ = Mat::zeros(6, 1, CV_64FC1);
}

// 计算相机速度
Mat Direct_Visual_Servoing::get_camera_velocity()
{
    Mat L_e_inv;
    get_feature_error_gray();
    get_interaction_matrix_gray();
    invert(this->L_e_, L_e_inv, DECOMP_SVD);
    cout << "L_e = " << L_e_.rows << ", " << L_e_.cols << endl;
    cout << "L_e_inv = " << L_e_inv.rows << ", " << L_e_inv.cols << endl;
    cout << "error_s_ = " << error_s_.rows << ", " << error_s_.cols << endl;
    this->camera_velocity_ = -this->lambda_ * L_e_inv * this->error_s_;
    cout << "cyh" << endl;
    return this->camera_velocity_;
}


// 计算直接视觉伺服特征误差
Mat Direct_Visual_Servoing::get_feature_error_gray()
{
    this->error_s_ = this->image_gray_current_.reshape(0, this->image_gray_current_.rows*this->image_gray_current_.cols)
                - this->image_gray_desired_.reshape(0, this->image_gray_desired_.rows*this->image_gray_desired_.cols);
    
    return this->error_s_;
}

// 计算灰度误差的交互矩阵（直接视觉伺服DVS）
Mat Direct_Visual_Servoing::get_interaction_matrix_gray()
{
    Mat Le_new = get_interaction_matrix(this->image_gray_current_, this->image_depth_current_, this->camera_intrinsic_);
    Mat Le_old = get_interaction_matrix(this->image_gray_desired_, this->image_depth_desired_, this->camera_intrinsic_);
    return 0.5*(Le_new + Le_old);
}

Mat Direct_Visual_Servoing::get_interaction_matrix(Mat image_gray, Mat image_depth, Mat Camera_Intrinsic)
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

    return L_e;
}

// 计算图像梯度
void Direct_Visual_Servoing::get_image_gradient(Mat image, Mat Camera_Intrinsic, Mat& I_x, Mat& I_y)
{
    Sobel(image, I_x, -1, 1, 0);
    Sobel(image, I_y, -1, 0, 1);
    I_x = I_x * Camera_Intrinsic.at<double>(0, 0);
    I_y = I_y * Camera_Intrinsic.at<double>(1, 1);
}

// 设置期望灰度图像
void Direct_Visual_Servoing::set_image_gray_desired(Mat image_gray_desired)
{
    image_gray_desired.copyTo(this->image_gray_desired_);
}

// 设置初始灰度图像
void Direct_Visual_Servoing::set_image_gray_initial(Mat image_gray_initial)
{
    image_gray_initial.copyTo(this->image_gray_initial_);
}

// 设置当前灰度图像
void Direct_Visual_Servoing::set_image_gray_current(Mat image_gray_current)
{
    image_gray_current.copyTo(this->image_gray_current_);
}

// 设置期望深度图像
void Direct_Visual_Servoing::set_image_depth_desired(Mat image_depth_desired)
{
    image_depth_desired.copyTo(this->image_depth_desired_);
}

// 设置初始深度图像
void Direct_Visual_Servoing::set_image_depth_initial(Mat image_depth_initial)
{
    image_depth_initial.copyTo(this->image_depth_initial_);
}

// 设置当前深度图像
void Direct_Visual_Servoing::set_image_depth_current(Mat image_depth_current)
{
    image_depth_current.copyTo(this->image_depth_current_);
}