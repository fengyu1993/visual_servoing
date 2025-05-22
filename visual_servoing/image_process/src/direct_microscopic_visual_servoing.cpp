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

    this->Mat_u_ = Mat::zeros(resolution_y, resolution_x, CV_64FC1);
    cv::parallel_for_(cv::Range(0, this->Mat_u_.rows), [&](const cv::Range& range) {
        for (int r = range.start; r < range.end; ++r) {
            double* rowPtr = this->Mat_u_.ptr<double>(r);
            for (int c = 0; c < this->Mat_u_.cols; ++c) {
                rowPtr[c] = c;
            }
        }
    });
    this->Mat_x_ = (this->Mat_u_ - this->camera_intrinsic_.c_u) / this->camera_intrinsic_.k_u;

    this->Mat_v_ = Mat::zeros(resolution_y, resolution_x, CV_64FC1);
    cv::parallel_for_(cv::Range(0, this->Mat_v_.rows), [&](const cv::Range& range) {
        for (int r = range.start; r < range.end; ++r) {
            double* rowPtr = this->Mat_v_.ptr<double>(r);
            for (int c = 0; c < this->Mat_v_.cols; ++c) {
                rowPtr[c] = r;  // 每个元素赋值为行号
            }
        }
    });
    this->Mat_y_ = (this->Mat_v_ - this->camera_intrinsic_.c_v) / this->camera_intrinsic_.k_v;
}

// 计算直接显微视觉伺服特征误差 交互矩阵
void Direct_Microscopic_Visual_Servoing::get_feature_error_interaction_matrix()
{
    this->error_s_ = this->image_gray_current_.reshape(0, this->image_gray_current_.rows*this->image_gray_current_.cols)
                - this->image_gray_desired_.reshape(0, this->image_gray_desired_.rows*this->image_gray_desired_.cols);  
    get_interaction_matrix_gray();
}

void Direct_Microscopic_Visual_Servoing::get_interaction_matrix_gray()
{
    Mat I_x, I_y, I_xx, I_yy, Delta_I;
    get_image_gradient_x(this->image_gray_current_, I_x);
    get_image_gradient_y(this->image_gray_current_, I_y);
    get_image_gradient_x(I_x, I_xx);
    get_image_gradient_y(I_y, I_yy);  
    cv::add(I_xx, I_yy, Delta_I);

    Mat Mat_div_Z = this->A_ * this->Mat_x_ + this->B_ * this->Mat_y_ + this->C_;
    Mat Mat_div_Zf_Z = 1 / this->camera_intrinsic_.Z_f - Mat_div_Z;
    Mat Mat_xIx_yIy = this->Mat_x_ * I_x + this->Mat_y_ * I_y;
    Mat Mat_Phi_Delta_I = this->Phi_ * Delta_I;

    Mat L_Ic_vx = -I_x * Mat_div_Z * this->camera_intrinsic_.D_f;
    Mat L_Ic_vy = -I_y * Mat_div_Z * this->camera_intrinsic_.D_f;
    Mat L_Ic_vz = Mat_xIx_yIy * Mat_div_Z 
                    + this->camera_intrinsic_.D_f * Mat_Phi_Delta_I * Mat_div_Z * Mat_div_Zf_Z;
    Mat L_Ic_wx = this->camera_intrinsic_.D_f * I_y 
                    + Mat_xIx_yIy / this->camera_intrinsic_.D_f * this->Mat_y_
                    + Mat_Phi_Delta_I * this->Mat_y_ * Mat_div_Zf_Z;
    Mat L_Ic_wy = -this->camera_intrinsic_.D_f * I_x
                    - Mat_xIx_yIy / this->camera_intrinsic_.D_f * this->Mat_x_
                    - Mat_Phi_Delta_I * this->Mat_x_ * Mat_div_Zf_Z;
    Mat L_Ic_wz = Mat_y_ * I_x - Mat_x_ * I_y;

    int totalElements = this->image_gray_current_.total();
    cv::Mat L_e_col1 = this->L_e_.col(0);
    cv::Mat L_e_col2 = this->L_e_.col(1);
    cv::Mat L_e_col3 = this->L_e_.col(2);
    cv::Mat L_e_col4 = this->L_e_.col(3);
    cv::Mat L_e_col5 = this->L_e_.col(4);
    cv::Mat L_e_col6 = this->L_e_.col(5);

    // // 使用 parallel_for_ 并行赋值
    cv::parallel_for_(cv::Range(0, totalElements), [&](const cv::Range& range) {
        for (int i = range.start; i < range.end; ++i) {
            L_e_col1.at<double>(i) = L_Ic_vx.at<double>(i);
        }
    });
    cv::parallel_for_(cv::Range(0, totalElements), [&](const cv::Range& range) {
        for (int i = range.start; i < range.end; ++i) {
            L_e_col2.at<double>(i) = L_Ic_vy.at<double>(i);
        }
    });
    cv::parallel_for_(cv::Range(0, totalElements), [&](const cv::Range& range) {
        for (int i = range.start; i < range.end; ++i) {
            L_e_col3.at<double>(i) = L_Ic_vz.at<double>(i);
        }
    });
    cv::parallel_for_(cv::Range(0, totalElements), [&](const cv::Range& range) {
        for (int i = range.start; i < range.end; ++i) {
            L_e_col4.at<double>(i) = L_Ic_wx.at<double>(i);
        }
    });
    cv::parallel_for_(cv::Range(0, totalElements), [&](const cv::Range& range) {
        for (int i = range.start; i < range.end; ++i) {
            L_e_col5.at<double>(i) = L_Ic_wy.at<double>(i);
        }
    });
    cv::parallel_for_(cv::Range(0, totalElements), [&](const cv::Range& range) {
        for (int i = range.start; i < range.end; ++i) {
            L_e_col6.at<double>(i) = L_Ic_wz.at<double>(i);
        }
    });
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
    I_x = I_x / this->camera_intrinsic_.k_u;
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
    I_y = I_y / this->camera_intrinsic_.k_v;
}


string Direct_Microscopic_Visual_Servoing::get_method_name()
{
    return "Direct_Microscopic_Visual_Servoing";
}





