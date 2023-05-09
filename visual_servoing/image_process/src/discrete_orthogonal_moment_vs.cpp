#include "discrete_orthogonal_moment_vs.h"
#include "direct_visual_servoing.h"
#include <opencv2/imgproc.hpp>
#include <math.h>


Discrete_Orthogonal_Moment_VS::Discrete_Orthogonal_Moment_VS
    (int order_min=4, int order_max=8, double delta_epsilon=0.1, double lambda_order=1.2, int resolution_x=640, int resolution_y=480)
    : Direct_Visual_Servoing(resolution_x, resolution_y)
{
    this->N_ = resolution_x; 
    this->M_ = resolution_y;
    this->order_min_ = order_min;
    this->order_max_ = order_max;  
    this->delta_epsilon_ =  delta_epsilon;
    this->lambda_order_ =  lambda_order;
}

// 计算特征误差 交互矩阵
void Discrete_Orthogonal_Moment_VS::get_feature_error_interaction_matrix()
{
    // 准备
    get_order_adaption();
    int num = int((double(this->order_) + 2) * (double(this->order_) + 1) / 2.0);
    this->L_e_ = Mat::zeros(num, 6, CV_64FC1); 
    this->error_s_ = Mat::zeros(num, 1, CV_64FC1); 
    Mat feature_new= Mat::zeros(num, 1, CV_64FC1); 
    Mat feature_old= Mat::zeros(num, 1, CV_64FC1); 
    Mat Le_new =  Mat::zeros(num, 6, CV_64FC1);
    Mat Le_old =  Mat::zeros(num, 6, CV_64FC1);
    Mat DOM_XY;
    int cnt = 0;
    // 计算离散正交矩所需矩阵 DOM_x_ DOM_y_
    get_DOM_matrix();
    // 计算灰度交互矩阵
    Mat L_I_new = get_interaction_matrix_gray(this->image_gray_current_, this->image_depth_current_, this->camera_intrinsic_);
    Mat L_I_old = get_interaction_matrix_gray(this->image_gray_desired_, this->image_depth_desired_, this->camera_intrinsic_);
    // 计算特征 交互矩阵
    for(int l = 0; l <= this->order_; l++)
    {
        for(int k = 0; k <= this->order_; k++)
        {
            if(l + k > this->order_){
                continue;
            }else{
                // 准备            
                DOM_XY = repeat(this->DOM_x_.row(l), this->M_, 1).mul(
                    repeat(this->DOM_y_.row(k).t(), 1, this->N_));                     
                // 计算特征
                ((double*)feature_new.data)[cnt] = sum(DOM_XY.mul(this->image_gray_current_))[0];
                ((double*)feature_old.data)[cnt] = sum(DOM_XY.mul(this->image_gray_desired_))[0];
                // 计算交互矩阵
                get_interaction_matrix_DOM_once(DOM_XY, L_I_new).copyTo(Le_new.row(cnt));
                get_interaction_matrix_DOM_once(DOM_XY, L_I_old).copyTo(Le_old.row(cnt));
                // 计数
                cnt++;
            }
        }
    }   
    // 计算特征误差 交互矩阵
    this->error_s_ = feature_new.rowRange(0, cnt) - feature_old.rowRange(0, cnt);
    this->L_e_ = 0.5*(Le_new.rowRange(0, cnt) + Le_old.rowRange(0, cnt));
}

// 计算每个特征就交互矩阵
Mat Discrete_Orthogonal_Moment_VS::get_interaction_matrix_DOM_once(Mat& DOM_XY, Mat& L_I)
{
    Mat L_once = Mat::zeros(1, L_I.cols, CV_64FC1);
    Mat DOM_XY_Vec = DOM_XY.reshape(0, DOM_XY.rows*DOM_XY.cols);
    Mat matArray[] = { DOM_XY_Vec, DOM_XY_Vec, DOM_XY_Vec, 
                       DOM_XY_Vec, DOM_XY_Vec, DOM_XY_Vec};
    Mat DOM_XY_Mat;
    hconcat(matArray, 6, DOM_XY_Mat); 
    reduce(DOM_XY_Mat.mul(L_I), L_once, 0, REDUCE_SUM);

    return L_once;
}

// 计算离散正交矩所需阶数
int Discrete_Orthogonal_Moment_VS::get_order_adaption()
{
    Mat vec_image_new = this->image_gray_current_.reshape(0, this->N_*this->M_);
    Mat vec_image_old = this->image_gray_desired_.reshape(0, this->N_*this->M_);
    Mat vec_image_init = this->image_gray_initial_.reshape(0, this->N_*this->M_);
    
    Mat err = vec_image_new - vec_image_old;
    Mat err_0 = vec_image_init - vec_image_old;

    Mat error_ave = err.t() * err / (err.rows * err.cols);
    this->error_pixel_ave_ = error_ave.at<double>(0,0);
    double error_pixel_ave = this->error_pixel_ave_;
    Mat error_ave_0 = err_0.t() * err_0 / (err_0.rows * err_0.cols);
    double error_pixel_ave_0 = error_ave_0.at<double>(0,0);

    double t = 1 / (1 + (error_pixel_ave/this->epsilon_) * (error_pixel_ave - this->epsilon_) / (error_pixel_ave_0 - this->epsilon_));
    double order_ = (this->order_max_ - this->order_min_)*t + this->order_min_;
    this->order_ = round(order_);  

    return this->order_;
}

// matlab中linspace函数
Mat Discrete_Orthogonal_Moment_VS::linspace(double begin, double finish, int number) 
{
     	double interval = (finish - begin) / (number - 1);
     	Mat f(1, number, CV_64FC1);
     	for (int j = 0; j < f.cols; j++) 
        {
      		f.at<double>(0,j)=begin+j*interval;
      	}
     	return f;
}
  
// 保存其他参数
void Discrete_Orthogonal_Moment_VS::save_data_other_parameter()
{
    save_data_error_pixel_ave();
    save_data_order();
    save_data_moments_parameter();
}

// 保存图像像素平均误差
void Discrete_Orthogonal_Moment_VS::save_data_error_pixel_ave()
{
    this->data_dom.error_pixel_ave_.push_back(this->error_pixel_ave_);
}

// 保存阶数
void Discrete_Orthogonal_Moment_VS::save_data_order()
{
    this->data_dom.order_list_.push_back(this->order_);
}

void Discrete_Orthogonal_Moment_VS::write_other_data(ofstream& oFile)
{
    write_data_error_pixel_ave(oFile);
    write_data_order(oFile);
    write_data_moments(oFile);
}

void Discrete_Orthogonal_Moment_VS::write_data_error_pixel_ave(ofstream& oFile)
{
    oFile << "error pixel ave" << endl;
    write_to_excel(this->data_dom.error_pixel_ave_, oFile);
}

void Discrete_Orthogonal_Moment_VS::write_data_order(ofstream& oFile)
{
    oFile << "order" << endl;
    write_to_excel(this->data_dom.order_list_, oFile);    
}

string Discrete_Orthogonal_Moment_VS::get_method_name()
{
    return "DOM_VS";
}

// 判断是否伺服成功
bool Discrete_Orthogonal_Moment_VS::is_success()
{
    double error_ave = ((double)*(this->data_dom.error_pixel_ave_.end<double>() - 1));

    // cout << "error_ave = " << error_ave << endl;

    if(error_ave < this->epsilon_)
	{
		cout << "Visual Servoing Success" << endl;
		return true;
	}
	else
	{        
		return false;
	}
}
