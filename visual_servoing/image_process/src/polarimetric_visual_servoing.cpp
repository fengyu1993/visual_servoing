#include "polarimetric_visual_servoing.h"
#include <opencv2/imgproc.hpp>
#include <math.h>
#include <iostream>
#include <fstream>
#include <string>
#include <ctime> 
#include <chrono>
#include <cmath>


Polarimetric_Visual_Servoing::Polarimetric_Visual_Servoing(int resolution_x=640, int resolution_y=480)
{
    this->resolution_x_ = resolution_x;
    this->resolution_y_ = resolution_y;

    this->image_I_0_desired_  = Mat::zeros(this->resolution_y_, this->resolution_x_, CV_64FC1);
    this->image_I_45_desired_ = Mat::zeros(this->resolution_y_, this->resolution_x_, CV_64FC1);
    this->image_I_90_desired_ = Mat::zeros(this->resolution_y_, this->resolution_x_, CV_64FC1);
    this->image_I_135_desired_= Mat::zeros(this->resolution_y_, this->resolution_x_, CV_64FC1);

    this->image_I_0_current_  = Mat::zeros(this->resolution_y_, this->resolution_x_, CV_64FC1);
    this->image_I_45_current_ = Mat::zeros(this->resolution_y_, this->resolution_x_, CV_64FC1);
    this->image_I_90_current_ = Mat::zeros(this->resolution_y_, this->resolution_x_, CV_64FC1);
    this->image_I_135_current_= Mat::zeros(this->resolution_y_, this->resolution_x_, CV_64FC1);
 
    this->image_depth_desired_ = Mat::zeros(this->resolution_y_, this->resolution_x_, CV_64FC1);
    this->image_depth_current_ = Mat::zeros(this->resolution_y_, this->resolution_x_, CV_64FC1);

    this->camera_intrinsic_ = Mat::zeros(3, 3, CV_64FC1);
    this->camera_velocity_ = Mat::zeros(6, 1, CV_64FC1);
	this->flag_first_ = true;
	this->iteration_num_ = 0;
}


// 初始化
void Polarimetric_Visual_Servoing::init_VS(double lambda, double epsilon, 
            Mat& image_I_0_desired, Mat& image_I_45_desired, Mat& image_I_90_desired, Mat& image_I_135_desired, Mat& image_depth_desired,   
            Mat& image_I_0_initial, Mat& image_I_45_initial, Mat& image_I_90_initial, Mat& image_I_135_initial, Mat camera_intrinsic, Mat pose_desired)
{
    this->lambda_ = lambda;
    this->epsilon_ = epsilon;
    set_camera_intrinsic(camera_intrinsic);
    set_image_depth_desired(image_depth_desired);
    set_image_gray_desired(image_I_0_desired, image_I_45_desired, image_I_90_desired, image_I_135_desired);
    set_image_gray_initial(image_I_0_initial, image_I_45_initial, image_I_90_initial, image_I_135_initial);
    set_pose_desired(pose_desired);
    save_data_image();
    save_pose_desired();
}

// 计算L_kappa
Mat Polarimetric_Visual_Servoing::get_L_kappa(Mat& camera_intrinsic)
{
    Mat L_kappa = (Mat_<double>(2,2) << camera_intrinsic.at<double>(0,0), 0, 0, camera_intrinsic.at<double>(1,1));
    return L_kappa;
}

// 计算偏振参数
void Polarimetric_Visual_Servoing::get_O_A_Phi(Mat I_0, Mat I_45, Mat I_90, Mat I_135, Mat& O, Mat& A, Mat& Phi)
{
    O = (I_0 + I_45 + I_90 + I_135) / 4;
    Mat err_I0 = I_0 - O;
    Mat err_I45 = I_45 - O;
    Mat err_I_90 = I_90 - O;
    Mat err_I135 = I_135 - O;
    sqrt((err_I0.mul(err_I0) + err_I45.mul(err_I45) + err_I_90.mul(err_I_90) + err_I135.mul(err_I135)) / 2, A);
    Mat temp_1 = (I_0 - O) / A;
    Mat temp_2 = (I_45 - O) / A;
    Mat temp_3 = (O - I_90) / A;
    Mat temp_4 = (O - I_135) / A;
    Mat B = (Mat_<double>(2,2) << 0, 1, 0.5, 0.2);
    Mat C = cv_asin(B);
    cout << "B" << endl << B << endl;
    cout << "C" << endl << C << endl;

    // Phi = 1/4 * (std::acos(temp_1) + std::asin(temp_2) + std::acos(temp_3) + std::asin(temp_4));
}


//     temp_1 = (I_0 - O) ./ A; id_1 = temp_1 > 1; temp_1(id_1) = 1; id_2 = temp_1 < -1; temp_1(id_2) = -1;
//     temp_2 = (I_45 - O) ./ A; id_1 = temp_2 > 1; temp_2(id_1) = 1; id_2 = temp_2 < -1; temp_2(id_2) = -1;
//     temp_3 = (O - I_90) ./ A; id_1 = temp_3 > 1; temp_3(id_1) = 1; id_2 = temp_3 < -1; temp_3(id_2) = -1;
//     temp_4 = (O - I_135) ./ A; id_1 = temp_4 > 1; temp_4(id_1) = 1; id_2 = temp_4 < -1; temp_4(id_2) = -1;   
//     Phi = 1/4 * (acos(temp_1) + asin(temp_2) + acos(temp_3) + asin(temp_4));
// end

Mat Polarimetric_Visual_Servoing::cv_acos(Mat a)
{
    Mat dst = Mat::zeros(a.size(), a.type());
    int rows = a.rows;
    int cols = a.cols;
    for ( int row = 0; row < rows; row++)
    {
        double* current_a = a.ptr<double>(row);
        double* current_dst = dst.ptr<double>(row);
        for (int col = 0; col< cols; col++)
        {
            current_dst[col] = std::acos(current_a[col]);
        }
    }
    return dst;
}

Mat Polarimetric_Visual_Servoing::cv_asin(Mat a)
{
    Mat dst = Mat::zeros(a.size(), a.type());
    int rows = a.rows;
    int cols = a.cols;
    for ( int row = 0; row < rows; row++)
    {
        double* current_a = a.ptr<double>(row);
        double* current_dst = dst.ptr<double>(row);
        for (int col = 0; col< cols; col++)
        {
            current_dst[col] = std::asin(current_a[col]);
        }
    }
    return dst;
}



// 计算相机速度
Mat Polarimetric_Visual_Servoing::get_camera_velocity()
{
	// 计算相机速度
    Mat L_e_inv;
	this->iteration_num_++;
    get_feature_error_interaction_matrix(); 
    invert(this->L_e_, L_e_inv, DECOMP_SVD);
    this->camera_velocity_ = -this->lambda_ * L_e_inv * this->error_s_;
    return this->camera_velocity_;
}

// 判断是否伺服成功
bool Polarimetric_Visual_Servoing::is_success()
{
	Mat error_ave = this->error_s_.t() * this->error_s_ / (this->error_s_.rows*this->error_s_.cols);

	if(error_ave.at<double>(0,0) < this->epsilon_)
	{
		cout << "Visual Servoing Success" << endl;
		return true;
	}
	else
	{
		return false;
	}
}

// 设置相机内参
void Polarimetric_Visual_Servoing::set_camera_intrinsic(Mat& camera_intrinsic)
{
    camera_intrinsic.copyTo(this->camera_intrinsic_);
}

// 设置期望灰度图像
void Polarimetric_Visual_Servoing::set_image_gray_desired(Mat& image_I_0_desired, Mat& image_I_45_desired, Mat& image_I_90_desired, Mat& image_I_135_desired)
{
    image_I_0_desired.copyTo(this->image_I_0_desired_); 
    image_I_45_desired.copyTo(this->image_I_45_desired_); 
    image_I_90_desired.copyTo(this->image_I_90_desired_); 
    image_I_135_desired.copyTo(this->image_I_135_desired_); 
}

// 设置当前灰度图像
void Polarimetric_Visual_Servoing::set_image_gray_current(Mat& image_I_0_current, Mat& image_I_45_current, Mat& image_I_90_current, Mat& image_I_135_current)

{
    image_I_0_current.copyTo(this->image_I_0_current_);
    image_I_45_current.copyTo(this->image_I_45_current_);
    image_I_90_current.copyTo(this->image_I_90_current_);
    image_I_135_current.copyTo(this->image_I_135_current_);
}

// 设置初始图像
void Polarimetric_Visual_Servoing::set_image_gray_initial(Mat& image_I_0_initial, Mat& image_I_45_initial, Mat& image_I_90_initial, Mat& image_I_135_initial)
{
    image_I_0_initial.copyTo(this->image_I_0_initial_);
    image_I_45_initial.copyTo(this->image_I_45_initial_);
    image_I_90_initial.copyTo(this->image_I_90_initial_);
    image_I_135_initial.copyTo(this->image_I_135_initial_);
}

// // 设置期望深度图像
void Polarimetric_Visual_Servoing::set_image_depth_desired(Mat& image_depth_desired)
{
    image_depth_desired.copyTo(this->image_depth_desired_);
}

// 设置当前深度图像
void Polarimetric_Visual_Servoing::set_image_depth_current(Mat& image_depth_current)
{
    image_depth_current.copyTo(this->image_depth_current_);
}

// // 保存期望位姿
void Polarimetric_Visual_Servoing::set_pose_desired(Mat& pose_desired)
{
	pose_desired.copyTo(this->pose_desired_);
}


// 保存图像数据
void Polarimetric_Visual_Servoing::save_pose_desired()
{
    this->pose_desired_.copyTo(this->data_pvs.pose_desired_);   
}

// 保存图像数据
void Polarimetric_Visual_Servoing::save_data_image()
{
    save_data_image_gray_desired();
    save_data_image_gray_initial();
}

// 保存期望图像
void Polarimetric_Visual_Servoing::save_data_image_gray_desired()
{
    this->image_I_0_desired_.copyTo(this->data_pvs.image_I_0_desired_);
    this->image_I_45_desired_.copyTo(this->data_pvs.image_I_45_desired_);
    this->image_I_90_desired_.copyTo(this->data_pvs.image_I_90_desired_);
    this->image_I_135_desired_.copyTo(this->data_pvs.image_I_135_desired_);
}

// 保存初始图像
void Polarimetric_Visual_Servoing::save_data_image_gray_initial()
{
    this->image_I_0_initial_.copyTo(this->data_pvs.image_I_0_initial_);
    this->image_I_45_initial_.copyTo(this->data_pvs.image_I_45_initial_);
    this->image_I_90_initial_.copyTo(this->data_pvs.image_I_90_initial_);
    this->image_I_135_initial_.copyTo(this->data_pvs.image_I_135_initial_);
}

// 保存相机速度
void Polarimetric_Visual_Servoing::save_data_camera_velocity()
{
    this->data_pvs.velocity_.push_back(this->camera_velocity_.t());
}

// // 保存特征误差
// void Polarimetric_Visual_Servoing::save_data_error_feature()
// {
//     this->data_pvs.error_feature_.push_back(this->error_s_);
// }

// 保存相机位姿
void Polarimetric_Visual_Servoing::save_data_camera_pose(Mat& pose)
{
    this->data_pvs.pose_.push_back(pose);
}

// 保存所有数据
void Polarimetric_Visual_Servoing::save_data(Mat pose)
{
    save_data_camera_velocity();
    save_data_camera_pose(pose);
    save_data_error_feature(); 
	save_data_vs_time();
}

// 记录时间
void Polarimetric_Visual_Servoing::save_data_vs_time()
{
	
	if (this->iteration_num_ == 1)
	{
		this->start_VS_time_ = clock();
		this->data_pvs.time_vs_.push_back(0.0);
	}
	else
	{
		this->data_pvs.time_vs_.push_back((double)(clock() - this->start_VS_time_) / CLOCKS_PER_SEC);
	}
}


// 将数据保存在文件中
void Polarimetric_Visual_Servoing::write_data()
{
    string file_name = get_save_file_name();
    string location = "/home/cyh/Work/visual_servoing_ws/src/visual_servoing/image_process/resource/data/";
    // 保存图像
    string saveImage_desired_I0 = location + file_name + "_desired_image_I0.png";
    string saveImage_desired_I45 = location + file_name + "_desired_image_I45.png";
    string saveImage_desired_I90 = location + file_name + "_desired_image_I90.png";
    string saveImage_desired_I135 = location + file_name + "_desired_image_I135.png";
    string saveImage_initial_I0 = location + file_name + "_initial_image_I0.png";
    string saveImage_initial_I45 = location + file_name + "_initial_image_I45.png";
    string saveImage_initial_I90 = location + file_name + "_initial_image_I90.png";
    string saveImage_initial_I135 = location + file_name + "_initial_image_I135.png";
    imwrite(saveImage_desired_I0, this->data_pvs.image_I_0_desired_*255);
    imwrite(saveImage_desired_I45, this->data_pvs.image_I_45_desired_*255);
    imwrite(saveImage_desired_I90, this->data_pvs.image_I_90_desired_*255);
    imwrite(saveImage_desired_I135, this->data_pvs.image_I_135_desired_*255);
    imwrite(saveImage_initial_I0, this->data_pvs.image_I_0_initial_*255);
    imwrite(saveImage_initial_I45, this->data_pvs.image_I_45_initial_*255);
    imwrite(saveImage_initial_I90, this->data_pvs.image_I_90_initial_*255);
    imwrite(saveImage_initial_I135, this->data_pvs.image_I_135_initial_*255);
    // 保存数据
    ofstream oFile;
	string excel_name = location + file_name + "_data.xls";
    oFile.open(excel_name, ios::out|ios::trunc);
    write_visual_servoing_data(oFile);
    // 关闭文件
    oFile.close();
}

// 写入基本视觉伺服数据到文件
void Polarimetric_Visual_Servoing::write_visual_servoing_data(ofstream& oFile)
{
    oFile << "camera velocity" << endl;
    write_to_excel(this->data_pvs.velocity_, oFile);
    oFile << "camera pose" << endl;
    write_to_excel(this->data_pvs.pose_, oFile);
    oFile << "camera desired pose" << endl;
    write_to_excel(this->data_pvs.pose_desired_, oFile);   
    oFile << "error feature" << endl;
    write_to_excel(this->data_pvs.error_feature_, oFile);
    oFile << "visual servoing time" << endl;
    write_to_excel(this->data_pvs.time_vs_, oFile);	
}

// 存储数据文件命名
string Polarimetric_Visual_Servoing::get_save_file_name()
{
    return get_date_time() + "_" + get_method_name();
}

// 视觉伺服方法名字
string Polarimetric_Visual_Servoing::get_method_name()
{
    return "Polarimetric_Visual_Servoing";
}

// 获取当前计算机时间
string Polarimetric_Visual_Servoing::get_date_time()
{
	auto to_string = [](const std::chrono::system_clock::time_point& t)->std::string
	{
		auto as_time_t = std::chrono::system_clock::to_time_t(t);
		struct tm tm;
#if defined(WIN32) || defined(_WINDLL)
		localtime_s(&tm, &as_time_t);  //win api，线程安全，而std::localtime线程不安全
#else
		localtime_r(&as_time_t, &tm);//linux api，线程安全
#endif

		std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(t.time_since_epoch());
		char buf[128];
		snprintf(buf, sizeof(buf), "%04d_%02d_%02d_%02d_%02d_%02d",
			tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
		return buf;
	};

	std::chrono::system_clock::time_point t = std::chrono::system_clock::now();
	return to_string(t);
}

void Polarimetric_Visual_Servoing::write_to_excel(Mat data, ofstream& oFile)
{
		int channels = data.channels();            //获取图像channel  
		int nrows = data.rows;                     //矩阵的行数  
		int ncols = data.cols*channels;            //矩阵的总列数=列数*channel分量数  
 
		//循环用变量
		int i = 0;
		int j = 0;
 
		if (data.depth() == CV_8U)//uchar
		{
			for (i = 0; i<nrows; i++)
			{
				for (j = 0; j<ncols; j++)
				{
					int tmpVal = (int)data.ptr<uchar>(i)[j];
					oFile << tmpVal << '\t';
				}
				oFile << endl;
			}
		}
		else if (data.depth() == CV_16S)//short
		{
			for (i = 0; i<nrows; i++)
			{
				for (j = 0; j<ncols; j++)
				{
					oFile << (short)data.ptr<short>(i)[j] << '\t';
				}
				oFile << endl;
			}
		}
		else if (data.depth() == CV_16U)//unsigned short
		{
			for (i = 0; i<nrows; i++)
			{
				for (j = 0; j<ncols; j++)
				{
					oFile << (unsigned short)data.ptr<unsigned short>(i)[j] << '\t';
				}
				oFile << endl;
			}
		}
		else if (data.depth() == CV_32S)//int 
		{
			for (i = 0; i<nrows; i++)
			{
				for (j = 0; j<ncols; j++)
				{
					oFile << (int)data.ptr<int>(i)[j] << '\t';
				}
				oFile << endl;
			}
		}
		else if (data.depth() == CV_32F)//float
		{
			for (i = 0; i<nrows; i++)
			{
				for (j = 0; j<ncols; j++)
				{
					oFile << (float)data.ptr<float>(i)[j] << '\t';
				}
				oFile << endl;
			}
		}
		else//CV_64F double
		{
			for (i = 0; i < nrows; i++)
			{
				for (j = 0; j < ncols; j++)
				{
					oFile << (double)data.ptr<double>(i)[j] << '\t';
				}
				oFile << endl;
			}
		}
}

void Polarimetric_Visual_Servoing::get_feature_error_interaction_matrix()
{
    // // 计算特征误差 交互矩阵
    // void Discrete_Orthogonal_Moment_VS::get_feature_error_interaction_matrix()
    // {
    //     // 准备
    //     get_order_adaption();
    //     int num = int((double(this->order_) + 2) * (double(this->order_) + 1) / 2.0);
    //     this->L_e_ = Mat::zeros(num, 6, CV_64FC1); 
    //     this->error_s_ = Mat::zeros(num, 1, CV_64FC1); 
    //     Mat feature_new= Mat::zeros(num, 1, CV_64FC1); 
    //     Mat feature_old= Mat::zeros(num, 1, CV_64FC1); 
    //     Mat Le_new =  Mat::zeros(num, 6, CV_64FC1);
    //     Mat Le_old =  Mat::zeros(num, 6, CV_64FC1);
    //     Mat DOM_XY;
    //     int cnt = 0;
    //     // 计算离散正交矩所需矩阵 DOM_x_ DOM_y_
    //     get_DOM_matrix();
    //     // 计算灰度交互矩阵
    //     Mat L_I_new = get_interaction_matrix_gray(this->image_gray_current_, this->image_depth_current_, this->camera_intrinsic_);
    //     Mat L_I_old = get_interaction_matrix_gray(this->image_gray_desired_, this->image_depth_desired_, this->camera_intrinsic_);
    //     // 计算特征 交互矩阵
    //     for(int l = 0; l <= this->order_; l++)
    //     {
    //         for(int k = 0; k <= this->order_; k++)
    //         {
    //             if(l + k > this->order_){
    //                 continue;
    //             }else{
    //                 // 准备            
    //                 DOM_XY = repeat(this->DOM_x_.row(l), this->M_, 1).mul(
    //                     repeat(this->DOM_y_.row(k).t(), 1, this->N_));                     
    //                 // 计算特征
    //                 ((double*)feature_new.data)[cnt] = sum(DOM_XY.mul(this->image_gray_current_))[0];
    //                 ((double*)feature_old.data)[cnt] = sum(DOM_XY.mul(this->image_gray_desired_))[0];
    //                 // 计算交互矩阵
    //                 get_interaction_matrix_DOM_once(DOM_XY, L_I_new).copyTo(Le_new.row(cnt));
    //                 get_interaction_matrix_DOM_once(DOM_XY, L_I_old).copyTo(Le_old.row(cnt));
    //                 // 计数
    //                 cnt++;
    //             }
    //         }
    //     }   
    //     // 计算特征误差 交互矩阵
    //     this->error_s_ = feature_new.rowRange(0, cnt) - feature_old.rowRange(0, cnt);
    //     this->L_e_ = 0.5*(Le_new.rowRange(0, cnt) + Le_old.rowRange(0, cnt));
    // }
}

void Polarimetric_Visual_Servoing::save_data_error_feature()
{
    // void Direct_Visual_Servoing::save_data_error_feature()
    // {
    //     Mat error_ave = (this->error_s_.t() * this->error_s_) / (this->error_s_.rows * this->error_s_.cols);
    //     this->data_vs.error_feature_.push_back(error_ave);
    // }
}