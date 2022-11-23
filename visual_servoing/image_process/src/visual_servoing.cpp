#include "visual_servoing.h"
#include <opencv2/imgproc.hpp>
#include <math.h>
#include <iostream>
#include <fstream>
#include <string>
#include <ctime> 
#include <chrono>

Visual_Servoing::Visual_Servoing(int resolution_x=640, int resolution_y=480)
{
    this->resolution_x_ = resolution_x;
    this->resolution_y_ = resolution_y;
    this->image_gray_desired_ = Mat::zeros(this->resolution_y_, this->resolution_x_, CV_64FC1);
    this->image_gray_current_ = Mat::zeros(this->resolution_y_, this->resolution_x_, CV_64FC1);
    this->image_depth_desired_ = Mat::zeros(this->resolution_y_, this->resolution_x_, CV_64FC1);
    this->image_depth_current_ = Mat::zeros(this->resolution_y_, this->resolution_x_, CV_64FC1);
    this->camera_intrinsic_ = Mat::zeros(3, 3, CV_64FC1);
    this->camera_velocity_ = Mat::zeros(6, 1, CV_64FC1);
}

// 初始化
void Visual_Servoing::init_VS(double lambda, double epsilon, Mat& image_gray_desired, Mat& image_depth_desired, Mat image_gray_initial, Mat camera_intrinsic, Mat pose_desired)
{
    this->lambda_ = lambda;
    this->epsilon_ = epsilon;
    set_camera_intrinsic(camera_intrinsic);
    set_image_depth_desired(image_depth_desired);
    set_image_gray_desired(image_gray_desired);
    set_image_gray_initial(image_gray_initial);
    set_pose_desired(pose_desired);
    save_data_image();
    save_pose_desired();
}

// 计算相机速度
Mat Visual_Servoing::get_camera_velocity()
{
    Mat L_e_inv;
    get_feature_error_interaction_matrix();
    invert(this->L_e_, L_e_inv, DECOMP_SVD);
    this->camera_velocity_ = -this->lambda_ * L_e_inv * this->error_s_;
    return this->camera_velocity_;
}

// 判断是否伺服成功
bool Visual_Servoing::is_success()
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
void Visual_Servoing::set_camera_intrinsic(Mat& camera_intrinsic)
{
    camera_intrinsic.copyTo(this->camera_intrinsic_);
}

// 设置期望灰度图像
void Visual_Servoing::set_image_gray_desired(Mat& image_gray_desired)
{
    image_gray_desired.copyTo(this->image_gray_desired_);
}

// 设置当前灰度图像
void Visual_Servoing::set_image_gray_current(Mat& image_gray_current)
{
    image_gray_current.copyTo(this->image_gray_current_);
	// this->image_gray_current_ = image_gray_current;
}

// 设置初始图像
void Visual_Servoing::set_image_gray_initial(Mat& image_gray_initial)
{
    image_gray_initial.copyTo(this->image_gray_initial_);
}

// 设置期望深度图像
void Visual_Servoing::set_image_depth_desired(Mat& image_depth_desired)
{
    image_depth_desired.copyTo(this->image_depth_desired_);
}

// 设置当前深度图像
void Visual_Servoing::set_image_depth_current(Mat& image_depth_current)
{
    image_depth_current.copyTo(this->image_depth_current_);
    // this->image_depth_current_ = image_depth_current;
}

// 保存期望位姿
void Visual_Servoing::set_pose_desired(Mat& pose_desired)
{
	pose_desired.copyTo(this->pose_desired_);
}


// 保存图像数据
void Visual_Servoing::save_pose_desired()
{
    this->pose_desired_.copyTo(this->data_vs.pose_desired_);   
}

// 保存图像数据
void Visual_Servoing::save_data_image()
{
    save_data_image_gray_desired();
    save_data_image_gray_initial();
}

// 保存期望图像
void Visual_Servoing::save_data_image_gray_desired()
{
    this->image_gray_desired_.copyTo(this->data_vs.image_gray_desired_);
}

// 保存初始图像
void Visual_Servoing::save_data_image_gray_initial()
{
    this->image_gray_initial_.copyTo(this->data_vs.image_gray_init_);
}

// 保存相机速度
void Visual_Servoing::save_data_camera_velocity()
{
    this->data_vs.velocity_.push_back(this->camera_velocity_.t());
}

// 保存特征误差
void Visual_Servoing::save_data_error_feature()
{
    this->data_vs.error_feature_.push_back(this->error_s_);
}

// 保存相机位姿
void Visual_Servoing::save_data_camera_pose(Mat& pose)
{
    this->data_vs.pose_.push_back(pose.t());
}

// 保存所有数据
void Visual_Servoing::save_data(Mat pose)
{
    save_data_camera_velocity();
    save_data_camera_pose(pose);
    save_data_error_feature(); 
    save_data_other_parameter();
}

// 将数据保存在文件中
void Visual_Servoing::write_data()
{
    string file_name = get_save_file_name();
    string location = "/home/cyh/Work/visual_servoing_ws/src/visual_servoing/image_process/resource/data/";
    // 保存图像
    string saveImage_desired = location + file_name + "_desired_image.png";
    string saveImage_initial = location + file_name + "_initial_image.png";
    imwrite(saveImage_desired, this->data_vs.image_gray_desired_*255);
    imwrite(saveImage_initial, this->data_vs.image_gray_init_*255);
    // 保存数据
    ofstream oFile;
	string excel_name = location + file_name + "_data.xls";
    oFile.open(excel_name, ios::out|ios::trunc);
    write_visual_servoing_data(oFile);
    write_other_data(oFile);
    // 关闭文件
    oFile.close();
}

// 写入基本视觉伺服数据到文件
void Visual_Servoing::write_visual_servoing_data(ofstream& oFile)
{
    oFile << "camera velocity" << endl;
    write_to_excel(this->data_vs.velocity_, oFile);
    oFile << "camera pose" << endl;
    write_to_excel(this->data_vs.pose_, oFile);
    oFile << "camera desired pose" << endl;
    write_to_excel(this->data_vs.pose_desired_.t(), oFile);   
    oFile << "error feature" << endl;
    write_to_excel(this->data_vs.error_feature_, oFile);
}

// 存储数据文件命名
string Visual_Servoing::get_save_file_name()
{
    return get_date_time() + "_" + get_method_name();
}

// 视觉伺服方法名字
string Visual_Servoing::get_method_name()
{
    return "Visual_Servoing";
}

// 获取当前计算机时间
string Visual_Servoing::get_date_time()
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

void Visual_Servoing::write_to_excel(Mat data, ofstream& oFile)
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

