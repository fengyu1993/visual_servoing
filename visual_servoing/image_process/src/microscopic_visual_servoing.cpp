#include "microscopic_visual_servoing.h"
#include <opencv2/imgproc.hpp>
#include <math.h>
#include <iostream>
#include <fstream>
#include <string>
#include <ctime> 
#include <chrono>

Microscopic_Visual_Servoing::Microscopic_Visual_Servoing(int resolution_x=640, int resolution_y=480)
{
    this->resolution_x_ = resolution_x;
    this->resolution_y_ = resolution_y;
    this->image_gray_desired_ = Mat::zeros(this->resolution_y_, this->resolution_x_, CV_64FC1);
    this->image_gray_current_ = Mat::zeros(this->resolution_y_, this->resolution_x_, CV_64FC1);
    this->camera_velocity_ = Mat::zeros(6, 1, CV_64FC1);
	this->flag_first_ = true;
	this->iteration_num_ = 0;
}

// ��ʼ��
void Microscopic_Visual_Servoing::init_VS(double lambda, double epsilon, Mat& image_gray_desired, Mat& image_gray_initial, camera_intrinsic camera_parameters, Mat pose_desired)
{
    this->lambda_ = lambda;
    this->epsilon_ = epsilon;
    set_camera_intrinsic(camera_parameters);
    set_image_gray_desired(image_gray_desired);
    set_image_gray_initial(image_gray_initial);
    // set_pose_desired(pose_desired);
    // save_data_image();
    // save_pose_desired();
}

// // ��������ٶ�
// Mat Microscopic_Visual_Servoing::get_camera_velocity()
// {
// 	// ��������ٶ�
//     Mat L_e_inv;
// 	this->iteration_num_++;
//     get_feature_error_interaction_matrix(); 
//     invert(this->L_e_, L_e_inv, DECOMP_SVD);
//     this->camera_velocity_ = -this->lambda_ * L_e_inv * this->error_s_;
//     return this->camera_velocity_;
// }

// // �ж��Ƿ��ŷ��ɹ�
// bool Microscopic_Visual_Servoing::is_success()
// {
// 	Mat error_ave = this->error_s_.t() * this->error_s_ / (this->error_s_.rows*this->error_s_.cols);

// 	if(error_ave.at<double>(0,0) < this->epsilon_)
// 	{
// 		cout << "Visual Servoing Success" << endl;
// 		return true;
// 	}
// 	else
// 	{
// 		return false;
// 	}
// }

// ��������ڲ�
void Microscopic_Visual_Servoing::set_camera_intrinsic(camera_intrinsic camera_parameters)
{
    this->camera_intrinsic_ = camera_parameters;
}

// ���������Ҷ�ͼ��
void Microscopic_Visual_Servoing::set_image_gray_desired(Mat& image_gray_desired)
{
    image_gray_desired.copyTo(this->image_gray_desired_);
}

// ���õ�ǰ�Ҷ�ͼ��
void Microscopic_Visual_Servoing::set_image_gray_current(Mat& image_gray_current)
{
    image_gray_current.copyTo(this->image_gray_current_);
	// this->image_gray_current_ = image_gray_current;
}

// ���ó�ʼͼ��
void Microscopic_Visual_Servoing::set_image_gray_initial(Mat& image_gray_initial)
{
    image_gray_initial.copyTo(this->image_gray_initial_);
}

// // ��������λ��
// void Microscopic_Visual_Servoing::set_pose_desired(Mat& pose_desired)
// {
// 	pose_desired.copyTo(this->pose_desired_);
// }


// // ����ͼ������
// void Microscopic_Visual_Servoing::save_pose_desired()
// {
//     this->pose_desired_.copyTo(this->data_vs_.pose_desired_);   
// }

// // ����ͼ������
// void Microscopic_Visual_Servoing::save_data_image()
// {
//     save_data_image_gray_desired();
//     save_data_image_gray_initial();
// }

// // ��������ͼ��
// void Microscopic_Visual_Servoing::save_data_image_gray_desired()
// {
//     this->image_gray_desired_.copyTo(this->data_vs_.image_gray_desired_);
// }

// // �����ʼͼ��
// void Microscopic_Visual_Servoing::save_data_image_gray_initial()
// {
//     this->image_gray_initial_.copyTo(this->data_vs_.image_gray_init_);
// }

// // ��������ٶ�
// void Microscopic_Visual_Servoing::save_data_camera_velocity()
// {
//     this->data_vs_.velocity_.push_back(this->camera_velocity_.t());
// }

// // �����������
// void Microscopic_Visual_Servoing::save_data_error_feature()
// {
//     this->data_vs_.error_feature_.push_back(this->error_s_);
// }

// // �������λ��
// void Microscopic_Visual_Servoing::save_data_camera_pose(Mat& pose)
// {
//     this->data_vs_.pose_.push_back(pose);
// }

// // ������������
// void Microscopic_Visual_Servoing::save_data(Mat pose)
// {
//     save_data_camera_velocity();
//     save_data_camera_pose(pose);
//     save_data_error_feature(); 
//     save_data_other_parameter();
// 	save_data_vs__time();
// }

// // ��¼ʱ��
// void Microscopic_Visual_Servoing::save_data_vs__time()
// {
	
// 	if (this->iteration_num_ == 1)
// 	{
// 		this->start_VS_time_ = clock();
// 		this->data_vs_.time_vs_.push_back(0.0);
// 	}
// 	else
// 	{
// 		this->data_vs_.time_vs_.push_back((double)(clock() - this->start_VS_time_) / CLOCKS_PER_SEC);
// 	}
// }


// // �����ݱ������ļ���
// void Microscopic_Visual_Servoing::write_data()
// {
//     string file_name = get_save_file_name();
//     string location = "/home/cyh/Work/visual_servoing_ws/src/visual_servoing/image_process/resource/data/";
//     // ����ͼ��
//     string saveImage_desired = location + file_name + "_desired_image.png";
//     string saveImage_initial = location + file_name + "_initial_image.png";
//     imwrite(saveImage_desired, this->data_vs_.image_gray_desired_*255);
//     imwrite(saveImage_initial, this->data_vs_.image_gray_init_*255);
//     // ��������
//     ofstream oFile;
// 	string excel_name = location + file_name + "_data.xls";
//     oFile.open(excel_name, ios::out|ios::trunc);
//     write_visual_servoing_data(oFile);
//     write_other_data(oFile);
//     // �ر��ļ�
//     oFile.close();
// }

// // д������Ӿ��ŷ����ݵ��ļ�
// void Microscopic_Visual_Servoing::write_visual_servoing_data(ofstream& oFile)
// {
//     oFile << "camera velocity" << endl;
//     write_to_excel(this->data_vs_.velocity_, oFile);
//     oFile << "camera pose" << endl;
//     write_to_excel(this->data_vs_.pose_, oFile);
//     oFile << "camera desired pose" << endl;
//     write_to_excel(this->data_vs_.pose_desired_, oFile);   
//     oFile << "error feature" << endl;
//     write_to_excel(this->data_vs_.error_feature_, oFile);
//     oFile << "visual servoing time" << endl;
//     write_to_excel(this->data_vs_.time_vs_, oFile);	
// }

// // �洢�����ļ�����
// string Microscopic_Visual_Servoing::get_save_file_name()
// {
//     return get_date_time() + "_" + get_method_name();
// }

// // �Ӿ��ŷ���������
// string Microscopic_Visual_Servoing::get_method_name()
// {
//     return "Visual_Servoing";
// }

// // ��ȡ��ǰ�����ʱ��
// string Microscopic_Visual_Servoing::get_date_time()
// {
// 	auto to_string = [](const std::chrono::system_clock::time_point& t)->std::string
// 	{
// 		auto as_time_t = std::chrono::system_clock::to_time_t(t);
// 		struct tm tm;
// #if defined(WIN32) || defined(_WINDLL)
// 		localtime_s(&tm, &as_time_t);  //win api���̰߳�ȫ����std::localtime�̲߳���ȫ
// #else
// 		localtime_r(&as_time_t, &tm);//linux api���̰߳�ȫ
// #endif

// 		std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(t.time_since_epoch());
// 		char buf[128];
// 		snprintf(buf, sizeof(buf), "%04d_%02d_%02d_%02d_%02d_%02d",
// 			tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
// 		return buf;
// 	};

// 	std::chrono::system_clock::time_point t = std::chrono::system_clock::now();
// 	return to_string(t);
// }

// void Microscopic_Visual_Servoing::write_to_excel(Mat data, ofstream& oFile)
// {
// 		int channels = data.channels();            //��ȡͼ��channel  
// 		int nrows = data.rows;                     //���������  
// 		int ncols = data.cols*channels;            //�����������=����*channel������  
 
// 		//ѭ���ñ���
// 		int i = 0;
// 		int j = 0;
 
// 		if (data.depth() == CV_8U)//uchar
// 		{
// 			for (i = 0; i<nrows; i++)
// 			{
// 				for (j = 0; j<ncols; j++)
// 				{
// 					int tmpVal = (int)data.ptr<uchar>(i)[j];
// 					oFile << tmpVal << '\t';
// 				}
// 				oFile << endl;
// 			}
// 		}
// 		else if (data.depth() == CV_16S)//short
// 		{
// 			for (i = 0; i<nrows; i++)
// 			{
// 				for (j = 0; j<ncols; j++)
// 				{
// 					oFile << (short)data.ptr<short>(i)[j] << '\t';
// 				}
// 				oFile << endl;
// 			}
// 		}
// 		else if (data.depth() == CV_16U)//unsigned short
// 		{
// 			for (i = 0; i<nrows; i++)
// 			{
// 				for (j = 0; j<ncols; j++)
// 				{
// 					oFile << (unsigned short)data.ptr<unsigned short>(i)[j] << '\t';
// 				}
// 				oFile << endl;
// 			}
// 		}
// 		else if (data.depth() == CV_32S)//int 
// 		{
// 			for (i = 0; i<nrows; i++)
// 			{
// 				for (j = 0; j<ncols; j++)
// 				{
// 					oFile << (int)data.ptr<int>(i)[j] << '\t';
// 				}
// 				oFile << endl;
// 			}
// 		}
// 		else if (data.depth() == CV_32F)//float
// 		{
// 			for (i = 0; i<nrows; i++)
// 			{
// 				for (j = 0; j<ncols; j++)
// 				{
// 					oFile << (float)data.ptr<float>(i)[j] << '\t';
// 				}
// 				oFile << endl;
// 			}
// 		}
// 		else//CV_64F double
// 		{
// 			for (i = 0; i < nrows; i++)
// 			{
// 				for (j = 0; j < ncols; j++)
// 				{
// 					oFile << (double)data.ptr<double>(i)[j] << '\t';
// 				}
// 				oFile << endl;
// 			}
// 		}
// }

