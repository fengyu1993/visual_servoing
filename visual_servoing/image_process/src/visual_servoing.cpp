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

// ��ʼ��
void Visual_Servoing::init_VS(double lambda, double epsilon, Mat image_gray_desired, Mat image_depth_desired, Mat image_gray_initial, Mat camera_intrinsic)
{
    this->lambda_ = lambda;
    this->epsilon_ = epsilon;
    set_camera_intrinsic(camera_intrinsic);
    set_image_depth_desired(image_depth_desired);
    set_image_gray_desired(image_gray_desired);
    set_image_gray_initial(image_gray_initial);
    save_data_image();
}

// ��������ٶ�
Mat Visual_Servoing::get_camera_velocity()
{
    Mat L_e_inv;
    get_feature_error_interaction_matrix();
    invert(this->L_e_, L_e_inv, DECOMP_SVD);
    this->camera_velocity_ = -this->lambda_ * L_e_inv * this->error_s_;
    return this->camera_velocity_;
}

// �ж��Ƿ��ŷ��ɹ�
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

// ��������ڲ�
void Visual_Servoing::set_camera_intrinsic(Mat camera_intrinsic)
{
    camera_intrinsic.copyTo(this->camera_intrinsic_);
}

// ���������Ҷ�ͼ��
void Visual_Servoing::set_image_gray_desired(Mat image_gray_desired)
{
    image_gray_desired.copyTo(this->image_gray_desired_);
}

// ���õ�ǰ�Ҷ�ͼ��
void Visual_Servoing::set_image_gray_current(Mat image_gray_current)
{
    // image_gray_current.copyTo(this->image_gray_current_);
    this->image_gray_current_ = image_gray_current;
}

// ���ó�ʼͼ��
void Visual_Servoing::set_image_gray_initial(Mat image_gray_initial)
{
    image_gray_initial.copyTo(this->image_gray_initial_);
}

// �����������ͼ��
void Visual_Servoing::set_image_depth_desired(Mat image_depth_desired)
{
    image_depth_desired.copyTo(this->image_depth_desired_);
}

// ���õ�ǰ���ͼ��
void Visual_Servoing::set_image_depth_current(Mat image_depth_current)
{
    // image_depth_current.copyTo(this->image_depth_current_);
    this->image_depth_current_ = image_depth_current;
}

// ����ͼ������
void Visual_Servoing::save_data_image()
{
    save_data_image_gray_desired();
    save_data_image_gray_initial();
}

// ��������ͼ��
void Visual_Servoing::save_data_image_gray_desired()
{
    this->image_gray_desired_.copyTo(this->data_vs.image_gray_desired_);
}

// �����ʼͼ��
void Visual_Servoing::save_data_image_gray_initial()
{
    this->image_gray_initial_.copyTo(this->data_vs.image_gray_init_);
}

// ��������ٶ�
void Visual_Servoing::save_data_camera_velocity()
{
    this->data_vs.velocity_.push_back(this->camera_velocity_.t());
}

// �����������
void Visual_Servoing::save_data_error_feature()
{
    this->data_vs.error_feature_.push_back(this->error_s_);
}

// �������λ��
void Visual_Servoing::save_data_camera_pose(Mat pose)
{
    this->data_vs.pose_.push_back(pose.t());
}

// ������������
void Visual_Servoing::save_data(Mat pose)
{
    save_data_camera_velocity();
    save_data_camera_pose(pose);
    save_data_error_feature(); 
    save_data_other_parameter();
}

// �����ݱ������ļ���
void Visual_Servoing::write_data()
{
    string file_name = get_save_file_name();
    string location = "/home/cyh/Work/visual_servoing_ws/src/visual_servoing/image_process/resource/data/";
    // ����ͼ��
    string saveImage_desired = location + file_name + "_desired_image.png";
    string saveImage_initial = location + file_name + "_initial_image.png";
    imwrite(saveImage_desired, this->data_vs.image_gray_desired_*255);
    imwrite(saveImage_initial, this->data_vs.image_gray_init_*255);
    // ��������
    ofstream oFile;
	string excel_name = location + file_name + "_data.xls";
    oFile.open(excel_name, ios::out|ios::trunc);
    write_visual_servoing_data(oFile);
    write_other_data(oFile);
    // �ر��ļ�
    oFile.close();
}

// д������Ӿ��ŷ����ݵ��ļ�
void Visual_Servoing::write_visual_servoing_data(ofstream& oFile)
{
    oFile << "camera velocity" << endl;
    write_to_excel(this->data_vs.velocity_, oFile);
    oFile << "camera pose" << endl;
    write_to_excel(this->data_vs.pose_, oFile);
    oFile << "error feature" << endl;
    write_to_excel(this->data_vs.error_feature_, oFile);
}

// �洢�����ļ�����
string Visual_Servoing::get_save_file_name()
{
    return get_date_time() + "_" + get_method_name();
}

// �Ӿ��ŷ���������
string Visual_Servoing::get_method_name()
{
    return "Visual_Servoing";
}

// ��ȡ��ǰ�����ʱ��
string Visual_Servoing::get_date_time()
{
	auto to_string = [](const std::chrono::system_clock::time_point& t)->std::string
	{
		auto as_time_t = std::chrono::system_clock::to_time_t(t);
		struct tm tm;
#if defined(WIN32) || defined(_WINDLL)
		localtime_s(&tm, &as_time_t);  //win api���̰߳�ȫ����std::localtime�̲߳���ȫ
#else
		localtime_r(&as_time_t, &tm);//linux api���̰߳�ȫ
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
		int channels = data.channels();            //��ȡͼ��channel  
		int nrows = data.rows;                     //���������  
		int ncols = data.cols*channels;            //�����������=����*channel������  
 
		//ѭ���ñ���
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

//��ת����õ���Ԫ��
Mat Visual_Servoing::Matrix2Quaternion(Mat matrix)
{
  double tr, qx, qy, qz, qw;

  // �������켣
  double a[4][4] = {0};
  for(int i=0;i<4;i++)
    for(int j=0;j<4;j++)
      a[i][j]=matrix.at<double>(i,j);
  
  // I removed + 1.0f; see discussion with Ethan
  double trace = a[0][0] + a[1][1] + a[2][2]; 
  if( trace > 0 ) {
    // I changed M_EPSILON to 0
    double s = 0.5 / sqrt(trace+ 1.0);
    qw = 0.25 / s;
    qx = ( a[2][1] - a[1][2] ) * s;
    qy = ( a[0][2] - a[2][0] ) * s;
    qz = ( a[1][0] - a[0][1] ) * s;
  } else {
    if ( a[0][0] > a[1][1] && a[0][0] > a[2][2] ) {
      double s = 2.0 * sqrt( 1.0 + a[0][0] - a[1][1] - a[2][2]);
      qw = (a[2][1] - a[1][2] ) / s;
      qx = 0.25 * s;
      qy = (a[0][1] + a[1][0] ) / s;
      qz = (a[0][2] + a[2][0] ) / s;
    } else if (a[1][1] > a[2][2]) {
      double s = 2.0 * sqrt( 1.0 + a[1][1] - a[0][0] - a[2][2]);
      qw = (a[0][2] - a[2][0] ) / s;
      qx = (a[0][1] + a[1][0] ) / s;
      qy = 0.25 * s;
      qz = (a[1][2] + a[2][1] ) / s;
    } else {
      double s = 2.0 * sqrt( 1.0 + a[2][2] - a[0][0] - a[1][1] );
      qw = (a[1][0] - a[0][1] ) / s;
      qx = (a[0][2] + a[2][0] ) / s;
      qy = (a[1][2] + a[2][1] ) / s;
      qz = 0.25 * s;
    }    
  }

  double q[] = {qw,qx,qy,qz};

  return cv::Mat(4,1,CV_64FC1,q).clone();
}
