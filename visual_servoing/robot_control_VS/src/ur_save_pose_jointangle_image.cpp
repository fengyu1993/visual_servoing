#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Twist.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <cv_bridge/cv_bridge.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <fstream>
#include <string>
#include <ctime> 
#include <chrono>
#include <tf/transform_listener.h>
#include <librealsense2/rs.hpp>
#include <librealsense2/h/rs_option.h>
#include "ArenaApi.h"
#include "SaveApi.h"

using namespace std;
using namespace cv;
using namespace sensor_msgs;
using namespace message_filters;

#define IMAGE_TIMEOUT 2000
#define SYSTEM_TIMEOUT 100
#define CHANNEL1 0
#define CHANNEL2 1
#define CHANNEL3 2
#define CHANNEL4 3
#define IMGLEN 1253376

Mat joint_group_positions;
Mat T_base_camera;
Mat img_polar;
Mat img_depth;
double depth_scale;

rs2::frame depth_filter(rs2::frame& depth_img);
Mat hole_fill(Mat& img_depth);
float get_depth_scale(rs2::device dev);
GenICam::gcstring Device_Init(Arena::IDevice* pDevice);
Arena::DeviceInfo SelectDevice(std::vector<Arena::DeviceInfo>& deviceInfos);
void GetPolarizedData(Arena::IDevice* pDevice, uint8_t* pDst);
void FetchData(const uint8_t* pSrc_Angles, size_t srcBytesPerPixel_Angles, uint8_t* pDst);
cv::Mat Quaternion2Matrix (cv::Mat q);
Mat get_T(tf::StampedTransform  pose);
void write_to_excel(Mat data, ofstream& oFile);
string get_date_time();
void save_joint_positions(moveit::planning_interface::MoveGroupInterface& move_group_interface);
void save_camera_pose();
void write_image();
void write_data();

int main(int argc, char** argv)
{
    ros::init(argc, argv, "save_pose_image");
    ros::NodeHandle nh;
    int resolution_x, resolution_y;
    nh.getParam("resolution_x", resolution_x);
    nh.getParam("resolution_y", resolution_y);

    ros::Rate loop_rate(50);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "ur";
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
    
    // Realsense L515
    rs2::pipeline pipe;
    rs2::config cfg;
    rs2::colorizer color_map;
    cfg.enable_stream(RS2_STREAM_DEPTH, resolution_x, resolution_y, RS2_FORMAT_Z16,30);
    rs2::pipeline_profile profile = pipe.start(cfg);
    rs2::align align_to_color(RS2_STREAM_COLOR);

    // arena
	Arena::ISystem* pSystem = Arena::OpenSystem();
	pSystem->UpdateDevices(SYSTEM_TIMEOUT);
	std::vector<Arena::DeviceInfo> deviceInfos = pSystem->GetDevices();
	if (deviceInfos.size() == 0)
	{
		std::cout << "\nNo camera connected\nPress enter to complete\n";
		std::getchar();
		return 0;
	}
	Arena::DeviceInfo selectedDeviceInfo = SelectDevice(deviceInfos);
	Arena::IDevice* pDevice = pSystem->CreateDevice(selectedDeviceInfo);

	GenICam::gcstring pixelFormatInitial = Device_Init(pDevice);
	size_t dstWidth = 1224;
	size_t dstHeight = 1024;
	size_t dstBitsPerPixel = 3 * 8;
	size_t dstBytesPerPixel = dstBitsPerPixel / 8;
	size_t dstStride = dstWidth * dstBitsPerPixel / 8;
	size_t dstDataSize = dstWidth * dstHeight * dstBitsPerPixel / 8;
	uint8_t* pDst = new uint8_t[dstBytesPerPixel * dstWidth * dstHeight];
	pDevice->StartStream();

    cout<< "Press space to save rgb_raw and depth_raw to a file."<<endl;

    while (ros::ok())
    {
        // read L515 image
		rs2::frameset frameset = pipe.wait_for_frames();
		rs2::frameset frameset_depth = align_to_color.process(frameset);
		auto depth = frameset_depth.get_depth_frame();
		depth_scale = get_depth_scale(profile.get_device());
		depth = depth_filter(depth);
		const int w_depth = depth.as<rs2::video_frame>().get_width();
		const int h_depth = depth.as<rs2::video_frame>().get_height();
		Mat depth_show(cv::Size(w_depth, h_depth), CV_8UC3, (void*)depth.apply_filter(color_map).get_data(), cv::Mat::AUTO_STEP);
		Mat img_depth(cv::Size(w_depth, h_depth), CV_16UC1, (void*)depth.get_data(), cv::Mat::AUTO_STEP);
		img_depth = hole_fill(img_depth);
		img_depth.convertTo(img_depth, CV_64FC1);
		img_depth = img_depth * (1000*depth_scale);
		img_depth.convertTo(img_depth, CV_16UC1);
		// read Arena
		GetPolarizedData(pDevice, pDst);
		cv::Mat srcImage(dstHeight, dstWidth, CV_8UC3, pDst);
		if (srcImage.empty()) {
			std::cerr << "Could not open or find the image!" << std::endl;
			continue;
		}
		cv::Mat img_polar;
		cv::resize(srcImage, img_polar, cv::Size(resolution_x, resolution_y));
        // show
        imshow("Polar", img_polar);
        imshow("Depth", depth_show);
        // test
        cout << "Depth = \n" <<  img_depth.rowRange(1,10).colRange(0,5) << endl;
        // save
        if((char)waitKey(10) == 32) // Spacebar
        {
            save_joint_positions(move_group_interface);
            save_camera_pose();
            std::cout << "Save data successfully" << endl;
            std::cout << "Press space to start..." << std::endl;
            if((char)waitKey() == 32)
            {
                write_data();
                write_image();
                std::cout << "Save image successfully" << endl;
            }
        }
        loop_rate.sleep();
    }

 	// close L515
	pipe.stop();
	// close arena
	pDst = NULL;
	delete[] pDst;
	pDevice->StopStream();
	Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "PixelFormat", pixelFormatInitial);
	pSystem->DestroyDevice(pDevice);
	Arena::CloseSystem(pSystem);
 
    return 0;
}

// Mat save_camera_intrinsic(rs2::stream_profile cprofile)
// {
//     ///获取彩色相机内参
//     rs2::video_stream_profile cvsprofile(cprofile);
//     rs2_intrinsics color_intrin =  cvsprofile.get_intrinsics();
//     camera_intrinsic = (Mat_<double>(3,3) << color_intrin.fx, 0, color_intrin.ppx,
//                                         0, color_intrin.fy, color_intrin.ppy,0, 0, 1);
//     cout << "camera_intrinsic = " << camera_intrinsic << endl;
//     return  camera_intrinsic; 
// }



void write_data()
{
    ofstream oFile;
    string location = "/home/cyh/Work/visual_servoing_ws/src/visual_servoing/robot_control_VS/param/";
    string time = get_date_time();
    string excel_name = location + time + "_pose_data.xls";
    oFile.open(excel_name, ios::out|ios::trunc);
    oFile << "joint angle" << endl;
    write_to_excel(joint_group_positions, oFile);
    oFile << "camera pose" << endl;
    write_to_excel(T_base_camera, oFile);
    oFile.close();
}

void write_image()
{
    // 保存深度图
    imwrite("/home/cyh/Work/visual_servoing_ws/src/visual_servoing/robot_control_VS/param/image_depth_init.png", img_depth);
    // 保存彩色图
    imwrite("/home/cyh/Work/visual_servoing_ws/src/visual_servoing/robot_control_VS/param/image_rgb_init.png", img_polar);
}

// 保存相机位姿
void save_camera_pose()
{
    tf::TransformListener listener;
    tf::StampedTransform transform;
    listener.waitForTransform("base_link", "camera_polar_frame", ros::Time(0), ros::Duration(3.0));
    listener.lookupTransform("base_link", "camera_polar_frame", ros::Time(0), transform);
    T_base_camera = get_T(transform);
    cout << "T_base_camera = \n" << T_base_camera << endl;
}

void save_joint_positions(moveit::planning_interface::MoveGroupInterface& move_group_interface)
{
    std::vector<double> joint_positions = move_group_interface.getCurrentJointValues();
    joint_group_positions = (Mat_<double>(1,6) << joint_positions[0], joint_positions[1], 
                            joint_positions[2], joint_positions[3], joint_positions[4], 
                            joint_positions[5]);
    cout << "joint_angle = \n" <<  joint_group_positions << endl;
}


cv::Mat Quaternion2Matrix (cv::Mat q)
{
  double w = q.at<double>(0);
  double x = q.at<double>(1);
  double y = q.at<double>(2);
  double z = q.at<double>(3);

  double xx = x*x;
  double yy = y*y;
  double zz = z*z;
  double xy = x*y;
  double wz = w*z;
  double wy = w*y;
  double xz = x*z;
  double yz = y*z;
  double wx = w*x;

  double ret[3][3];
  ret[0][0] = 1.0-2*(yy+zz);
  ret[0][1] = 2*(xy-wz);
  ret[0][2] = 2*(wy+xz);
 
  ret[1][0] = 2*(xy+wz);
  ret[1][1] = 1.0-2*(xx+zz);
  ret[1][2] = 2*(yz-wx);
 
  ret[2][0] = 2*(xz-wy);
  ret[2][1] = 2*(yz+wx);
  ret[2][2] = 1.0-2*(xx+yy);
 
  return cv::Mat(3,3,CV_64FC1,ret).clone();
}


Mat get_T(tf::StampedTransform transform)
{
    double x = transform.getOrigin().getX();
    double y = transform.getOrigin().getY();
    double z = transform.getOrigin().getZ();
    double W = transform.getRotation().getW();
    double X = transform.getRotation().getX();
    double Y = transform.getRotation().getY();
    double Z = transform.getRotation().getZ();

    Mat T = Mat::eye(4,4,CV_64FC1);
    Mat p = (Mat_<double>(3,1) << x, y, z);
    p.copyTo(T.rowRange(0,3).colRange(3,4));
    Mat q = (Mat_<double>(4,1) << W, X, Y, Z);
    Mat R = Quaternion2Matrix(q);
    R.copyTo(T.rowRange(0,3).colRange(0,3));
    return T;
}


void write_to_excel(Mat data, ofstream& oFile)
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
					oFile << (float)data.ptr<float>(i)[j] << "\t";
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
					oFile << (double)data.ptr<double>(i)[j] << "\t";
				}
				oFile << endl;
			}
		}
}


// 获取当前计算机时间
string get_date_time()
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

void FetchData(const uint8_t* pSrc_Angles, size_t srcBytesPerPixel_Angles, uint8_t* pDst)
{
	uint16_t I_0 = 0;
	uint8_t I_45 = 0;
	uint8_t I_90 = 0;
	uint8_t I_135 = 0;
	uint8_t O = 0;

	double A = 0;
	double phi0 = 0;

	uint8_t Dolp = 0;
	uint8_t Aolp = 0;

	// size_t test_num = 1253375;

	for (size_t i = 0; i < IMGLEN; i += 1)
	{
		I_0 = pSrc_Angles[i * srcBytesPerPixel_Angles + CHANNEL1];
		I_45 = pSrc_Angles[i * srcBytesPerPixel_Angles + CHANNEL2];
		I_90 = pSrc_Angles[i * srcBytesPerPixel_Angles + CHANNEL3];
		I_135 = pSrc_Angles[i * srcBytesPerPixel_Angles + CHANNEL4];

		O = (I_0 + I_90) / 2.0;
		A = sqrt(((I_0 - I_90) * (I_0 - I_90) + (I_45 - I_135) * (I_45 - I_135)) / 4.0);
		phi0 = 0.5 * atan2(I_45 - I_135, I_0 - I_90); // 使用atan2来处理象限问题

		Dolp = (A * 255) / O;
		Aolp = (phi0 + 3.14159 / 2) / 3.14159 * 255.0;//不确定

		pDst[i * 3 + CHANNEL1] = O;
		pDst[i * 3 + CHANNEL2] = Dolp;
		pDst[i * 3 + CHANNEL3] = Aolp;
	}

	// printf("\tpSrc_Angles[0]:%d\tpSrc_Angles[2]:%d\t\n", pSrc_Angles[test_num * 4 + 0], pSrc_Angles[test_num * 4 + 2]);
}

void GetPolarizedData(Arena::IDevice* pDevice, uint8_t* pDst)
{
	
	Arena::IImage* pImage_Angles = pDevice->GetImage(IMAGE_TIMEOUT);
	// SaveImage(pImage, FILE_NAME_SRC);

	// src info
	uint64_t srcPF_Angles = pImage_Angles->GetPixelFormat();
	// size_t srcWidth_Angles = pImage_Angles->GetWidth();
	size_t srcHeight_Angles = pImage_Angles->GetHeight();
	size_t srcBitsPerPixel_Angles = Arena::GetBitsPerPixel(srcPF_Angles);
	size_t srcBytesPerPixel_Angles = srcBitsPerPixel_Angles / 8;
	// size_t srcStride_Angles = srcWidth_Angles * srcBitsPerPixel_Angles / 8;
	// size_t srcDataSize_Angles = srcWidth_Angles * srcHeight_Angles * srcBitsPerPixel_Angles / 8;

	const uint8_t* pSrc_Angles = pImage_Angles->GetData();
	
	// std::cout << "\t" << "srcWidth:" << srcWidth_Angles << "\t\tsrcHeight:" << srcHeight_Angles << "\t\tsrcBytesPerPixel:" 
	// << srcBytesPerPixel_Angles << "\t\tsrcStride:" << srcStride_Angles << "\t\tsrcDataSize:" << srcDataSize_Angles << "\n";

	FetchData(pSrc_Angles, srcBytesPerPixel_Angles, pDst);

	// clean up
	pDevice->RequeueBuffer(pImage_Angles);


	// std::cout << "\t" << "Polarized Data is Ready\n";
	
}

Arena::DeviceInfo SelectDevice(std::vector<Arena::DeviceInfo>& deviceInfos)
{
    if (deviceInfos.size() == 1)
    {
        std::cout  << "\n" "\tOnly one device detected: "  << deviceInfos[0].ModelName() << "\t" << deviceInfos[0].SerialNumber() << "\t" << deviceInfos[0].IpAddressStr() << ".\n";
        std::cout  << "\tAutomatically selecting this device.\n";
        return deviceInfos[0];
    }
    
    std::cout << "\nSelect device:\n";
    for (size_t i = 0; i < deviceInfos.size(); i++)
    {
        std::cout << "\t" << i + 1 << ". " << deviceInfos[i].ModelName() << "\t" << deviceInfos[i].SerialNumber() << "\t" << deviceInfos[i].IpAddressStr() << "\n";
    }
    size_t selection = 0;
	
	do
	{
		std::cout << "\tMake selection (1-" << deviceInfos.size() << "): ";
		std::cin >> selection;
		
		if (std::cin.fail())
		{
			std::cin.clear();
			while (std::cin.get() != '\n');
			std::cout << "\tInvalid input. Please enter a number.\n";			
		}
		else if (selection <= 0 || selection > deviceInfos.size())
        {
            std::cout << "\tInvalid device selected. Please select a device in the range (1-" << deviceInfos.size() << ").\n";
        }
		
	} while (selection <= 0 || selection > deviceInfos.size());
    
    return deviceInfos[selection - 1];
}

GenICam::gcstring Device_Init(Arena::IDevice* pDevice)
{
	//打印相机初始图像格式
	GenICam::gcstring pixelFormatInitial = Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "PixelFormat");
	std::cout << "\tpixelFormatInitial:" << pixelFormatInitial << "\n";
	
	//初始化相机参数
	// enable stream auto negotiate packet size
	Arena::SetNodeValue<bool>(
	pDevice->GetTLStreamNodeMap(),
	"StreamAutoNegotiatePacketSize",
	true);

	// bool StreamAutoNegotiatePacketSize = Arena::GetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize");
	// std::cout << "\t" << "Device Init" << StreamAutoNegotiatePacketSize;


	// enable stream packet resend
	Arena::SetNodeValue<bool>(
	pDevice->GetTLStreamNodeMap(),
	"StreamPacketResendEnable",
	true);	

	//设置图像格式
	std::cout << "\tSet PolarizedAngles_0d_45d_90d_135d_BayerRG8 to pixel format\n";
	Arena::SetNodeValue<GenICam::gcstring>(
		pDevice->GetNodeMap(),
		"PixelFormat",
		"PolarizedAngles_0d_45d_90d_135d_BayerRG8");

	std::cout << "\tDevice Init\n";

	return pixelFormatInitial;
}

float get_depth_scale(rs2::device dev)
{
    // Go over the device's sensors
    for (rs2::sensor& sensor : dev.query_sensors())
    {
        // Check if the sensor if a depth sensor
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
        {
            return dpt.get_depth_scale();
        }
    }
    throw std::runtime_error("Device does not have a depth sensor");
}

rs2::frame depth_filter(rs2::frame& depth_img)
{
    // Declare filters
    rs2::decimation_filter dec_filter;  // Decimation - reduces depth frame density
    dec_filter.set_option(rs2_option::RS2_OPTION_FILTER_MAGNITUDE, 1);
    rs2::threshold_filter thr_filter;   // Threshold  - removes values outside recommended range
    rs2::spatial_filter spat_filter;    // Spatial    - edge-preserving spatial smoothing
    rs2::temporal_filter temp_filter;   // Temporal   - reduces temporal noise    
    rs2::hole_filling_filter hole_fill_filter;

    
    // Declare disparity transform from depth to disparity and vice versa
    const std::string disparity_filter_name = "Disparity";
    rs2::disparity_transform depth_to_disparity(true);
    rs2::disparity_transform disparity_to_depth(false);

    rs2::frame filtered = dec_filter.process(depth_img);
    filtered = thr_filter.process(filtered);
    filtered = depth_to_disparity.process(filtered);
    filtered = spat_filter.process(filtered);
    filtered = temp_filter.process(filtered);
    filtered = disparity_to_depth.process(filtered);
    filtered = hole_fill_filter.process(filtered);

    return filtered;
}

Mat hole_fill(Mat& img_depth)
{
    Mat img_fill;
    img_depth.copyTo(img_fill);
    unsigned short ave = mean(img_fill)[0];
    for(int i = 0; i < img_fill.rows; i++)
    {
        for(int j = 0; j < img_fill.cols; j++)
        {
            if(img_fill.at<unsigned short>(i,j) == 0)
            {
                img_fill.at<unsigned short>(i,j) = ave;
            }
        }
    }
    return img_fill;
}