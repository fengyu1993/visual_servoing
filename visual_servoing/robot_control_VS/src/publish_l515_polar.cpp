#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
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
#include <image_transport/image_transport.h> 
#include "ArenaApi.h"
#include "SaveApi.h"

#define IMAGE_TIMEOUT 2000
#define SYSTEM_TIMEOUT 100
#define TAB1 "  "
#define TAB2 "    "
#define NEW_WIDTH 500
#define NEW_HEIGHT 300
#define CHANNEL1 0
#define CHANNEL2 1
#define CHANNEL3 2
#define CHANNEL4 3
#define IMGLEN 1253376

using namespace std;
using namespace cv;
using namespace sensor_msgs;
using namespace message_filters;


rs2::frame depth_filter(rs2::frame& depth_img);
Mat hole_fill(Mat& img_depth);
float get_depth_scale(rs2::device dev);
GenICam::gcstring Device_Init(Arena::IDevice* pDevice);
Arena::DeviceInfo SelectDevice(std::vector<Arena::DeviceInfo>& deviceInfos);
void GetPolarizedData(Arena::IDevice* pDevice, uint8_t* pDst);
void FetchData(const uint8_t* pSrc_Angles, size_t srcBytesPerPixel_Angles, uint8_t* pDst);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "publish_l515_polar");
    ros::NodeHandle nh;
    int resolution_x, resolution_y;
    nh.getParam("resolution_x", resolution_x);
    nh.getParam("resolution_y", resolution_y);

    image_transport::ImageTransport it(nh); 
    image_transport::Publisher polar_pub = it.advertise("/camera/polarized_image", 1);
    image_transport::Publisher depth_pub = it.advertise("/camera/depth_image", 1);
    image_transport::Publisher depth_show_pub = it.advertise("/camera/depth_show_image", 1);

    ros::Rate loop_rate(20);

	bool exceptionThrown = false;
    try
    {
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

		// 循环
		int w_depth, h_depth;

		while (ros::ok())
		{
			// realsense l515
			rs2::frameset frameset = pipe.wait_for_frames();
			rs2::frameset frameset_depth = align_to_color.process(frameset);
			auto depth = frameset_depth.get_depth_frame();
			float depth_scale = get_depth_scale(profile.get_device());
			depth = depth_filter(depth);
			w_depth = depth.as<rs2::video_frame>().get_width();
			h_depth = depth.as<rs2::video_frame>().get_height();
			Mat depth_show(cv::Size(w_depth, h_depth), CV_8UC3, (void*)depth.apply_filter(color_map).get_data(), cv::Mat::AUTO_STEP);
			Mat img_depth(cv::Size(w_depth, h_depth), CV_16UC1, (void*)depth.get_data(), cv::Mat::AUTO_STEP);
			img_depth = hole_fill(img_depth);
			img_depth.convertTo(img_depth, CV_64FC1);
			img_depth = img_depth * (1000*depth_scale);
			img_depth.convertTo(img_depth, CV_16UC1);
			// Arena
			GetPolarizedData(pDevice, pDst);
			cv::Mat srcImage(dstHeight, dstWidth, CV_8UC3, pDst);
			if (srcImage.empty()) {
				std::cerr << "Could not open or find the image!" << std::endl;
				continue;
			}
			cv::Mat image_polar;
			cv::resize(srcImage, image_polar, cv::Size(NEW_WIDTH, NEW_HEIGHT));
			// publist
			sensor_msgs::ImagePtr  depth_show_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", depth_show).toImageMsg(); 
			sensor_msgs::ImagePtr  depth_raw_msg = cv_bridge::CvImage(std_msgs::Header(), "mono16", img_depth).toImageMsg(); 
			sensor_msgs::ImagePtr  polar_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_polar).toImageMsg();
			depth_show_msg->header.stamp = ros::Time::now();
			depth_raw_msg->header.stamp = ros::Time::now();
			polar_msg->header.stamp = ros::Time::now();  
			
			depth_show_pub.publish(depth_show_msg);
			depth_pub.publish(depth_raw_msg);
			polar_pub.publish(polar_msg);

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
	catch (GenICam::GenericException& ge)
	{
		std::cout << "\nGenICam exception thrown: " << ge.what() << "\n";
		exceptionThrown = true;
	}
	catch (std::exception& ex)
	{
		std::cout << "\nStandard exception thrown: " << ex.what() << "\n";
		exceptionThrown = true;
	}
	catch (...)
	{
		std::cout << "\nUnexpected exception thrown\n";
		exceptionThrown = true;
	}
}

void FetchData(const uint8_t* pSrc_Angles, size_t srcBytesPerPixel_Angles, uint8_t* pDst)
{
	// std::cout << TAB2 << "Fetch Data To pDst\n";
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

	// std::cout << TAB2 << "111111\n";
	
	Arena::IImage* pImage_Angles = pDevice->GetImage(IMAGE_TIMEOUT);
	// SaveImage(pImage, FILE_NAME_SRC);

	// std::cout << TAB2 << "2222222\n";

	// src info
	uint64_t srcPF_Angles = pImage_Angles->GetPixelFormat();
	// std::cout << TAB2 << "333333\n";
	// size_t srcWidth_Angles = pImage_Angles->GetWidth();
	size_t srcHeight_Angles = pImage_Angles->GetHeight();
	size_t srcBitsPerPixel_Angles = Arena::GetBitsPerPixel(srcPF_Angles);
	size_t srcBytesPerPixel_Angles = srcBitsPerPixel_Angles / 8;
	// size_t srcStride_Angles = srcWidth_Angles * srcBitsPerPixel_Angles / 8;
	// size_t srcDataSize_Angles = srcWidth_Angles * srcHeight_Angles * srcBitsPerPixel_Angles / 8;

	const uint8_t* pSrc_Angles = pImage_Angles->GetData();
	
	// std::cout << TAB2 << "srcWidth:" << srcWidth_Angles << "\t\tsrcHeight:" << srcHeight_Angles << "\t\tsrcBytesPerPixel:" 
	// << srcBytesPerPixel_Angles << "\t\tsrcStride:" << srcStride_Angles << "\t\tsrcDataSize:" << srcDataSize_Angles << "\n";

	FetchData(pSrc_Angles, srcBytesPerPixel_Angles, pDst);

	// clean up
	pDevice->RequeueBuffer(pImage_Angles);


	// std::cout << TAB2 << "Polarized Data is Ready\n";
	
}

Arena::DeviceInfo SelectDevice(std::vector<Arena::DeviceInfo>& deviceInfos)
{
    if (deviceInfos.size() == 1)
    {
        std::cout  << "\n" << TAB1 << "Only one device detected: "  << deviceInfos[0].ModelName() << TAB1 << deviceInfos[0].SerialNumber() << TAB1 << deviceInfos[0].IpAddressStr() << ".\n";
        std::cout  << TAB1 << "Automatically selecting this device.\n";
        return deviceInfos[0];
    }
    
    std::cout << "\nSelect device:\n";
    for (size_t i = 0; i < deviceInfos.size(); i++)
    {
        std::cout << TAB1 << i + 1 << ". " << deviceInfos[i].ModelName() << TAB1 << deviceInfos[i].SerialNumber() << TAB1 << deviceInfos[i].IpAddressStr() << "\n";
    }
    size_t selection = 0;
	
	do
	{
		std::cout << TAB1 << "Make selection (1-" << deviceInfos.size() << "): ";
		std::cin >> selection;
		
		if (std::cin.fail())
		{
			std::cin.clear();
			while (std::cin.get() != '\n');
			std::cout << TAB1 << "Invalid input. Please enter a number.\n";			
		}
		else if (selection <= 0 || selection > deviceInfos.size())
        {
            std::cout << TAB1 << "Invalid device selected. Please select a device in the range (1-" << deviceInfos.size() << ").\n";
        }
		
	} while (selection <= 0 || selection > deviceInfos.size());
    
    return deviceInfos[selection - 1];
}

GenICam::gcstring Device_Init(Arena::IDevice* pDevice)
{
	//打印相机初始图像格式
	GenICam::gcstring pixelFormatInitial = Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "PixelFormat");
	std::cout << TAB1 << "pixelFormatInitial:" << pixelFormatInitial << "\n";
	
	//初始化相机参数
	// enable stream auto negotiate packet size
	Arena::SetNodeValue<bool>(
	pDevice->GetTLStreamNodeMap(),
	"StreamAutoNegotiatePacketSize",
	true);

	// bool StreamAutoNegotiatePacketSize = Arena::GetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize");
	// std::cout << TAB2 << "Device Init" << StreamAutoNegotiatePacketSize;


	// enable stream packet resend
	Arena::SetNodeValue<bool>(
	pDevice->GetTLStreamNodeMap(),
	"StreamPacketResendEnable",
	true);	

	//设置图像格式
	std::cout << TAB2 << "Set PolarizedAngles_0d_45d_90d_135d_BayerRG8 to pixel format\n";
	Arena::SetNodeValue<GenICam::gcstring>(
		pDevice->GetNodeMap(),
		"PixelFormat",
		"PolarizedAngles_0d_45d_90d_135d_BayerRG8");

	std::cout << TAB2 << "Device Init\n";

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