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
#define CHANNEL1 0
#define CHANNEL2 1
#define CHANNEL3 2
#define CHANNEL4 3
#define IMGLEN 1253376

using namespace std;
using namespace cv;
using namespace sensor_msgs;
using namespace message_filters;

size_t dstWidth = 1224;
size_t dstHeight = 1024;

rs2::frame depth_filter(rs2::frame& depth_img);
Mat hole_fill(Mat& img_depth);
float get_depth_scale(rs2::device dev);
GenICam::gcstring Device_Init(Arena::IDevice* pDevice);
Arena::DeviceInfo SelectDevice(std::vector<Arena::DeviceInfo>& deviceInfos);
void GetPolarizedData(Arena::IDevice* pDevice, uint8_t* pDst);
Mat GetPolarizedData(Arena::IDevice* pDevice);
void FetchData(const uint8_t* pSrc_Angles, size_t srcBytesPerPixel_Angles, uint8_t* pDst);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "publish_LUCID");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh); 
    image_transport::Publisher polar_pub = it.advertise("/LUCID/polarized_image_raw", 1);

    ros::Rate loop_rate(50);

	bool exceptionThrown = false;
    try
    {
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
		size_t dstBitsPerPixel = 3 * 8;
		size_t dstBytesPerPixel = dstBitsPerPixel / 8;
		size_t dstStride = dstWidth * dstBitsPerPixel / 8;
		size_t dstDataSize = dstWidth * dstHeight * dstBitsPerPixel / 8;
		uint8_t* pDst = new uint8_t[dstBytesPerPixel * dstWidth * dstHeight];
		pDevice->StartStream();

		while (ros::ok())
		{
			// Arena
            Mat srcImage = GetPolarizedData(pDevice);
			if (srcImage.empty()) {
				std::cerr << "Could not open or find the image!" << std::endl;
				continue;
			}
			// publist
			sensor_msgs::ImagePtr  polar_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", srcImage).toImageMsg();
			polar_msg->header.stamp = ros::Time::now();  
			polar_pub.publish(polar_msg);

			loop_rate.sleep();
		}
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
	// std::cout << "\t" << "Fetch Data To pDst\n";
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
		phi0 = 0.5 * atan2(I_45 - I_135, I_0 - I_90); 

		Dolp = (A * 255) / O;
		Aolp = (phi0 + 3.14159 / 2) / 3.14159 * 255.0;//�1�7�1�7�0�2�1�7�1�7

		pDst[i * 3 + CHANNEL1] = O;
		pDst[i * 3 + CHANNEL2] = Dolp;
		pDst[i * 3 + CHANNEL3] = Aolp;
	}

	// printf("\tpSrc_Angles[0]:%d\tpSrc_Angles[2]:%d\t\n", pSrc_Angles[test_num * 4 + 0], pSrc_Angles[test_num * 4 + 2]);
}

Mat GetPolarizedData(Arena::IDevice* pDevice)
{

	// std::cout << TAB2 << "111111\n";
	
	Arena::IImage* pImage_Angles_data = pDevice->GetImage(IMAGE_TIMEOUT);

	Arena::IImage* pImage_Angles = Arena::ImageFactory::Copy(pImage_Angles_data);
 		
 	pDevice->RequeueBuffer(pImage_Angles_data);

 	// src info
	uint64_t srcPF_Angles = pImage_Angles->GetPixelFormat();
	// std::cout << TAB2 << "333333\n";
	// size_t srcWidth_Angles = pImage_Angles->GetWidth();
	size_t srcHeight_Angles = pImage_Angles->GetHeight();
	size_t srcBitsPerPixel_Angles = Arena::GetBitsPerPixel(srcPF_Angles);
	size_t srcBytesPerPixel_Angles = srcBitsPerPixel_Angles / 8;
	// size_t srcStride_Angles = srcWidth_Angles * srcBitsPerPixel_Angles / 8;
	// size_t srcDataSize_Angles = srcWidth_Angles * srcHeight_Angles * srcBitsPerPixel_Angles / 8;

    cv::Mat srcImage(dstHeight, dstWidth, CV_8UC(4), (void*)pImage_Angles->GetData());

	return srcImage;
}


void GetPolarizedData(Arena::IDevice* pDevice, uint8_t* pDst)
{

	// std::cout << TAB2 << "111111\n";
	
	Arena::IImage* pImage_Angles_data = pDevice->GetImage(IMAGE_TIMEOUT);

	Arena::IImage* pImage_Angles = Arena::ImageFactory::Copy(pImage_Angles_data);
 		
 	pDevice->RequeueBuffer(pImage_Angles_data);

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

	delete pImage_Angles;
	// std::cout << TAB2 << "srcWidth:" << srcWidth_Angles << "\t\tsrcHeight:" << srcHeight_Angles << "\t\tsrcBytesPerPixel:" 
	// << srcBytesPerPixel_Angles << "\t\tsrcStride:" << srcStride_Angles << "\t\tsrcDataSize:" << srcDataSize_Angles << "\n";

	FetchData(pSrc_Angles, srcBytesPerPixel_Angles, pDst);
	
	delete pSrc_Angles;
	
}

Arena::DeviceInfo SelectDevice(std::vector<Arena::DeviceInfo>& deviceInfos)
{
    if (deviceInfos.size() == 1)
    {
        std::cout  << "\n" << "\tOnly one device detected: "  << deviceInfos[0].ModelName() << "\t" << deviceInfos[0].SerialNumber() << "\t" << deviceInfos[0].IpAddressStr() << ".\n";
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
		std::cout << "\t" << "Make selection (1-" << deviceInfos.size() << "): ";
		std::cin >> selection;
		
		if (std::cin.fail())
		{
			std::cin.clear();
			while (std::cin.get() != '\n');
			std::cout << "\t" << "Invalid input. Please enter a number.\n";			
		}
		else if (selection <= 0 || selection > deviceInfos.size())
        {
            std::cout << "\t" << "Invalid device selected. Please select a device in the range (1-" << deviceInfos.size() << ").\n";
        }
		
	} while (selection <= 0 || selection > deviceInfos.size());
    
    return deviceInfos[selection - 1];
}

GenICam::gcstring Device_Init(Arena::IDevice* pDevice)
{
	//�1�7�1�7�0�3�1�7�1�7�1�7�1�7�1�7�0�3�0�0�1�7�1�7�1�7�0�4
	GenICam::gcstring pixelFormatInitial = Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "PixelFormat");
	std::cout << "\t" << "pixelFormatInitial:" << pixelFormatInitial << "\n";
	
	//�1�7�1�7�0�3�1�7�1�7�1�7�1�7�1�7�1�7�1�7�1�7�1�7
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

	//�1�7�1�7�1�7�1�7�0�0�1�7�1�7�1�7�0�4
	std::cout << "\t" << "Set PolarizedAngles_0d_45d_90d_135d_BayerRG8 to pixel format\n";
	Arena::SetNodeValue<GenICam::gcstring>(
		pDevice->GetNodeMap(),
		"PixelFormat",
		"PolarizedAngles_0d_45d_90d_135d_BayerRG8");

	std::cout << "\t" << "Device Init\n";

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