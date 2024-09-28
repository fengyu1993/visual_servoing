#include <ros/ros.h>
#include "ArenaApi.h"
#include "SaveApi.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>



#define _USE_MATH_DEFINES
#include <math.h>

#define TAB1 "  "
#define TAB2 "    "


// 已经实现偏振图像的发布和订阅

// 图像发布格式为BGR8，其中各个像素的
// 通道1蓝色代表背景平均光强(I0+I90)/2
// 通道2绿色代表Dolp = (A * 255) / (O + A);
// 通道3红色代表Aolp = phi0;

// 分辨率可以改

// 发布频率平均有22.17hz

// =-=-=-=-=-=-=-=-=-
// =-=- SETTINGS =-=-
// =-=-=-=-=-=-=-=-=-

// name of file to save
// #define FILE_NAME_PATTERN "Images/PolarizedAngles_0d_45d_90d_135d_BayerRG8.jpg"
#define FILE_NAME "Images/Polarized_Img/PolarizedAngles_0d_45d_90d_135d_BGR8.jpg"
// #define FILE_NAME_SRC "Images/Cpp_Save/PolarizedAngles_BayerRG8.png"

// pixel format
// #define PIXEL_FORMAT Mono8
#define PIXEL_FORMAT BGR8

// image timeout
#define IMAGE_TIMEOUT 2000

// system timeout
#define SYSTEM_TIMEOUT 100

#define CHANNEL1 0
#define CHANNEL2 1
#define CHANNEL3 2
#define CHANNEL4 3


//1224 * 1024
#define IMGLEN 1253376

#define NEW_WIDTH 500
#define NEW_HEIGHT 300


// =-=-=-=-=-=-=-=-=-
// =-=- EXAMPLE -=-=-
// =-=-=-=-=-=-=-=-=-




void FetchData(const uint8_t* pSrc_Angles, size_t srcBytesPerPixel_Angles, uint8_t* pDst)
{
	std::cout << TAB2 << "Fetch Data To pDst\n";
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


	std::cout << TAB2 << "Polarized Data is Ready\n";
	
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

// =-=-=-=-=-=-=-=-=-
// =- PREPARATION -=-                              
// =- & CLEAN UP =-=-
// =-=-=-=-=-=-=-=-=-

int main(int argc, char *argv[])
{
	// flag to track when an exception has been thrown
	bool exceptionThrown = false;
	std::cout << "Hello";

	ros::init(argc, argv, "pub_polarized_img_node");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/polarizedimage", 1);

	ros::Rate loop_rate(50);

	try
	{
		// prepare example
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



		// run example
		std::cout << TAB2 << "Creat New Image\n";
		// dst info
		size_t dstWidth = 1224;
		size_t dstHeight = 1024;
		size_t dstBitsPerPixel = 3 * 8;
		size_t dstBytesPerPixel = dstBitsPerPixel / 8;
		size_t dstStride = dstWidth * dstBitsPerPixel / 8;
		size_t dstDataSize = dstWidth * dstHeight * dstBitsPerPixel / 8;
		uint8_t* pDst = new uint8_t[dstBytesPerPixel * dstWidth * dstHeight];
		std::cout << TAB2 << "dstWidth:" << dstWidth << "\t\tdstHeight:" << dstHeight << "\t\tdstBytesPerPixel:" 
		<< dstBytesPerPixel << "\t\tdstStride:" << dstStride << "\t\tdstDataSize:" << dstDataSize << "\n";


		
		
		// retrieve image
		std::cout << TAB2 << "Acquire PolarizedAngles_0d_45d_90d_135d_BayerRG8 image\n";
		pDevice->StartStream();



		while (nh.ok())
		{
			std::cout << TAB2 << "Get Polarized Data\n";
			GetPolarizedData(pDevice, pDst);


			std::cout << TAB2 << "Convert To CVMat\n";
			cv::Mat srcImage(dstHeight, dstWidth, CV_8UC3, pDst);
			if (srcImage.empty()) {
				std::cerr << "Could not open or find the image!" << std::endl;
				continue;
			}
			// 创建一个空的Mat对象用于存储调整后的图像
			cv::Mat image;
			// 使用cv::resize函数调整图像大小
			cv::resize(srcImage, image, cv::Size(NEW_WIDTH, NEW_HEIGHT));

			
			sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
			msg->header.stamp = ros::Time::now();  // 更新时间戳

			pub.publish(msg);
			//Use "rqt_image_view" to view the image
			ros::spinOnce();
			loop_rate.sleep();
		}
		

		// clean up
		pDst = NULL;
		delete[] pDst;


		std::cout << "\nExample complete\n";

		// clean up example
		//关闭相机
		pDevice->StopStream();
		// return nodes to their initial values
		Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "PixelFormat", pixelFormatInitial);
		// Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "PixelFormat", "BayerRG8");
		pSystem->DestroyDevice(pDevice);
		Arena::CloseSystem(pSystem);
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

	std::cout << "Press enter to complete\n";
	// std::cin.ignore();
	// std::getchar();

	if (exceptionThrown)
		return -1;
	else
		return 0;
}