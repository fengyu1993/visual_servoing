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


// ??????????????

// ???????BGR8????????
// ??1??????????(I0+I90)/2
// ??2????Dolp = (A * 255) / (O + A);
// ??3????Aolp = phi0;

// ??????

// ???????22.17hz

// =-=-=-=-=-=-=-=-=-
// =-=- SETTINGS =-=-
// =-=-=-=-=-=-=-=-=-

// name of file to save
// #define FILE_NAME_PATTERN "Images/PolarizedAngles_0d_45d_90d_135d_BayerRG8.jpg"
#define FILE_NAME "Images/Polarized_Img/PolarizedAngles_0d_45d_90d_135d_BGR8111.jpg"
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
#define NEW_HEIGHT 500

#define WIDTH 1224
#define HEIGHT 1024


// =-=-=-=-=-=-=-=-=-
// =-=- EXAMPLE -=-=-
// =-=-=-=-=-=-=-=-=-




void FetchData(const uint8_t* pSrc_Angles, size_t srcBytesPerPixel_Angles, uint8_t* pDst)
{
	std::cout << TAB2 << "Fetch Data To pDst\n";
	uint8_t I_0 = 0;
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

		
		A = sqrt(((I_0 - I_90) * (I_0 - I_90) + (I_45 - I_135) * (I_45 - I_135)) / 4.0);
		phi0 = 0.5 * atan2(I_45 - I_135, I_0 - I_90); // ??atan2???????

		O = MAX((I_0 + I_90), (I_45 + I_135)) / 2.0;

		Dolp = (A * 255.0) / (O + 0.001);
		Aolp = (phi0 + CV_PI) * 180.0 / CV_PI;

		pDst[i * 3 + CHANNEL1] = O;
		pDst[i * 3 + CHANNEL2] = Dolp;
		pDst[i * 3 + CHANNEL3] = Aolp;
	}

	// printf("\tpSrc_Angles[0]:%d\tpSrc_Angles[2]:%d\t\n", pSrc_Angles[test_num * 4 + 0], pSrc_Angles[test_num * 4 + 2]);
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
	//??????????
	GenICam::gcstring pixelFormatInitial = Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "PixelFormat");
	std::cout << TAB1 << "pixelFormatInitial:" << pixelFormatInitial << "\n";
	
	//???????
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

	//??????
	std::cout << TAB2 << "Set PolarizedAngles_0d_45d_90d_135d_BayerRG8 to pixel format\n";
	Arena::SetNodeValue<GenICam::gcstring>(
		pDevice->GetNodeMap(),
		"PixelFormat",
		"PolarizedAngles_0d_45d_90d_135d_BayerRG8");

	//??????
	// std::cout << TAB2 << "Set PolarizedAngles_0d_45d_90d_135d_BayerRG8 to pixel format\n";
	Arena::SetNodeValue<int64_t>(
		pDevice->GetNodeMap(),
		"Width",
		WIDTH);

	Arena::SetNodeValue<int64_t>(
		pDevice->GetNodeMap(),
		"Height",
		HEIGHT);

	// Arena::SetNodeValue<bool>(
	// 	pDevice->GetTLStreamNodeMap(),
	// 	"Acquisition Frame Rate Enable",
	// 	true);

	// double Framerate = Arena::GetNodeValue<double>(pDevice->GetNodeMap(), "Acquisition Frame Rate");
	// std::cout << TAB1 << "Framerate:" << Framerate << "\n";

	std::cout << TAB2 << "Device Init\n";

	return pixelFormatInitial;
}



cv::Mat CenterAlignCrop(cv::Mat srcImage, size_t width, size_t height)
{
	cv::Mat Image;
	if(width <= WIDTH && height <= HEIGHT)
	{
		if((width % 2) == 0 && (height % 2) == 0)
		{
			Image = srcImage(cv::Rect(WIDTH / 2 - width / 2, HEIGHT / 2 - height / 2, width, height));
		}
		else
		{
			std::cout << TAB2 << "Size Type is Wrong,  Using 1224 * 1024\n";
			Image = srcImage;
		}
	}
	else
	{
		std::cout << TAB2 << "New Size is Wrong,  Using 1224 * 1024\n";
		Image = srcImage;
	}
	return Image;
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

	ros::init(argc, argv, "publish_LUCID");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/LUCID/polarized_image_raw", 1);

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
		size_t dstWidth = WIDTH;
		size_t dstHeight = HEIGHT;
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

		// cv::namedWindow("view");
		// cv::startWindowThread();
		// cv::namedWindow("view1");
		// cv::startWindowThread();

		cv::Mat kernel = (cv::Mat_<float>(3, 3) << 
				1.0f / 9.0f, 1.0f / 9.0f, 1.0f / 9.0f,
                1.0f / 9.0f, 1.0f / 9.0f, 1.0f / 9.0f,
                1.0f / 9.0f, 1.0f / 9.0f, 1.0f / 9.0f);



		



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
			// cv::imwrite(FILE_NAME, srcImage);


			


			// ??????Mat????????????
			// cv::Mat image;
			// ??cv::resize????????
			// cv::resize(srcImage, image, cv::Size(NEW_WIDTH, NEW_HEIGHT));
			// cv::blur(image, image, cv::Size(3, 3));
			
			// ??cv::Rect????????
			cv::Mat image = CenterAlignCrop(srcImage, NEW_WIDTH, NEW_HEIGHT);

			// cv::Mat highpassImage;
			// cv::filter2D(srcImage, highpassImage, -1, kernel, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);
			// cv::imshow("view", srcImage);
			// cv::imshow("view1", image);

			
			
			
			

			
			sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", srcImage).toImageMsg();
			msg->header.stamp = ros::Time::now();  // ?????

			pub.publish(msg);
			//Use "rqt_image_view" to view the image
			ros::spinOnce();
			loop_rate.sleep();
		}
		// cv::destroyWindow("view");
		// cv::destroyWindow("view1");
		

		// clean up
		pDst = NULL;
		delete[] pDst;


		std::cout << "\nExample complete\n";

		// clean up example
		//????
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