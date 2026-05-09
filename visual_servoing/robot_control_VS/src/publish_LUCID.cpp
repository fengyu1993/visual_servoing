#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <ArenaApi.h>

#define IMAGE_TIMEOUT 2000
#define SYSTEM_TIMEOUT 100

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lucid_camera_node");
    ros::NodeHandle nh;

    // 使用 image_transport 发布图像，便于后续使用压缩传输或 rviz 查看
    image_transport::ImageTransport it(nh);
    image_transport::Publisher Iall_gray_pub = it.advertise("camera/polarized_Iall_gray", 1);
    image_transport::Publisher I0_color_pub = it.advertise("camera/polarized_I0_color", 1);
    image_transport::Publisher I45_color_pub = it.advertise("camera/polarized_I45_color", 1);
    image_transport::Publisher I90_color_pub = it.advertise("camera/polarized_I90_color", 1);
    image_transport::Publisher I135_color_pub = it.advertise("camera/polarized_I135_color", 1);
    image_transport::Publisher I0_gray_pub = it.advertise("camera/polarized_I0_gray", 1);
    image_transport::Publisher I45_gray_pub = it.advertise("camera/polarized_I45_gray", 1);
    image_transport::Publisher I90_gray_pub = it.advertise("camera/polarized_I90_gray", 1);
    image_transport::Publisher I135_gray_pub = it.advertise("camera/polarized_I135_gray", 1);

    Arena::ISystem* pSystem = nullptr;
    Arena::IDevice* pDevice = nullptr;

    try
    {
        ROS_INFO("Initialize Arena SDK, Search for camera");
        pSystem = Arena::OpenSystem();
        pSystem->UpdateDevices(SYSTEM_TIMEOUT);
        std::vector<Arena::DeviceInfo> deviceInfos = pSystem->GetDevices();

        if (deviceInfos.empty())
        {
            ROS_ERROR("NO Lucid Camera, Check IP");
            Arena::CloseSystem(pSystem);
            return -1;
        }

        pDevice = pSystem->CreateDevice(deviceInfos[0]);
        ROS_INFO("Connected camera: %s", Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "DeviceModelName").c_str());

        // 设置偏振图像输出格式
        GenICam::gcstring anglesFormat = "PolarizedAngles_0d_45d_90d_135d_BayerRG8";
        Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "PixelFormat", anglesFormat);
        // 优化网络传输
        Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", true);
        Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamPacketResendEnable", true);
        // 启动视频流
        pDevice->StartStream();
        ROS_INFO("Video stream has started, starting to post topics: /camera/polarized_raw");
        std::vector<cv::Mat> I_all;
        while (ros::ok())
        {
            Arena::IImage* pImage = pDevice->GetImage(IMAGE_TIMEOUT);
            if (pImage)
            {
                size_t width = pImage->GetWidth();
                size_t height = pImage->GetHeight();
                size_t bpp = Arena::GetBitsPerPixel(pImage->GetPixelFormat());
                // 判断是否为 4 个字节 (0, 45, 90, 135)
                if (bpp == 32) 
                {
                    // 创建 OpenCV Mat，不拷贝数据，直接映射
                    cv::Mat raw_mat(height, width, CV_8UC4, (void*)pImage->GetData());
                    // 拆分四个角度的通道
                    std::vector<cv::Mat> planes;
                    cv::split(raw_mat, planes); 
                    // 准备存储彩色图的容器
                    cv::Mat color_0, color_45, color_90, color_135;
                    // 准备存储灰度图的容器
                    cv::Mat gray_0, gray_45, gray_90, gray_135;                  
                    // 对每个角度进行 Bayer -> BGR 转换
                    cv::cvtColor(planes[0], color_0,  cv::COLOR_BayerBG2BGR);
                    cv::cvtColor(planes[1], color_45, cv::COLOR_BayerBG2BGR);
                    cv::cvtColor(planes[2], color_90, cv::COLOR_BayerBG2BGR);
                    cv::cvtColor(planes[3], color_135,cv::COLOR_BayerBG2BGR);
                    // 对每个角度进行 BGR -> gray 转换
                    cv::cvtColor(color_0, gray_0,  cv::COLOR_BGR2GRAY);
                    cv::cvtColor(color_45, gray_45, cv::COLOR_BGR2GRAY);
                    cv::cvtColor(color_90, gray_90, cv::COLOR_BGR2GRAY);
                    cv::cvtColor(color_135, gray_135,cv::COLOR_BGR2GRAY);
                    // 在生成四幅灰度图后立即执行, 将4个单通道合并为1个4通道图像
                    std::vector<cv::Mat> gray_channels = {gray_0, gray_45, gray_90, gray_135};
                    cv::Mat gray_4ch;       
                    cv::merge(gray_channels, gray_4ch);  
                    // 转换为 ROS Image 消息 (使用 8UC4 编码)
                    std_msgs::Header header;
                    header.stamp = ros::Time::now();
                    header.frame_id = "camera_optical_frame";
                    sensor_msgs::ImagePtr msg_gray_4ch = cv_bridge::CvImage(header, "8UC4", gray_4ch).toImageMsg();
                    Iall_gray_pub.publish(msg_gray_4ch);
                    // 彩色图消息
                    sensor_msgs::ImagePtr msg_color_0 = cv_bridge::CvImage(header, "bgr8", color_0).toImageMsg();
                    sensor_msgs::ImagePtr msg_color_45 = cv_bridge::CvImage(header, "bgr8", color_45).toImageMsg();
                    sensor_msgs::ImagePtr msg_color_90 = cv_bridge::CvImage(header, "bgr8", color_90).toImageMsg();
                    sensor_msgs::ImagePtr msg_color_135 = cv_bridge::CvImage(header, "bgr8", color_135).toImageMsg();
                    // 灰度图消息
                    sensor_msgs::ImagePtr msg_gray_0 = cv_bridge::CvImage(header, "mono8", gray_0).toImageMsg();
                    sensor_msgs::ImagePtr msg_gray_45 = cv_bridge::CvImage(header, "mono8", gray_45).toImageMsg();
                    sensor_msgs::ImagePtr msg_gray_90 = cv_bridge::CvImage(header, "mono8", gray_90).toImageMsg();
                    sensor_msgs::ImagePtr msg_gray_135 = cv_bridge::CvImage(header, "mono8", gray_135).toImageMsg();
                    // 发布
                    I0_color_pub.publish(msg_color_0);
                    I45_color_pub.publish(msg_color_45);
                    I90_color_pub.publish(msg_color_90);
                    I135_color_pub.publish(msg_color_135);
                    I0_gray_pub.publish(msg_gray_0);
                    I45_gray_pub.publish(msg_gray_45);
                    I90_gray_pub.publish(msg_gray_90);
                    I135_gray_pub.publish(msg_gray_135);
                }
                else
                {
                    ROS_WARN_ONCE("接收到的图像格式不是 32bpp，当前 bpp: %zu", bpp);
                }

                // 将缓冲区还给相机队列
                pDevice->RequeueBuffer(pImage);
            }
            ros::spinOnce();
        }

        ROS_INFO("正在停止相机流...");
        pDevice->StopStream();
        pSystem->DestroyDevice(pDevice);
        Arena::CloseSystem(pSystem);
    }
    catch (GenICam::GenericException& ge)
    {
        ROS_ERROR("GenICam 错误: %s", ge.what());
    }
    catch (std::exception& ex)
    {
        ROS_ERROR("标准异常: %s", ex.what());
    }

    return 0;
}