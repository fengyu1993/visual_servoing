#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <ArenaApi.h>
#include <robot_control_VS/PolarizedImages.h> 

#define IMAGE_TIMEOUT 3000
#define SYSTEM_TIMEOUT 100

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lucid_camera_node");
    ros::NodeHandle nh;
    sensor_msgs::ImagePtr msg_color_0, msg_color_45, msg_color_90, msg_color_135;
    sensor_msgs::ImagePtr msg_gray_0, msg_gray_45, msg_gray_90, msg_gray_135;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher I0_color_pub, I45_color_pub, I90_color_pub, I135_color_pub;
    image_transport::Publisher I0_gray_pub, I45_gray_pub, I90_gray_pub, I135_gray_pub;
    size_t width, height, bpp;
    cv::Mat color_0, color_45, color_90, color_135, color_S0_double, color_S0_8U;
    cv::Mat gray_0, gray_45, gray_90, gray_135, gray_S0_double, gray_S0_8U;  
    cv::Mat plane_0, plane_45, plane_90, plane_135;
    size_t plane_size; 
    uint8_t* data_ptr;


    bool flag = false;
    // 自定义消息发布
    ros::Publisher Iall_color_pub = nh.advertise<robot_control_VS::PolarizedImages>("camera/polarized_Iall_color", 1);
    ros::Publisher Iall_gray_pub = nh.advertise<robot_control_VS::PolarizedImages>("camera/polarized_Iall_gray", 1);
    if(flag){
        I0_color_pub = it.advertise("camera/polarized_I0_color", 1);
        I45_color_pub = it.advertise("camera/polarized_I45_color", 1);
        I90_color_pub = it.advertise("camera/polarized_I90_color", 1);
        I135_color_pub = it.advertise("camera/polarized_I135_color", 1);
        I0_gray_pub = it.advertise("camera/polarized_I0_gray", 1);
        I45_gray_pub = it.advertise("camera/polarized_I45_gray", 1);
        I90_gray_pub = it.advertise("camera/polarized_I90_gray", 1);
        I135_gray_pub = it.advertise("camera/polarized_I135_gray", 1);
    }
 
    Arena::ISystem* pSystem = nullptr;
    Arena::IDevice* pDevice = nullptr;
    Arena::IImage* pImage; 
    std::vector<Arena::DeviceInfo> deviceInfos;
    std::vector<cv::Mat> planes;
    GenICam::gcstring anglesFormat = "PolarizedAngles_0d_45d_90d_135d_BayerRG8";
    try
    {
        ROS_INFO("Initialize Arena SDK, Search for camera");
        pSystem = Arena::OpenSystem();
        pSystem->UpdateDevices(SYSTEM_TIMEOUT);
        deviceInfos = pSystem->GetDevices();

        if (deviceInfos.empty())
        {
            ROS_ERROR("NO Lucid Camera, Check IP");
            Arena::CloseSystem(pSystem);
            return -1;
        }

        pDevice = pSystem->CreateDevice(deviceInfos[0]);
        ROS_INFO("Connected camera: %s", Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "DeviceModelName").c_str());

        // 设置偏振图像输出格式
        Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "PixelFormat", anglesFormat);
        // 优化网络传输
        Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", true);
        Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamPacketResendEnable", true);
        // 启动视频流
        pDevice->StartStream();
        ROS_INFO("Video stream has started, starting to post topics: /camera/polarized_raw");
        while (ros::ok())
        {
            pImage = pDevice->GetImage(IMAGE_TIMEOUT);
            if (pImage)
            {
                width = pImage->GetWidth();
                height = pImage->GetHeight();
                bpp = Arena::GetBitsPerPixel(pImage->GetPixelFormat());
                // 判断是否为 4 个字节 (0, 45, 90, 135)
                if (bpp == 32) 
                {
                    // 创建 OpenCV Mat，不拷贝数据，直接映射
                    cv::Mat raw_mat(height, width, CV_8UC4, (void*)pImage->GetData());
                    // 拆分四个角度的通道
                    cv::split(raw_mat, planes);                
                    // 对每个角度进行 Bayer -> BGR 转换
                    cv::cvtColor(planes[0], color_0,  cv::COLOR_BayerBG2BGR);
                    cv::cvtColor(planes[1], color_45, cv::COLOR_BayerBG2BGR);
                    cv::cvtColor(planes[2], color_90, cv::COLOR_BayerBG2BGR);
                    cv::cvtColor(planes[3], color_135,cv::COLOR_BayerBG2BGR);
                    // 计算S0彩色
                    cv::Mat temp;
                    color_0.convertTo(color_S0_double, CV_64FC3); 
                    color_45.convertTo(temp, CV_64FC3); color_S0_double += temp;
                    color_90.convertTo(temp, CV_64FC3); color_S0_double += temp;
                    color_135.convertTo(temp, CV_64FC3); color_S0_double += temp;
                    color_S0_double /= 4.0;
                    color_S0_double.convertTo(color_S0_8U, CV_8UC3);
                    // 对每个角度进行 BGR -> gray 转换
                    cv::cvtColor(color_0, gray_0,  cv::COLOR_BGR2GRAY);
                    cv::cvtColor(color_45, gray_45, cv::COLOR_BGR2GRAY);
                    cv::cvtColor(color_90, gray_90, cv::COLOR_BGR2GRAY);
                    cv::cvtColor(color_135, gray_135,cv::COLOR_BGR2GRAY);   
                    // 计算S0 灰度
                    gray_0.convertTo(gray_S0_double, CV_64FC1);
                    gray_45.convertTo(temp, CV_64FC1); gray_S0_double += temp;
                    gray_90.convertTo(temp, CV_64FC1); gray_S0_double += temp;
                    gray_135.convertTo(temp, CV_64FC1); gray_S0_double += temp;
                    gray_S0_double /= 4.0;
                    gray_S0_double.convertTo(gray_S0_8U, CV_8UC1);
                    // 转换为 ROS Image 消息 
                    std_msgs::Header header;
                    header.frame_id = "camera_optical_frame";
                    // 用自定义的消息
                    robot_control_VS::PolarizedImagesPtr msg_polarized_color(new robot_control_VS::PolarizedImages());
                    robot_control_VS::PolarizedImagesPtr msg_polarized_gray(new robot_control_VS::PolarizedImages());
                    // 填充彩色图 (bgr8)
                    msg_polarized_color->image_0 = *(cv_bridge::CvImage(header, "bgr8", color_0).toImageMsg());
                    msg_polarized_color->image_45 = *(cv_bridge::CvImage(header, "bgr8", color_45).toImageMsg());
                    msg_polarized_color->image_90 = *(cv_bridge::CvImage(header, "bgr8", color_90).toImageMsg());
                    msg_polarized_color->image_135 = *(cv_bridge::CvImage(header, "bgr8", color_135).toImageMsg());
                    msg_polarized_color->image_S0 = *(cv_bridge::CvImage(header, "bgr8", color_S0_8U).toImageMsg());
                    // 填充灰度图 (mono8)
                    msg_polarized_gray->image_0 = *(cv_bridge::CvImage(header, "mono8", gray_0).toImageMsg());
                    msg_polarized_gray->image_45 = *(cv_bridge::CvImage(header, "mono8", gray_45).toImageMsg());
                    msg_polarized_gray->image_90 = *(cv_bridge::CvImage(header, "mono8", gray_90).toImageMsg());
                    msg_polarized_gray->image_135 = *(cv_bridge::CvImage(header, "mono8", gray_135).toImageMsg());
                    msg_polarized_gray->image_S0 = *(cv_bridge::CvImage(header, "mono8", gray_S0_8U).toImageMsg());                    
                    if(flag){
                        // 彩色图消息
                        msg_color_0 = cv_bridge::CvImage(header, "bgr8", color_0).toImageMsg();
                        msg_color_45 = cv_bridge::CvImage(header, "bgr8", color_45).toImageMsg();
                        msg_color_90 = cv_bridge::CvImage(header, "bgr8", color_90).toImageMsg();
                        msg_color_135 = cv_bridge::CvImage(header, "bgr8", color_135).toImageMsg();
                        // 灰度图消息
                        msg_gray_0 = cv_bridge::CvImage(header, "mono8", gray_0).toImageMsg();
                        msg_gray_45 = cv_bridge::CvImage(header, "mono8", gray_45).toImageMsg();
                        msg_gray_90 = cv_bridge::CvImage(header, "mono8", gray_90).toImageMsg();
                        msg_gray_135 = cv_bridge::CvImage(header, "mono8", gray_135).toImageMsg();
                    }
                    // 时间戳
                    header.stamp = ros::Time::now();
                    msg_polarized_color->header = header;
                    msg_polarized_gray->header = header;
                    if(flag){
                        msg_color_0->header = header;
                        msg_color_45->header = header;
                        msg_color_90->header = header;
                        msg_color_135->header = header;
                        msg_gray_0->header = header;
                        msg_gray_45->header = header;
                        msg_gray_90->header = header;
                        msg_gray_135->header = header;
                    }
                    // 发布
                    Iall_color_pub.publish(msg_polarized_color);
                    Iall_gray_pub.publish(msg_polarized_gray);
                    if(flag){
                        I0_color_pub.publish(msg_color_0);
                        I45_color_pub.publish(msg_color_45);
                        I90_color_pub.publish(msg_color_90);
                        I135_color_pub.publish(msg_color_135);
                        I0_gray_pub.publish(msg_gray_0);
                        I45_gray_pub.publish(msg_gray_45);
                        I90_gray_pub.publish(msg_gray_90);
                        I135_gray_pub.publish(msg_gray_135);
                    }
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

