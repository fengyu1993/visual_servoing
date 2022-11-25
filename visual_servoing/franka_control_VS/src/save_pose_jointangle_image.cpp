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

using namespace std;
using namespace cv;
using namespace sensor_msgs;
using namespace message_filters;

Mat joint_group_positions;
Mat T_link0_camera;
Mat img_rgb;
Mat img_depth;


rs2::frame depth_filter(rs2::frame& depth_img);
void callback(const ImageConstPtr& image_color_msg, const ImageConstPtr& image_depth_msg);
cv::Mat Quaternion2Matrix (cv::Mat q);
Mat get_T(tf::StampedTransform  pose);
void write_to_excel(Mat data, ofstream& oFile);
string get_date_time();
void save_joint_positions(moveit::planning_interface::MoveGroupInterface& move_group_interface);
void save_camera_pose();
void write_image();
void write_data();
void save_image(rs2::frame color, rs2::frame depth);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "save_pose_image");
    ros::NodeHandle nh;

    ros::Rate loop_rate(30);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "panda_arm";
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
    
    // Create a pipeline to easily configure and start the camera
    rs2::pipeline pipe;
    rs2::config cfg;
    rs2::colorizer color_map;
    cfg.enable_stream(RS2_STREAM_COLOR, 320, 240);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480);
    pipe.start(cfg);
    rs2::align align_to_color(RS2_STREAM_COLOR);

    cout<< "Press space to save rgb_raw and depth_raw to a file."<<endl;
 
    while (ros::ok())
    {
        // read image
        rs2::frameset frameset = pipe.wait_for_frames();
        rs2::frameset frameset_depth = align_to_color.process(frameset);
        // With the aligned frameset we proceed as usual
        auto depth = frameset_depth.get_depth_frame();
        auto color = frameset_depth.get_color_frame();
        // filter
        depth = depth_filter(depth);
        // show
        const int w_depth = depth.as<rs2::depth_frame>().get_width();
        const int h_depth = depth.as<rs2::depth_frame>().get_height();
        const int w_rgb = color.as<rs2::video_frame>().get_width();
        const int h_rgb = color.as<rs2::video_frame>().get_height();
        Mat rgb_show(cv::Size(w_rgb, h_rgb), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
        Mat depth_show(cv::Size(w_depth, h_depth), CV_8UC3, (void*)depth.apply_filter(color_map).get_data(), cv::Mat::AUTO_STEP);
        imshow("Color", rgb_show);
        imshow("Depth", depth_show);
        // save
        if((char)waitKey(10) == 32)
        {
            save_image(color, depth);
            save_joint_positions(move_group_interface);
            save_camera_pose();
            write_image();
            write_data();
        }
        loop_rate.sleep();
    }
 
 
    return 0;
}

void save_image(rs2::frame color, rs2::frame depth)
{
    const int w_depth = depth.as<rs2::video_frame>().get_width();
    const int h_depth = depth.as<rs2::video_frame>().get_height();
    const int w_rgb = color.as<rs2::video_frame>().get_width();
    const int h_rgb = color.as<rs2::video_frame>().get_height();
    Mat img_rgb_temp(cv::Size(w_rgb, h_rgb), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
    Mat img_depth_temp(cv::Size(w_depth, h_depth), CV_16UC1, (void*)depth.get_data(), cv::Mat::AUTO_STEP);
    img_rgb_temp.copyTo(img_rgb);
    img_depth_temp.copyTo(img_depth);
}

void write_data()
{
    ofstream oFile;
    string location = "/home/cyh/Work/visual_servoing_ws/src/visual_servoing/franka_control_VS/param/";
    string time = get_date_time();
    string excel_name = location + time + "_pose_data.xls";
    oFile.open(excel_name, ios::out|ios::trunc);
    oFile << "joint angle" << endl;
    write_to_excel(joint_group_positions, oFile);
    oFile << "pose" << endl;
    write_to_excel(T_link0_camera, oFile);
    oFile.close();
}

void write_image()
{
    // 保存深度图
    imwrite("/home/cyh/Work/visual_servoing_ws/src/visual_servoing/franka_control_VS/param/image_depth_desired.png", img_depth);
    // 保存彩色图
    imwrite("/home/cyh/Work/visual_servoing_ws/src/visual_servoing/franka_control_VS/param/image_rgb_desired.png", img_rgb);
}

// 保存相机位姿
void save_camera_pose()
{
    tf::TransformListener listener;
    tf::StampedTransform transform;
    listener.waitForTransform("panda_link0", "camera_link", ros::Time(0), ros::Duration(3.0));
    listener.lookupTransform("panda_link0", "camera_link", ros::Time(0), transform);
    T_link0_camera = get_T(transform);
    cout << "T_link0_camera = \n" << T_link0_camera << endl;
}

void save_joint_positions(moveit::planning_interface::MoveGroupInterface& move_group_interface)
{
    std::vector<double> joint_positions = move_group_interface.getCurrentJointValues();
    joint_group_positions = (Mat_<double>(1,7) << joint_positions[0], joint_positions[1], 
                            joint_positions[2], joint_positions[3], joint_positions[4], 
                            joint_positions[5], joint_positions[6]);
    cout << "joint_angle = \n" <<  joint_group_positions << endl;
}
rs2::frame depth_filter(rs2::frame& depth_img)
{
    // Declare filters
    rs2::decimation_filter dec_filter;  // Decimation - reduces depth frame density
    dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 1);
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

