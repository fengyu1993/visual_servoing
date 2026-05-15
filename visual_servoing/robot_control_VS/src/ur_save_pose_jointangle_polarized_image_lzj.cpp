#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <robot_control_VS/PolarizedImages.h>

#include <message_filters/sync_policies/approximate_time.h>

using namespace std;
using namespace cv;
using namespace sensor_msgs;
using namespace message_filters;

Mat joint_group_positions;
Mat T_base_camera;
Mat img_polar_color_0, img_polar_color_45, img_polar_color_90, img_polar_color_135;
Mat img_polar_gray_0, img_polar_gray_45, img_polar_gray_90, img_polar_gray_135;
Mat S0_color, S0_gray, S0_color_display, S0_gray_display;

void write_to_excel(Mat data, ofstream& oFile)
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


// ��ȡ��ǰ�����ʱ��
string get_date_time()
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

void write_data()
{
    ofstream oFile;
    string location = "/workspace/catkin_ws/src/visual_servoing-master/visual_servoing/robot_control_VS/param/";
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
    // �����ɫͼ
    imwrite("/workspace/catkin_ws/src/visual_servoing-master/visual_servoing/robot_control_VS/param/DVS_Polarized/image_polar_init.png", S0_color);
    imwrite("/workspace/catkin_ws/src/visual_servoing-master/visual_servoing/robot_control_VS/param/DVS_Polarized/image_polar_color_0.png", img_polar_color_0);
    imwrite("/workspace/catkin_ws/src/visual_servoing-master/visual_servoing/robot_control_VS/param/DVS_Polarized/image_polar_color_45.png", img_polar_color_45);
    imwrite("/workspace/catkin_ws/src/visual_servoing-master/visual_servoing/robot_control_VS/param/DVS_Polarized/image_polar_color_90.png", img_polar_color_90);
    imwrite("/workspace/catkin_ws/src/visual_servoing-master/visual_servoing/robot_control_VS/param/DVS_Polarized/image_polar_color_135.png", img_polar_color_135);

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




void save_joint_positions(moveit::planning_interface::MoveGroupInterface& move_group_interface)
{
    std::vector<double> joint_positions = move_group_interface.getCurrentJointValues();
    joint_group_positions = (Mat_<double>(1,6) << joint_positions[0], joint_positions[1], 
                            joint_positions[2], joint_positions[3], joint_positions[4], 
                            joint_positions[5]);
    cout << "joint_angle = \n" <<  joint_group_positions << endl;
}

// �������λ��
void save_camera_pose()
{
    tf::TransformListener listener;
    tf::StampedTransform transform;
    try {
        if (listener.waitForTransform("base_link", "camera_polar_frame", ros::Time(0), ros::Duration(3.0))) 
        {
            listener.lookupTransform("base_link", "camera_polar_frame", ros::Time(0), transform);
            T_base_camera = get_T(transform);
            cout << ">>> Get true pose successfully\n" << T_base_camera << endl;
        } 
        else 
        {
            ROS_WARN("!!! WARNING: camera_polar_frame NOT FOUND! Please check TF Tree");
            T_base_camera = Mat::eye(4, 4, CV_64FC1);
        }
    } 
    catch (tf::TransformException &ex) {
        ROS_ERROR("TF is exection: %s", ex.what());
        T_base_camera = Mat::eye(4, 4, CV_64FC1);
    }
    // listener.waitForTransform("base_link", "camera_polar_frame", ros::Time(0), ros::Duration(3.0));
    // listener.lookupTransform("base_link", "camera_polar_frame", ros::Time(0), transform);
    // T_base_camera = get_T(transform);
    // cout << "T_base_camera = \n" << T_base_camera << endl;
}

void Callback(const robot_control_VS::PolarizedImagesConstPtr& image_polar_color_msg, const robot_control_VS::PolarizedImagesConstPtr& image_polar_gray_msg)
{
    // ROS_INFO("cyh");
    try {
        cv_bridge::CvImagePtr cv_ptr_color_0 = cv_bridge::toCvCopy(image_polar_color_msg->image_0, "bgr8");
        cv_bridge::CvImagePtr cv_ptr_color_45 = cv_bridge::toCvCopy(image_polar_color_msg->image_45, "bgr8");
        cv_bridge::CvImagePtr cv_ptr_color_90 = cv_bridge::toCvCopy(image_polar_color_msg->image_90, "bgr8");
        cv_bridge::CvImagePtr cv_ptr_color_135 = cv_bridge::toCvCopy(image_polar_color_msg->image_135, "bgr8");
        cv_bridge::CvImagePtr cv_ptr_gray_0 = cv_bridge::toCvCopy(image_polar_gray_msg->image_0, "mono8");
        cv_bridge::CvImagePtr cv_ptr_gray_45 = cv_bridge::toCvCopy(image_polar_gray_msg->image_45, "mono8");
        cv_bridge::CvImagePtr cv_ptr_gray_90 = cv_bridge::toCvCopy(image_polar_gray_msg->image_90, "mono8");
        cv_bridge::CvImagePtr cv_ptr_gray_135 = cv_bridge::toCvCopy(image_polar_gray_msg->image_135, "mono8");


        img_polar_color_0 = cv_ptr_color_0->image.clone();
        img_polar_color_45 = cv_ptr_color_45->image.clone();
        img_polar_color_90 = cv_ptr_color_90->image.clone();
        img_polar_color_135 = cv_ptr_color_135->image.clone();
        img_polar_gray_0 = cv_ptr_gray_0->image.clone();
        img_polar_gray_45 = cv_ptr_gray_45->image.clone();
        img_polar_gray_90 = cv_ptr_gray_90->image.clone();
        img_polar_gray_135 = cv_ptr_gray_135->image.clone();

        cv::Mat temp;
        img_polar_color_0.convertTo(S0_color, CV_64FC3); 
        img_polar_color_45.convertTo(temp, CV_64FC3); S0_color += temp;
        img_polar_color_90.convertTo(temp, CV_64FC3); S0_color += temp;
        img_polar_color_135.convertTo(temp, CV_64FC3); S0_color += temp;
        S0_color = S0_color / 4.0;
        S0_color.convertTo(S0_color_display, CV_8UC3);


        img_polar_gray_0.convertTo(S0_gray, CV_64FC1);
        img_polar_gray_45.convertTo(temp, CV_64FC1); S0_gray += temp;
        img_polar_gray_90.convertTo(temp, CV_64FC1); S0_gray += temp;
        img_polar_gray_135.convertTo(temp, CV_64FC1); S0_gray += temp;
        S0_gray = S0_gray / 4.0;
        S0_gray.convertTo(S0_gray_display, CV_8UC1);          
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge ת���쳣: %s", e.what());
    }
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "save_pose_image");
    ros::NodeHandle nh;

	int resolution_x, resolution_y;
	nh.getParam("resolution_x", resolution_x);
    nh.getParam("resolution_y", resolution_y);
	// S0_color = Mat::zeros(resolution_y, resolution_x, CV_64FC3);
	// S0_gray = Mat::zeros(resolution_y, resolution_x, CV_64FC1);

	// static const std::string PLANNING_GROUP = "manipulator";
    // moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

	message_filters::Subscriber<robot_control_VS::PolarizedImages>      image_polar_color_sub(nh,"camera/polarized_Iall_color", 1);
    message_filters::Subscriber<robot_control_VS::PolarizedImages>      image_polar_gray_sub(nh,"camera/polarized_Iall_gray", 1);
	// TimeSynchronizer<robot_control_VS::PolarizedImages, robot_control_VS::PolarizedImages>  *sync;
    // sync = new TimeSynchronizer<robot_control_VS::PolarizedImages, robot_control_VS::PolarizedImages>(image_polar_color_sub, image_polar_gray_sub, 1);
    // sync->registerCallback(boost::bind(&Callback, _1, _2));
    typedef message_filters::sync_policies::ApproximateTime<robot_control_VS::PolarizedImages, robot_control_VS::PolarizedImages> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> *sync;
    sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), image_polar_color_sub, image_polar_gray_sub);
    sync->registerCallback(boost::bind(&Callback, _1, _2));

	ros::Rate loop_rate(30);
    ros::AsyncSpinner spinner(1);
    spinner.start();

	cv::namedWindow("Polar_color", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Polar_gray", cv::WINDOW_AUTOSIZE);

    bool is_ready_to_save = false;
	while (ros::ok())
    {
        ros::spinOnce();
        // imshow("Polar_color", S0_color_display);
        // imshow("Polar_gray", S0_gray_display);

        if (!S0_color_display.empty() && !S0_gray_display.empty())
        {
            imshow("Polar_color", S0_color_display);
            imshow("Polar_gray", S0_gray_display);
        }

        int key = waitKey(10);
        if(key == 32)
        {
            if (!is_ready_to_save) 
            {
                // save_joint_positions(move_group_interface);
                save_camera_pose();
                std::cout << ">>> Data is ready, press space to start write..." << std::endl;
                is_ready_to_save = true;
            }
            else 
            {
                if (!S0_color.empty())
                {
                    write_data();
                    write_image();
                    std::cout << ">>> Save image successfully" << std::endl;
                }
                else
                {
                    std::cout << "ERROR!  Check camera topic" << std::endl;
                }
                is_ready_to_save = false;
            }
        }
        else if (key == 27)
        {
            break; 
        }
        // imshow("Polar_color", S0_color_display);
        // imshow("Polar_gray", S0_gray_display);

        // if((char)waitKey(10) == 32) // Spacebar
        // {
        //     save_joint_positions(move_group_interface);
        //     save_camera_pose();
        //     std::cout << "Save data successfully" << endl;
        //     std::cout << "Press space to start write ..." << std::endl;
        //     if((char)waitKey() == 32)
        //     {
        //         write_data();
        //         write_image();
        //         std::cout << "Save image successfully" << endl;
        //     }
        // }

        loop_rate.sleep();		
	}

    return 0;
}
