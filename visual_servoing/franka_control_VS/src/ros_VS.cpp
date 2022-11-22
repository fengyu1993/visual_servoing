# include "ros_VS.h"


Ros_VS::Ros_VS()
{
    this->flag_success = false;

    this->nh_.getParam("control_rate", this->control_rate_);

    initialize_time_sync();

    this->pub_camera_twist_ = this->nh_.advertise<geometry_msgs::Twist>("/cartesian_velocity_node_controller/cartesian_velocity", 5);

}

void Ros_VS::initialize_time_sync()
{
    image_color_sub_.subscribe(this->nh_,"/camera/color/image_raw", 1);
    image_depth_sub_.subscribe(this->nh_,"/camera/aligned_depth_to_color/image_raw", 1);
    sync = new TimeSynchronizer<Image, Image>(image_color_sub_, image_depth_sub_, 10);
    sync->registerCallback(boost::bind(&Ros_VS::Callback, this, _1, _2));
}


void Ros_VS::get_parameters(int& resolution_x, int& resolution_y, double& lambda, double& epsilon, Mat& image_gray_desired, Mat& image_depth_desired, Mat& image_gray_initial, Mat& camera_intrinsic, Mat& pose_desired)
{
    // ��������
    this->nh_.getParam("resolution_x", resolution_x);
    this->nh_.getParam("resolution_y", resolution_y);
    this->nh_.getParam("lambda", lambda);
    this->nh_.getParam("epsilon", epsilon);
    // ͼ�����
    string loaction, name;
    this->nh_.getParam("resource_location", loaction);
    this->nh_.getParam("image_gray_desired_name", name);
    image_gray_desired = imread(loaction + name, IMREAD_GRAYSCALE);
    this->nh_.getParam("image_depth_desired_name", name);
    image_depth_desired = imread(loaction + name);  
    this->nh_.getParam("image_gray_initial_name", name);
    image_gray_initial = imread(loaction + name, IMREAD_GRAYSCALE);  
    // ����ڲ�
    XmlRpc::XmlRpcValue param_yaml;
    this->nh_.getParam("camera_intrinsic", param_yaml);
    double intrinsic_temp[9];
    for(int i=0; i<9; i++) 
    {
        intrinsic_temp[i] = param_yaml[i];
    }
    Mat camera_intrinsic_temp = Mat(3, 3, CV_64FC1, intrinsic_temp);
    camera_intrinsic_temp.copyTo(camera_intrinsic);
    // ����λ��
    this->nh_.getParam("pose_desired", param_yaml);
    double pose_temp[16];
    for(int i=0; i<16; i++) 
    {
        pose_temp[i] = param_yaml[i];
    }     
    Mat pose_desired_temp = Mat(4, 4, CV_64FC1, pose_temp);
    pose_desired_temp.copyTo(pose_desired);
}


void Ros_VS::set_resolution_parameters(int resolution_x, int resolution_y)
{
    this->nh_.setParam("/camera/realsense2_camera/depth_height", resolution_y);
    this->nh_.setParam("/camera/realsense2_camera/depth_width", resolution_x);
    this->nh_.setParam("/camera/realsense2_camera/color_height", resolution_y);
    this->nh_.setParam("/camera/realsense2_camera/color_width", resolution_x);
}

void Ros_VS::get_image_data_convert(const ImageConstPtr& image_color_msg, const ImageConstPtr& image_depth_msg, Mat& color_img, Mat& depth_img)
{
    // rgbת�Ҷ� [0,255]->[1,0]
    cv_bridge::CvImagePtr cv_ptr_color = cv_bridge::toCvCopy(image_color_msg, sensor_msgs::image_encodings::BGR8);
    Mat img_new_rgb = cv_ptr_color->image;
    cvtColor(img_new_rgb, color_img, CV_BGR2GRAY);
    color_img.convertTo(color_img, CV_64FC1);
    color_img = 1 - color_img / 255.0;
    // ���ͼ
    cv_bridge::CvImagePtr cv_ptr_depth = cv_bridge::toCvCopy(image_depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
    depth_img = cv_ptr_depth->image;
    depth_img.convertTo(depth_img, CV_64FC1);
    depth_img = depth_img / 1000.0;
}

Mat Ros_VS::get_camera_pose()
{
    /*****************/
    return Mat::eye(4,4,CV_64FC1);
}

Mat Ros_VS::velocity_camera_to_base(Mat velocity, Mat pose)
{
    Mat R = pose.rowRange(0,3).colRange(0,3);
    Mat p = pose.rowRange(0,3).colRange(3,4);
    double xa = p.at<double>(0,0), ya = p.at<double>(1,0), za = p.at<double>(2,0);
    Mat p_cross = (Mat_<double>(3,3)<< 0.0, -za, ya, za, 0.0, -xa, -ya, xa, 0.0);
    Mat AdT = Mat::zeros(6,6,CV_64FC1);
    R.copyTo(AdT.rowRange(0,3).colRange(0,3));
    R.copyTo(AdT.rowRange(3,6).colRange(3,6));
    AdT.rowRange(0,3).colRange(3,6) = p_cross * R;
    Mat V = AdT * velocity;
    return V;
}