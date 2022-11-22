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


void Ros_VS::get_parameters_VS(int& resolution_x, int& resolution_y, double& lambda, double& epsilon, Mat& image_gray_desired, Mat& image_depth_desired, Mat& image_gray_initial, Mat& camera_intrinsic, Mat& pose_desired, Mat& pose_initial)
{
    // 基本参数
    this->nh_.getParam("resolution_x", resolution_x);
    this->nh_.getParam("resolution_y", resolution_y);
    this->nh_.getParam("lambda", lambda);
    this->nh_.getParam("epsilon", epsilon);
    // 图像参数
    string loaction, name;
    this->nh_.getParam("resource_location", loaction);
    this->nh_.getParam("image_gray_desired_name", name);
    image_gray_desired = imread(loaction + name, IMREAD_GRAYSCALE);
    this->nh_.getParam("image_depth_desired_name", name);
    image_depth_desired = imread(loaction + name);  
    this->nh_.getParam("image_gray_initial_name", name);
    image_gray_initial = imread(loaction + name, IMREAD_GRAYSCALE);  
    // 相机内参
    camera_intrinsic = get_parameter_Matrix("camera_intrinsic", 3, 3);
    // 期望位姿
    pose_desired = get_parameter_Matrix("pose_desired", 4, 4);
    // 期望位姿
    pose_initial = get_parameter_Matrix("pose_initial", 4, 4);
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
    // rgb转灰度 [0,255]->[1,0]
    cv_bridge::CvImagePtr cv_ptr_color = cv_bridge::toCvCopy(image_color_msg, sensor_msgs::image_encodings::BGR8);
    Mat img_new_rgb = cv_ptr_color->image;
    cvtColor(img_new_rgb, color_img, CV_BGR2GRAY);
    color_img.convertTo(color_img, CV_64FC1);
    color_img = 1 - color_img / 255.0;
    // 深度图
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

Mat Ros_VS::get_parameter_Matrix(string str, int row, int col)
{
    Mat Matrix;
    XmlRpc::XmlRpcValue param_yaml;
    this->nh_.getParam(str, param_yaml);
    double data[param_yaml.size()];
    for(int i=0; i<param_yaml.size(); i++) 
    {
        data[i] = param_yaml[i];
    }
    Mat Matrix_temp = Mat(row, col, CV_64FC1, data);
    Matrix_temp.copyTo(Matrix);   
    return Matrix;
}