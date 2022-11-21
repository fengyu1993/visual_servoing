# include "ros_VS.h"


Ros_VS::Ros_VS()
{
    this->flag_success = false;

    this->nh_.getParam("control_rate", this->control_rate_);

    image_color_sub_.subscribe(this->nh_,"/camera/color/image_raw", 1);// bgr8
    image_depth_sub_.subscribe(this->nh_,"/camera/aligned_depth_to_color/image_raw", 1);
    TimeSynchronizer<Image, Image> sync(image_color_sub_, image_depth_sub_, 10);
    sync.registerCallback(boost::bind(&Ros_VS::Callback, this, _1, _2));

    pub_ = this->nh_.advertise<geometry_msgs::Twist>("/cartesian_velocity_node_controller/cartesian_velocity", 5);
}

void Ros_VS::get_parameters(int& resolution_x, int& resolution_y, double& lambda, double& epsilon, Mat& image_gray_desired, Mat& image_depth_desired, Mat& image_gray_initial, Mat& camera_intrinsic, Mat& pose_desired)
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
    XmlRpc::XmlRpcValue param_yaml;
    this->nh_.getParam("camera_intrinsic", param_yaml);
    double intrinsic_temp[9];
    for(int i=0; i<9; i++) 
    {
        intrinsic_temp[i] = param_yaml[i];
    }
    Mat camera_intrinsic_temp = Mat(3, 3, CV_64FC1, intrinsic_temp);
    camera_intrinsic_temp.copyTo(camera_intrinsic);
    // 期望位姿
    this->nh_.getParam("pose_desired", param_yaml);
    double pose_temp[16];
    for(int i=0; i<16; i++) 
    {
        pose_temp[i] = param_yaml[i];
    }     
    Mat pose_desired_temp = Mat(4, 4, CV_64FC1, pose_temp);
    pose_desired_temp.copyTo(pose_desired);
}
