#include "ros_PVS.h"
#include <opencv2/opencv.hpp>

Ros_PVS::Ros_PVS()
{
    // 设置分辨率
    int resolution_x, resolution_y;
    get_parameters_resolution(resolution_x, resolution_y);
    // 初始化
    this->PVS = new Polarimetric_Visual_Servoing(resolution_x, resolution_y);
    this->flag_success_ = false;
    this->nh_.getParam("control_rate", this->control_rate_);
    this->joint_angle_initial_ = get_parameter_Matrix("joint_angle_initial", 7, 1);
    this->move_group_interface_ = new moveit::planning_interface::MoveGroupInterface("ur");  
    this->pub_camera_twist_ = this->nh_.advertise<geometry_msgs::Twist>("/cartesian_velocity_node_controller/cartesian_velocity", 5);
    this->start_PVS = false;
}

void Ros_PVS::Callback(const ImageConstPtr& image_polar_msg, const ImageConstPtr& image_depth_msg)
{
    if(this->start_PVS)
    {      
        // 数据转换
        Mat depth_new, polar_O_new, polar_A_new, polar_Phi_new;
        get_image_data_convert(image_polar_msg, image_depth_msg, polar_O_new,  polar_A_new, polar_Phi_new, depth_new);
        // 获取相机位姿
        Mat camera_pose = get_camera_pose(); 
        // 计算相机速度并保存数据
        this->PVS->set_image_depth_current(depth_new);
        this->PVS->set_image_polar_current(polar_O_new, polar_A_new, polar_Phi_new); 
        // 准备
        if(this->PVS->flag_first_)
        {  
            double lambda, epsilon, eta, phi_pol, k;
            Mat polar_O_old, polar_A_old, polar_Phi_old, depth_old, camera_intrinsic, pose_desired;
            // 获取参数
            get_parameters_PVS(lambda, epsilon, eta, phi_pol, k, polar_O_old, polar_A_old, polar_Phi_old, depth_old, camera_intrinsic, pose_desired);
            this->PVS->init_VS(lambda, epsilon, eta, phi_pol, k, polar_O_old, polar_A_old, polar_Phi_old, depth_old, polar_O_new, polar_A_new, polar_Phi_new, camera_intrinsic, pose_desired);
            this->PVS->flag_first_ = false;
        }

        Mat camera_velocity = this->PVS->get_camera_velocity(); 

        this->PVS->save_data(camera_pose);
        // ROS_INFO("cyh");  
        // cout << "img_old = \n" <<  img_old.rowRange(0,10).colRange(0,5) << endl;
        // cout << "img_new = \n" <<  img_new.rowRange(0,10).colRange(0,5) << endl;
        // cout << "depth_old = \n" <<  depth_old.rowRange(0,10).colRange(0,5) << endl;
        // cout << "depth_new = \n" <<  depth_new.rowRange(0,10).colRange(0,5) << endl;
        // cout << "camera_velocity = \n" << camera_velocity << endl;
        cout << "iteration_num = " << this->PVS->iteration_num_ << endl;
        cout << "error = " << ((double)*(this->PVS->data_pvs.error_feature_.end<double>() - 1)) << endl;

        // 判断是否成功并做速度转换
        if(this->PVS->is_success() || this->PVS->iteration_num_ > 2000)
        {
            this->flag_success_ = true;
            this->PVS->write_data();  
            this->camera_velocity_base_ = 0 * camera_velocity;
            this->start_PVS = false;
        }
        else
        {
            this->flag_success_ = false;
            // 速度转换
            this->camera_velocity_base_ = velocity_camera_to_base(camera_velocity, camera_pose);
        }

       // 发布速度信息
        geometry_msgs::Twist camera_Twist;
        camera_Twist.linear.x = this->camera_velocity_base_.at<double>(0,0);
        camera_Twist.linear.y = this->camera_velocity_base_.at<double>(1,0);
        camera_Twist.linear.z = this->camera_velocity_base_.at<double>(2,0);
        camera_Twist.angular.x = this->camera_velocity_base_.at<double>(3,0);
        camera_Twist.angular.y = this->camera_velocity_base_.at<double>(4,0);
        camera_Twist.angular.z = this->camera_velocity_base_.at<double>(5,0);
        this->pub_camera_twist_.publish(camera_Twist);
    }
}

void Ros_PVS::get_parameters_resolution(int& resolution_x, int& resolution_y)
{
    this->nh_.getParam("resolution_x", resolution_x);
    this->nh_.getParam("resolution_y", resolution_y);
}

Mat Ros_PVS::get_parameter_Matrix(string str, int row, int col)
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


void Ros_PVS::initialize_time_sync()
{
    image_polar_sub_.subscribe(this->nh_,"/camera/polarized_image", 1);
    image_depth_sub_.subscribe(this->nh_,"/camera/depth_image", 1);
    this->sync_ = new TimeSynchronizer<Image, Image>(image_polar_sub_, image_depth_sub_, 1);
    this->sync_->registerCallback(boost::bind(&Ros_PVS::Callback, this, _1, _2));
}



void Ros_PVS::get_parameters_PVS(double& lambda, double& epsilon, double& eta, double& phi_pol, double& k, 
                Mat& polar_O_desired, Mat& polar_A_desired, Mat& polar_Phi_desired, 
                Mat& image_depth_desired, Mat& camera_intrinsic, Mat& pose_desired)
{
    // 基本参数
    this->nh_.getParam("lambda", lambda);
    this->nh_.getParam("epsilon", epsilon);
    this->nh_.getParam("eta", eta);
    this->nh_.getParam("phi_pol", phi_pol);
    this->nh_.getParam("k", k);
    this->nh_.getParam("name_link0", this->name_link0_);
    this->nh_.getParam("name_camera_frame", this->name_camera_frame_);
    // 图像参数
    string loaction, name_polar, name_depth;
    this->nh_.getParam("resource_location", loaction);
    this->nh_.getParam("image_polar_desired_name", name_polar);
    this->nh_.getParam("image_depth_desired_name", name_depth);
    // 读偏振图
    Mat image_polar_desired = imread(loaction + name_polar, IMREAD_COLOR);
    polar_image_operate(image_polar_desired, polar_O_desired, polar_A_desired, polar_Phi_desired);
    // 读深度图
    Mat image_depth_desired_temp = imread(loaction + name_depth, IMREAD_UNCHANGED); 
    image_depth_desired = depth_image_operate(image_depth_desired_temp);   
    // 相机内参
    camera_intrinsic = get_parameter_Matrix("camera_intrinsic", 3, 3);
    // 期望位姿
    pose_desired = get_parameter_Matrix("pose_desired", 4, 4);
}

void Ros_PVS::get_image_data_convert(const ImageConstPtr& image_polar_msg, const ImageConstPtr& image_depth_msg, Mat& polar_O_new, Mat& polar_A_new, Mat& polar_Phi_new, Mat& depth_img)
{
    // 偏振数据
    cv_bridge::CvImagePtr cv_ptr_polar = cv_bridge::toCvCopy(image_polar_msg, sensor_msgs::image_encodings::BGR8);
    Mat img_polar = cv_ptr_polar->image;
    polar_image_operate(img_polar, polar_O_new, polar_A_new, polar_Phi_new);
    // 深度图
    cv_bridge::CvImagePtr cv_ptr_depth = cv_bridge::toCvCopy(image_depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
    Mat depth_img_temp = cv_ptr_depth->image;
    depth_img = depth_image_operate(depth_img_temp);
}

void Ros_PVS::polar_image_operate(Mat& img_polar, Mat& polar_O_new, Mat& polar_A_new, Mat& polar_Phi_new)
{
    Mat polar_gray;
    vector<Mat> channels;

    img_polar.convertTo(polar_gray, CV_64FC1);
    split(polar_gray, channels); 
    channels.at(0).copyTo(polar_O_new);
    polar_A_new = channels.at(1).mul(polar_O_new + 0.001) / 255.0;
    polar_Phi_new = 2.0 * (channels.at(2) * CV_PI / 180.0 - CV_PI);
}

Mat Ros_PVS::depth_image_operate(Mat& image_depth)
{
    Mat image_depth_return;
    image_depth.convertTo(image_depth_return, CV_64FC1);
    image_depth_return = image_depth_return / 1000.0;
    return image_depth_return;
}

Mat Ros_PVS::velocity_camera_to_base(Mat velocity, Mat pose)
{
    Mat R_camera_to_base = pose.rowRange(0,3).colRange(0,3);
    Mat V_effector_to_base = Mat::zeros(6,1,CV_64FC1);
    V_effector_to_base.rowRange(0,3).colRange(0,1) = R_camera_to_base * velocity.rowRange(0,3).colRange(0,1);
    V_effector_to_base.rowRange(3,6).colRange(0,1) = R_camera_to_base * velocity.rowRange(3,6).colRange(0,1);

    return V_effector_to_base;
}


Mat Ros_PVS::get_T(tf::StampedTransform  transform)
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

Mat Ros_PVS::Quaternion2Matrix (Mat q)
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


void Ros_PVS::robot_move_to_target_joint_angle(std::vector<double> joint_group_positions_target)
{
    this->move_group_interface_->setJointValueTarget(joint_group_positions_target);
    this->move_group_interface_->setMaxAccelerationScalingFactor(0.05);
    this->move_group_interface_->setMaxVelocityScalingFactor(0.05);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (this->move_group_interface_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(success)
    {
        std::cout << "Press Enter to move the robot..." << std::endl;
        std::cin.ignore();
        this->move_group_interface_->move();
        std::cout << "Move finish" << std::endl;
    }
    else
    {
        std::cout << "moveit joint plan fail ! ! !" << std::endl;
    }
}

Mat Ros_PVS::get_camera_pose()
{
    tf::StampedTransform transform;
    this->listener_camera_pose_.waitForTransform(this->name_link0_, this->name_camera_frame_, ros::Time(0), ros::Duration(3.0));
    this->listener_camera_pose_.lookupTransform(this->name_link0_, this->name_camera_frame_, ros::Time(0), transform);
    Mat T = get_T(transform);
    return T;
}
