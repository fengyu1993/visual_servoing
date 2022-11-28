# include "ros_VS.h"


Ros_VS::Ros_VS()
{
   this->flag_success_ = false;

    this->nh_.getParam("control_rate", this->control_rate_);
    this->joint_angle_initial_ = get_parameter_Matrix("joint_angle_initial", 7, 1);

    this->move_group_interface_ = new moveit::planning_interface::MoveGroupInterface("panda_arm");

    this->pub_camera_twist_ = this->nh_.advertise<geometry_msgs::Twist>("/cartesian_velocity_node_controller/cartesian_velocity", 5);

    this->start_VS = true;
}

void Ros_VS::initialize_time_sync()
{
    image_color_sub_.subscribe(this->nh_,"/camera/color/image_raw", 1);
    image_depth_sub_.subscribe(this->nh_,"/camera/aligned_depth_to_color/image_raw", 1);
    this->sync_ = new TimeSynchronizer<Image, Image>(image_color_sub_, image_depth_sub_, 1);
    this->sync_->registerCallback(boost::bind(&Ros_VS::Callback, this, _1, _2));
}

void Ros_VS::get_parameters_resolution(int& resolution_x, int& resolution_y)
{
    this->nh_.getParam("resolution_x", resolution_x);
    this->nh_.getParam("resolution_y", resolution_y);
}

void Ros_VS::get_parameters_VS(double& lambda, double& epsilon, Mat& image_gray_desired, Mat& image_depth_desired, Mat& camera_intrinsic, Mat& pose_desired)
{
    // 基本参数
    this->nh_.getParam("lambda", lambda);
    this->nh_.getParam("epsilon", epsilon);
    // 图像参数
    string loaction, name;
    this->nh_.getParam("resource_location", loaction);
    // 读彩色图
    this->nh_.getParam("image_rgb_desired_name", name);
    Mat image_rgb_desired = imread(loaction + name, IMREAD_COLOR);
    image_gray_desired = rgb_image_operate(image_rgb_desired);
    // 读深度图
    this->nh_.getParam("image_depth_desired_name", name);
    Mat image_depth_desired_temp = imread(loaction + name, IMREAD_UNCHANGED); 
    image_depth_desired = depth_image_operate(image_depth_desired_temp);   
    // 相机内参
    camera_intrinsic = get_parameter_Matrix("camera_intrinsic", 3, 3);
    // 期望位姿
    pose_desired = get_parameter_Matrix("pose_desired", 4, 4);
}


void Ros_VS::set_resolution_parameters(int resolution_x, int resolution_y)
{
    this->nh_.setParam("/camera/realsense2_camera/depth_height", resolution_y);
    this->nh_.setParam("/camera/realsense2_camera/depth_width", resolution_x);
    this->nh_.setParam("/camera/realsense2_camera/color_height", resolution_y);
    this->nh_.setParam("/camera/realsense2_camera/color_width", resolution_x);
}

void Ros_VS::get_image_data_convert(const ImageConstPtr& image_color_msg, const ImageConstPtr& image_depth_msg, Mat& gray_img, Mat& depth_img)
{
    // rgb转灰度 [0,255]->[1,0]
    cv_bridge::CvImagePtr cv_ptr_color = cv_bridge::toCvCopy(image_color_msg, sensor_msgs::image_encodings::BGR8);
    Mat img_new_rgb = cv_ptr_color->image;
    gray_img = rgb_image_operate(img_new_rgb);
    // 深度图
    cv_bridge::CvImagePtr cv_ptr_depth = cv_bridge::toCvCopy(image_depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
    Mat depth_img_temp = cv_ptr_depth->image;
    depth_img = depth_image_operate(depth_img_temp);
}

Mat Ros_VS::get_camera_pose()
{
    tf::StampedTransform transform;
    this->listener_camera_pose_.waitForTransform("panda_link0", "camera_link", ros::Time(0), ros::Duration(3.0));
    this->listener_camera_pose_.lookupTransform("panda_link0", "camera_link", ros::Time(0), transform);
    Mat T = get_T(transform);
    return T;
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

Mat Ros_VS::rgb_image_operate(Mat& image_rgb)
{
    Mat image_gray;
    cvtColor(image_rgb, image_gray, CV_BGR2GRAY);
    image_gray.convertTo(image_gray, CV_64FC1);
    image_gray = 1 - image_gray/255.0; 
    return image_gray;
}

Mat Ros_VS::depth_image_operate(Mat& image_depth)
{
    Mat image_depth_return;
    image_depth.convertTo(image_depth_return, CV_64FC1);
    image_depth_return = image_depth_return / 1000.0;
    return image_depth_return;
}

Mat Ros_VS::get_T(tf::StampedTransform  transform)
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

Mat Ros_VS::Quaternion2Matrix (Mat q)
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


void Ros_VS::franka_move_to_target_joint_angle(std::vector<double> joint_group_positions_target)
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





    // imshow("desired_color", image_rgb_desired);
    // imshow("desired_gray", image_gray_desired);
    // imshow("initial_color", image_rgb_initial);
    // imshow("initial_gray", image_gray_initial);
    // imshow("desired_depth", image_depth_desired);
    // waitKey();