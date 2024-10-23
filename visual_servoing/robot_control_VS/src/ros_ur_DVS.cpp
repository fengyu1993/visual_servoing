#include "ros_ur_DVS.h"

Ros_ur_DVS::Ros_ur_DVS()
{
    // 设置分辨率
    int resolution_x, resolution_y;
    get_parameters_resolution(resolution_x, resolution_y);
    // 初始化
    this->DVS = new Direct_Visual_Servoing(resolution_x, resolution_y);
    this->flag_success_ = false;
    this->nh_.getParam("control_rate", this->control_rate_);
    this->nh_.getParam("name_link0", this->name_link0_);
    this->nh_.getParam("name_camera_frame", this->name_camera_frame_);
    this->nh_.getParam("name_effector", this->name_effector_);
    this->joint_angle_initial_VS_ = get_parameter_Matrix("joint_angle_initial_VS", 6, 1);
    this->pub_twist_ = this->nh_.advertise<geometry_msgs::Twist>("/twist_controller/command", 5);
    this->start_DVS = false;
    this->goal.trajectory.joint_names.push_back("shoulder_pan_joint");
    this->goal.trajectory.joint_names.push_back("shoulder_lift_joint");
    this->goal.trajectory.joint_names.push_back("elbow_joint");
    this->goal.trajectory.joint_names.push_back("wrist_1_joint");
    this->goal.trajectory.joint_names.push_back("wrist_2_joint");
    this->goal.trajectory.joint_names.push_back("wrist_3_joint");
    this->client = new Client("pos_joint_traj_controller/follow_joint_trajectory", true);  
    ROS_INFO("Waiting for action server to start.");
    this->client->waitForServer();  // 将会一直等待直到动作服务器可用
    ROS_INFO("Server started, sending goal.");
}

void Ros_ur_DVS::Callback(const ImageConstPtr& image_polar_msg, const ImageConstPtr& image_depth_msg)
{
    if(this->start_DVS)
    {      
        // 数据转换
        Mat depth_new, img_new;
        get_image_data_convert(image_polar_msg, image_depth_msg, img_new, depth_new);
        // 获取相机位姿
        Mat camera_pose = get_camera_pose(); 
        // 计算相机速度并保存数据
        this->DVS->set_image_depth_current(depth_new);
        this->DVS->set_image_gray_current(img_new); 
        // 准备
        if(this->DVS->flag_first_)
        {  
            double lambda, epsilon;
            Mat img_old, depth_old, camera_intrinsic, pose_desired;
            // 获取参数
            get_parameters_DVS(lambda, epsilon, img_old, depth_old, camera_intrinsic, pose_desired);
            this->DVS->init_VS(lambda, epsilon, img_old, depth_old, img_new, camera_intrinsic, pose_desired);
            this->DVS->flag_first_ = false;
        }

        Mat camera_velocity = this->DVS->get_camera_velocity(); 

        this->DVS->save_data(camera_pose);
        // ROS_INFO("cyh");  
        // cout << "img_old = \n" <<  img_old.rowRange(0,10).colRange(0,5) << endl;
        // cout << "img_new = \n" <<  img_new.rowRange(0,10).colRange(0,5) << endl;
        // cout << "depth_old = \n" <<  depth_old.rowRange(0,10).colRange(0,5) << endl;
        // cout << "depth_new = \n" <<  depth_new.rowRange(0,10).colRange(0,5) << endl;
        // cout << "camera_velocity = \n" << camera_velocity << endl;
        cout << "iteration_num = " << this->DVS->iteration_num_ << endl;
        cout << "error = " << ((double)*(this->DVS->data_vs.error_feature_.end<double>() - 1)) << endl;

        // 判断是否成功并做速度转换
        if(this->DVS->is_success() || this->DVS->iteration_num_ > 2000)
        {
            this->flag_success_ = true;
            this->DVS->write_data();  
            camera_velocity = 0 * camera_velocity;
            this->start_DVS = false;
        }
        else
        {
            this->flag_success_ = false;
            // 速度转换
            
        }

       // 发布速度信息
        twist_publist(camera_velocity);
    }
}


Mat Ros_ur_DVS::get_camera_pose()
{
    tf::StampedTransform transform;
    const int max_attempts = 5;
    const double retry_delay = 0.1;
    for (int attempt = 0; attempt < max_attempts; ++attempt)
    {
        try
        {
            tf::StampedTransform transform; 
            // 尝试获取当前时刻的变换
            this->listener_pose_.lookupTransform(this->name_link0_, this->name_camera_frame_, ros::Time(0), transform);
            // this->listener_pose_.lookupTransform("base", "camera_polar_frame", ros::Time(0), transform);
            Mat camera_to_base = get_T(transform);  
            return camera_to_base;
        }
        catch (tf::TransformException &ex)
        {
            // 如果获取变换失败，打印错误信息并稍作等待
            ROS_WARN("Failed to get transform, retrying... (%d/%d)", attempt + 1, max_attempts);
            ros::Duration(retry_delay).sleep();
        }
    }
    return Mat::eye(4,4,CV_64FC1);
}

void Ros_ur_DVS::twist_publist(Mat camera_velocity)
{
    Mat T_effector_to_base = Mat::eye(4, 4, CV_64F);
    Mat T_camera_to_effector = Mat::eye(4, 4, CV_64F);
    get_camera_effector_pose(T_effector_to_base, T_camera_to_effector);
    // 速度转换
    Mat effector_twist = get_effector_velocity(camera_velocity, T_camera_to_effector);
    this->effector_velocity_base_ = velocity_effector_to_base(effector_twist, T_effector_to_base);
    // 发布速度信息
    geometry_msgs::Twist effector_Twist;
    effector_Twist.linear.x = this->effector_velocity_base_.at<double>(0,0);
    effector_Twist.linear.y = this->effector_velocity_base_.at<double>(1,0);
    effector_Twist.linear.z = this->effector_velocity_base_.at<double>(2,0);
    effector_Twist.angular.x = this->effector_velocity_base_.at<double>(3,0);
    effector_Twist.angular.y = this->effector_velocity_base_.at<double>(4,0);
    effector_Twist.angular.z = this->effector_velocity_base_.at<double>(5,0);
    this->pub_twist_.publish(effector_Twist);
}

void Ros_ur_DVS::get_parameters_resolution(int& resolution_x, int& resolution_y)
{
    this->nh_.getParam("resolution_x", resolution_x);
    this->nh_.getParam("resolution_y", resolution_y);
}

Mat Ros_ur_DVS::get_parameter_Matrix(string str, int row, int col)
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


void Ros_ur_DVS::initialize_time_sync()
{
    image_color_sub_.subscribe(this->nh_,"/camera/polarized_image", 1);
    image_depth_sub_.subscribe(this->nh_,"/camera/depth_image", 1);
    this->sync_ = new TimeSynchronizer<Image, Image>(image_color_sub_, image_depth_sub_, 1);
    this->sync_->registerCallback(boost::bind(&Ros_ur_DVS::Callback, this, _1, _2));
}



void Ros_ur_DVS::get_parameters_DVS(double& lambda, double& epsilon, Mat& image_gray_desired, Mat& image_depth_desired, Mat& camera_intrinsic, Mat& pose_desired)
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

void Ros_ur_DVS::get_image_data_convert(const ImageConstPtr& image_polar_msg, const ImageConstPtr& image_depth_msg, Mat& gray_img, Mat& depth_img)
{
    // rgb转灰度 [0,255]->[1,0]
    cv_bridge::CvImagePtr cv_ptr_polar = cv_bridge::toCvCopy(image_polar_msg, sensor_msgs::image_encodings::BGR8);
    Mat img_new_polar = cv_ptr_polar->image;
    // gray_img = rgb_image_operate(img_new_rgb); % realsense camera
    vector<Mat> channels;
    split(img_new_polar, channels); // 将图像分割成多个通道
    gray_img = channels[0];
    // 深度图
    cv_bridge::CvImagePtr cv_ptr_depth = cv_bridge::toCvCopy(image_depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
    Mat depth_img_temp = cv_ptr_depth->image;
    depth_img = depth_image_operate(depth_img_temp);
}

Mat Ros_ur_DVS::rgb_image_operate(Mat& image_rgb)
{
    Mat image_gray;
    cvtColor(image_rgb, image_gray, CV_BGR2GRAY);
    image_gray.convertTo(image_gray, CV_64FC1);
    image_gray = image_gray/255.0; 
    return image_gray;
}

Mat Ros_ur_DVS::depth_image_operate(Mat& image_depth)
{
    Mat image_depth_return;
    image_depth.convertTo(image_depth_return, CV_64FC1);
    image_depth_return = image_depth_return / 1000.0;
    return image_depth_return;
}

Mat Ros_ur_DVS::velocity_effector_to_base(Mat velocity, Mat effector_to_base)
{
    Mat R_effector_to_base = effector_to_base.rowRange(0,3).colRange(0,3);
    Mat V_effector_to_base = Mat::zeros(6,1,CV_64FC1);
    V_effector_to_base.rowRange(0,3).colRange(0,1) = R_effector_to_base * velocity.rowRange(0,3).colRange(0,1);
    V_effector_to_base.rowRange(3,6).colRange(0,1) = R_effector_to_base * velocity.rowRange(3,6).colRange(0,1);

    return V_effector_to_base;
}


Mat Ros_ur_DVS::get_T(tf::StampedTransform  transform)
{
    tf::Matrix3x3 rotation_matrix = transform.getBasis();
    tf::Vector3 translation_vector = transform.getOrigin();

    Mat T = Mat::eye(4,4,CV_64FC1);
    Mat p = (Mat_<double>(3,1) << translation_vector[0], translation_vector[1], translation_vector[2]);
    p.copyTo(T.rowRange(0,3).colRange(3,4));
    Mat R = (Mat_<double>(3,3) << rotation_matrix[0][0], rotation_matrix[0][1], rotation_matrix[0][2], 
                                  rotation_matrix[1][0], rotation_matrix[1][1], rotation_matrix[1][2], 
                                  rotation_matrix[2][0], rotation_matrix[2][1], rotation_matrix[2][2]);
    R.copyTo(T.rowRange(0,3).colRange(0,3));

    return T; 
}


void Ros_ur_DVS::robot_move_to_target_joint_angle(std::vector<double> joint_group_positions_target)
{
   // 添加轨迹点
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.resize(6); 
    for (int i=0; i<6; i++)
        point.positions[i] = joint_group_positions_target[i];
    point.time_from_start = ros::Duration(5.0);
    this->goal.trajectory.points.clear();
    this->goal.trajectory.points.push_back(point);
    this->goal.trajectory.header.stamp = ros::Time::now(); 
    this->client->sendGoal(goal);
    // 等待结果
    bool finished_before_timeout = this->client->waitForResult(ros::Duration(6.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = this->client->getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
    }
    else
        ROS_INFO("Action did not finish before the time out.");
}

void Ros_ur_DVS::get_camera_effector_pose(Mat& effector_to_base, Mat& camera_to_effector)
{
    tf::StampedTransform transform;
    const int max_attempts = 5;
    const double retry_delay = 0.1;
    for (int attempt = 0; attempt < max_attempts; ++attempt)
    {
        try
        {
            tf::StampedTransform transform; //"base" "tool0_controller" "camera_polar_frame"
            // 尝试获取当前时刻的变换
            this->listener_pose_.lookupTransform(this->name_link0_, this->name_effector_, ros::Time(0), transform);
            effector_to_base = get_T(transform);

            this->listener_pose_.lookupTransform(this->name_effector_, this->name_camera_frame_, ros::Time(0), transform);
            camera_to_effector = get_T(transform);  
        }
        catch (tf::TransformException &ex)
        {
            // 如果获取变换失败，打印错误信息并稍作等待
            ROS_WARN("Failed to get transform, retrying... (%d/%d)", attempt + 1, max_attempts);
            ros::Duration(retry_delay).sleep();
        }
    }
}

Mat Ros_ur_DVS::get_effector_velocity(Mat camera_velocity, Mat camera_to_effector)
{
    Mat AdT = Mat::zeros(6,6,CV_64FC1);
    Mat R = camera_to_effector.rowRange(0,3).colRange(0,3);
    Mat p = camera_to_effector.rowRange(0,3).colRange(3,4);
    Mat effector_velocity_base= Mat::zeros(6,1,CV_64FC1);
    Mat p_so3 = (Mat_<double>(3,3) << 0, -p.at<double>(2,0), p.at<double>(1,0), 
                                p.at<double>(2,0), 0, -p.at<double>(0,0),
                                 -p.at<double>(1,0), p.at<double>(0,0), 0);
    effector_velocity_base.rowRange(0,3).colRange(0,1) = R * camera_velocity.rowRange(0,3).colRange(0,1) + 
                        p_so3 * R * camera_velocity.rowRange(3,6).colRange(0,1);                   
    effector_velocity_base.rowRange(3,6).colRange(0,1) = R * camera_velocity.rowRange(3,6).colRange(0,1);

    return  effector_velocity_base;
}
