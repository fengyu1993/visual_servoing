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
    this->nh_.getParam("itera_num_all", this->itera_num_all_);
    this->nh_.getParam("name_link0", this->name_link0_);
    this->nh_.getParam("name_camera_frame", this->name_camera_frame_);
    this->nh_.getParam("name_effector", this->name_effector_);
    this->joint_angle_initial_VS_ = get_parameter_Matrix("joint_angle_initial_VS", 6, 1);
    this->joint_angle_start_ = get_parameter_Matrix("joint_angle_start", 6, 1);
    this->pub_twist_ = this->nh_.advertise<geometry_msgs::Twist>("/twist_controller/command", 5);
    this->image_polarized_sub_ = this->nh_.subscribe("camera/polarized_Iall_gray", 10, &Ros_ur_DVS::Callback, this);
     
    this->start_DVS = false;
    this->goal.trajectory.joint_names.push_back("shoulder_pan_joint");
    this->goal.trajectory.joint_names.push_back("shoulder_lift_joint");
    this->goal.trajectory.joint_names.push_back("elbow_joint");
    this->goal.trajectory.joint_names.push_back("wrist_1_joint");
    this->goal.trajectory.joint_names.push_back("wrist_2_joint");
    this->goal.trajectory.joint_names.push_back("wrist_3_joint");
    this->R_temp_ = Mat::eye(3, 3, CV_64FC1);
    this->p_temp_= Mat::zeros(3, 1, CV_64FC1);
    this->p_so3_temp_= Mat::zeros(3, 3, CV_64FC1);
    this->effector_twist_ = Mat::zeros(6,1,CV_64FC1);
    this->effector_twist_base_ = Mat::zeros(6,1,CV_64FC1);
    this->T_effector_to_base_ = Mat::eye(4, 4, CV_64FC1);
    this->T_camera_to_effector_ = Mat::eye(4, 4, CV_64FC1);
    this->T_camera_to_base_ = Mat::eye(4, 4, CV_64FC1);
    this->px_temp_ = 0; this->py_temp_ = 0; this->pz_temp_ = 0; 
    this->camera_velocity_ = Mat::zeros(6,1,CV_64FC1);

    this->client = new Client("pos_joint_traj_controller/follow_joint_trajectory", true);  
    ROS_INFO("Waiting for action server to start.");
    this->client->waitForServer();  // 将会一直等待直到动作服务器可用
    ROS_INFO("Server started, sending goal.");
}



void Ros_ur_DVS::Callback(const robot_control_VS::PolarizedImagesConstPtr& gray_msg)
{
    if(this->start_DVS)
    {   
        // 计算视觉伺服的图像
        if(!this->get_visual_servoing_image(gray_msg, this->img_new_)){
            ROS_INFO("get_visual_servoing_image error!!!");
            return;
        }
        // 第一次伺服保存数据
        if(this->DVS->flag_first_)
        {  
            ROS_INFO("first VS!!!");
            double lambda, epsilon;
            Mat img_old, depth_old, camera_intrinsic, pose_desired;
            // 获取参数
            this->get_parameters_DVS(lambda, epsilon, img_old, depth_old, camera_intrinsic, pose_desired);         
            this->DVS->init_VS(lambda, epsilon, img_old, depth_old, this->img_new_, camera_intrinsic, pose_desired);
            this->DVS->flag_first_ = false;
        }
        this->depth_new_ = Mat::ones(this->img_new_.size(), CV_64FC1) * 1.5;
        this->DVS->set_image_depth_current(this->depth_new_);
        this->DVS->set_image_gray_current(this->img_new_); 
        // 获取相机位姿
        this->get_camera_pose(this->T_camera_to_base_); 
        // 计算相机速度
        this->camera_velocity_ = this->DVS->get_camera_velocity(); 
        this->camera_velocity_ = (cv::Mat_<double>(6,1) << 0.00, 0.00, 0.00, 0.04, 0.00, 0.00); 
        // 保存数据  
        this->DVS->save_data(this->T_camera_to_base_);
        // 判断是否成功并做速度转换
        if(this->DVS->is_success() || this->DVS->iteration_num_ > this->itera_num_all_)
        {
            cout << "VS finish" << endl;
            this->flag_success_ = true;
            this->DVS->write_data();  
            this->camera_velocity_ = 0 * this->camera_velocity_;
            this->start_DVS = false;
        }
        else
        {
            this->flag_success_ = false;
         }
       // 发布速度信息
        this->twist_publist(this->camera_velocity_);
    }
}




bool Ros_ur_DVS::get_visual_servoing_image(const robot_control_VS::PolarizedImagesConstPtr& gray_msg, cv::Mat& VS_image)
{
    try {
        cv_bridge::toCvCopy(gray_msg->image_S0, "mono8")->image.convertTo(VS_image, CV_64FC1);
        return true;
    } catch (const cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return false;
    } catch (const std::exception& e) {
        ROS_ERROR("Exception: %s", e.what());
        return false;
    }
}



void Ros_ur_DVS::get_camera_pose(Mat& T_camera_to_base)
{
    tf::StampedTransform transform;
    const int max_attempts = 5;
    const double retry_delay = 0.1;
    for (int attempt = 0; attempt < max_attempts; ++attempt)
    {
        try
        {
            // 尝试获取当前时刻的变换
            this->listener_pose_.lookupTransform(this->name_link0_, this->name_camera_frame_, ros::Time(0), this->transform_temp_);
            get_T(this->transform_temp_, T_camera_to_base);  
        }
        catch (tf::TransformException &ex)
        {
            // 如果获取变换失败，打印错误信息并稍作等待
            ROS_WARN("Failed to get transform, retrying... (%d/%d)", attempt + 1, max_attempts);
            ros::Duration(retry_delay).sleep();
        }
    }
}

Mat Ros_ur_DVS::velocity_camera_to_base(Mat velocity, Mat pose)
{
    Mat R_camera_to_base = pose.rowRange(0,3).colRange(0,3);
    Mat V_effector_to_base = Mat::zeros(6,1,CV_64FC1);
    V_effector_to_base.rowRange(0,3).colRange(0,1) = R_camera_to_base * velocity.rowRange(0,3).colRange(0,1);
    V_effector_to_base.rowRange(3,6).colRange(0,1) = R_camera_to_base * velocity.rowRange(3,6).colRange(0,1);

    return V_effector_to_base;
}

void Ros_ur_DVS::twist_publist(Mat camera_velocity)
{
    get_camera_effector_pose(this->T_effector_to_base_, this->T_camera_to_effector_);
    // 速度转换
    get_effector_twist(camera_velocity, this->T_camera_to_effector_, this->effector_twist_);
    velocity_effector_to_base(this->effector_twist_, this->T_effector_to_base_, this->effector_twist_base_);      
    // 发布速度信息
    const double* vel_ptr = this->effector_twist_base_.ptr<double>(0); 
    this->msg_camera_twist_.linear.x = vel_ptr[0];
    this->msg_camera_twist_.linear.y = vel_ptr[1];
    this->msg_camera_twist_.linear.z = vel_ptr[2];
    this->msg_camera_twist_.angular.x = vel_ptr[3];
    this->msg_camera_twist_.angular.y = vel_ptr[4];
    this->msg_camera_twist_.angular.z = vel_ptr[5];
    this->pub_twist_.publish(msg_camera_twist_);
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



void Ros_ur_DVS::get_parameters_DVS(double& lambda, double& epsilon, Mat& image_gray_desired, Mat& image_depth_desired, Mat& camera_intrinsic, Mat& pose_desired)
{
    // 基本参数
    this->nh_.getParam("lambda", lambda);
    this->nh_.getParam("epsilon", epsilon);
    // 图像参数
    string loaction, name;
    this->nh_.getParam("resource_location", loaction); 
    // 读偏振图
    this->nh_.getParam("image_polar_desired_name", name);
    Mat image_rgb_desired = imread(loaction + name, IMREAD_COLOR);
    rgb_image_operate(image_rgb_desired, image_gray_desired);  
    // 赋值深度图
    double depth_desired;
    this->nh_.getParam("depth_desired", depth_desired);
    image_depth_desired = Mat::ones(image_rgb_desired.size(), CV_64F) * 1.5;
    // 相机内参
    camera_intrinsic = get_parameter_Matrix("camera_intrinsic", 3, 3);
    // 期望位姿
    pose_desired = get_parameter_Matrix("pose_desired", 4, 4);
}



void Ros_ur_DVS::rgb_image_operate(Mat& image_rgb, Mat& image_gray)
{
    Mat temp_gray;
    cvtColor(image_rgb, temp_gray, cv::COLOR_BGR2GRAY);
    temp_gray.convertTo(image_gray, CV_64FC1);
}


void Ros_ur_DVS::get_T(tf::StampedTransform  transform, Mat& T)
{
    tf::Matrix3x3 rotation_matrix = transform.getBasis();
    tf::Vector3 translation_vector = transform.getOrigin();

    T.at<double>(0, 3) = translation_vector[0];
    T.at<double>(1, 3) = translation_vector[1];
    T.at<double>(2, 3) = translation_vector[2];

    T.at<double>(0, 0) = rotation_matrix[0][0]; T.at<double>(0, 1) = rotation_matrix[0][1]; T.at<double>(0, 2) = rotation_matrix[0][2];
    T.at<double>(1, 0) = rotation_matrix[1][0]; T.at<double>(1, 1) = rotation_matrix[1][1]; T.at<double>(1, 2) = rotation_matrix[1][2];
    T.at<double>(2, 0) = rotation_matrix[2][0]; T.at<double>(2, 1) = rotation_matrix[2][1]; T.at<double>(2, 2) = rotation_matrix[2][2];
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


void Ros_ur_DVS::get_camera_effector_pose(Mat& T_effector_to_base, Mat& T_camera_to_effector)
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
            this->listener_pose_.lookupTransform(this->name_link0_, this->name_effector_, ros::Time(0), this->transform_temp_);
            this->get_T(this->transform_temp_, T_effector_to_base);

            this->listener_pose_.lookupTransform(this->name_effector_, this->name_camera_frame_, ros::Time(0), this->transform_temp_);
            this->get_T(this->transform_temp_, T_camera_to_effector);  
        }
        catch (tf::TransformException &ex)
        {
            // 如果获取变换失败，打印错误信息并稍作等待
            ROS_WARN("Failed to get transform, retrying... (%d/%d)", attempt + 1, max_attempts);
            ros::Duration(retry_delay).sleep();
        }
    }
}

void Ros_ur_DVS::get_effector_twist(const Mat& camera_velocity, const Mat& T_camera_to_effector, Mat& effector_twist)
{
    this->R_temp_ = T_camera_to_effector(cv::Rect(0, 0, 3, 3));
    this->p_temp_ = T_camera_to_effector(cv::Rect(3, 0, 1, 3));
    this->px_temp_ = this->p_temp_.at<double>(0);
    this->py_temp_ = this->p_temp_.at<double>(1);
    this->pz_temp_ = this->p_temp_.at<double>(2);

    this->p_so3_temp_.at<double>(0,1) = -this->pz_temp_;  this->p_so3_temp_.at<double>(0,2) = this->py_temp_;
    this->p_so3_temp_.at<double>(1,0) = this->pz_temp_;   this->p_so3_temp_.at<double>(1,2) = -this->px_temp_;
    this->p_so3_temp_.at<double>(2,0) = -this->py_temp_;  this->p_so3_temp_.at<double>(2,1) = this->px_temp_;   

    effector_twist.rowRange(0,3).colRange(0,1) = this->R_temp_ * camera_velocity.rowRange(0,3).colRange(0,1) + 
                        this->p_so3_temp_ * this->R_temp_ * camera_velocity.rowRange(3,6).colRange(0,1);                   
    effector_twist.rowRange(3,6).colRange(0,1) = this->R_temp_ * camera_velocity.rowRange(3,6).colRange(0,1);

}

void Ros_ur_DVS::velocity_effector_to_base(const Mat& velocity, const Mat& effector_to_base, Mat& effector_twist_bast)
{
    effector_twist_bast.rowRange(0,3).colRange(0,1) = effector_to_base.rowRange(0,3).colRange(0,3) * velocity.rowRange(0,3).colRange(0,1);
    effector_twist_bast.rowRange(3,6).colRange(0,1) = effector_to_base.rowRange(0,3).colRange(0,3) * velocity.rowRange(3,6).colRange(0,1);
}