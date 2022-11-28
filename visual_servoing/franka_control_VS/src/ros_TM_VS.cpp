# include "ros_TM_VS.h"

Ros_TM_VS::Ros_TM_VS() : Ros_VS()
{ 
    int resolution_x, resolution_y;
    int order_min, order_max;
    double delta_epsilon, lambda_order;
    // 获取分辨率
    get_parameters_resolution(resolution_x, resolution_y);
    // 获取DOM参数
    get_parameters_DOM(order_min, order_max, delta_epsilon, lambda_order);
    // 初始化
    this->TM_VS = new Techebichef_Moments_VS(order_min, order_max, delta_epsilon, lambda_order, resolution_x, resolution_y);
}

void Ros_TM_VS::Callback(const ImageConstPtr& image_color_msg, const ImageConstPtr& image_depth_msg)
{
    if(this->start_VS)
    {
        ROS_INFO("cyh");        
        // 数据转换
        Mat depth_new, img_new;
        get_image_data_convert(image_color_msg, image_depth_msg, img_new, depth_new);
        // 获取相机位姿
        Mat camera_pose = get_camera_pose();
        // 计算相机速度并保存数据
        this->TM_VS->set_image_depth_current(depth_new);
        this->TM_VS->set_image_gray_current(img_new);
        if(this->TM_VS->flag_first)
        {
            // 准备
            double lambda, epsilon;
            Mat img_old, depth_old, camera_intrinsic, pose_desired;
            // 获取参数
            get_parameters_VS(lambda, epsilon, img_old, depth_old, camera_intrinsic, pose_desired);
            this->TM_VS->init_VS(lambda, epsilon, img_old, depth_old, img_new, camera_intrinsic, pose_desired);
            this->TM_VS->flag_first = false;
        }

        Mat camera_velocity = this->TM_VS->get_camera_velocity();
        cout << "camera_velocity = " << camera_velocity.t() << endl;
        
        this->TM_VS->save_data(camera_pose);
        // 判断是否成功并做速度转换
        if(this->TM_VS->is_success())
        {
            this->flag_success_ = true;
            this->TM_VS->write_data();  
            camera_velocity = 0 * camera_velocity;
            this->start_VS = false;
        }
        else
        {
            this->flag_success_ = false;
            // 速度转换
            camera_velocity = velocity_camera_to_base(camera_velocity, camera_pose);
        }
        // 发布速度信息
        geometry_msgs::Twist camera_Twist;
        // camera_Twist.linear.x = camera_velocity.at<double>(0,0);
        // camera_Twist.linear.y = camera_velocity.at<double>(1,0);
        // camera_Twist.linear.z = camera_velocity.at<double>(2,0);
        // camera_Twist.angular.x = camera_velocity.at<double>(3,0);
        // camera_Twist.angular.y = camera_velocity.at<double>(4,0);
        // camera_Twist.angular.z = camera_velocity.at<double>(5,0);
        camera_Twist.linear.x = 0.05;
        camera_Twist.linear.y = 0;
        camera_Twist.linear.z = 0;
        camera_Twist.angular.x = 0;
        camera_Twist.angular.y = 0;
        camera_Twist.angular.z = 0;
        this->pub_camera_twist_.publish(camera_Twist);
    }
}













