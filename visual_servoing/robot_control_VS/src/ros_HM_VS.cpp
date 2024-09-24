# include "ros_HM_VS.h"

Ros_HM_VS::Ros_HM_VS() : Ros_VS()
{ 
    int resolution_x, resolution_y;
    int order_min, order_max;
    double delta_epsilon, lambda_order;
    // ��ȡ�ֱ���
    get_parameters_resolution(resolution_x, resolution_y);
    // ��ȡDOM����
    get_parameters_DOM(order_min, order_max, delta_epsilon, lambda_order);
    // ��ʼ��
    this->HM_VS = new Hahn_Moments_VS(order_min, order_max, delta_epsilon, lambda_order, resolution_x, resolution_y);
}

void Ros_HM_VS::Callback(const ImageConstPtr& image_color_msg, const ImageConstPtr& image_depth_msg)
{
    if(this->start_VS)
    {     
        // ����ת��
        Mat depth_new, img_new;
        get_image_data_convert(image_color_msg, image_depth_msg, img_new, depth_new);
        // ��ȡ���λ��
        Mat camera_pose = get_camera_pose();
        // ��������ٶȲ���������
        this->HM_VS->set_image_depth_current(depth_new);
        this->HM_VS->set_image_gray_current(img_new);
        if(this->HM_VS->flag_first_)
        {
            // ׼��
            double lambda, epsilon;
            Mat img_old, depth_old, camera_intrinsic, pose_desired;
            // ��ȡ����
            get_parameters_VS(lambda, epsilon, img_old, depth_old, camera_intrinsic, pose_desired);
            this->HM_VS->init_VS(lambda, epsilon, img_old, depth_old, img_new, camera_intrinsic, pose_desired);
            this->HM_VS->flag_first_ = false;
        }
 
        Mat camera_velocity = this->HM_VS->get_camera_velocity();
        this->HM_VS->save_data(camera_pose);
         
        ROS_INFO("iteration_num = %i", this->HM_VS->iteration_num_);
        ROS_INFO("error = %f", ((double)*(this->HM_VS->data_dom.error_pixel_ave_.end<double>() - 1)));
 
        // �ж��Ƿ�ɹ������ٶ�ת��
        if(this->HM_VS->is_success() || this->HM_VS->iteration_num_ > 220)
        {
            this->flag_success_ = true;
            this->HM_VS->write_data();  
            this->camera_velocity_base_ = 0 * camera_velocity;
            this->start_VS = false;
        }
        else
        {
            this->flag_success_ = false;
            // �ٶ�ת��
            this->camera_velocity_base_ = velocity_camera_to_base(camera_velocity, camera_pose);
        }
        // �����ٶ���Ϣ
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













