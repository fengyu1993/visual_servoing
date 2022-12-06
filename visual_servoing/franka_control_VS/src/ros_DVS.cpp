# include "ros_DVS.h"

Ros_DVS::Ros_DVS() : Ros_VS()
{ 
    int resolution_x, resolution_y;
    // 设置分辨率
    get_parameters_resolution(resolution_x, resolution_y);
    // 初始化
    this->DVS = new Direct_Visual_Servoing(resolution_x, resolution_y);
}

void Ros_DVS::Callback(const ImageConstPtr& image_color_msg, const ImageConstPtr& image_depth_msg)
{
    if(this->start_VS)
    {      
        // 数据转换
        Mat depth_new, img_new;
        get_image_data_convert(image_color_msg, image_depth_msg, img_new, depth_new);
        // 获取相机位姿
        Mat camera_pose = get_camera_pose(); 
        // 计算相机速度并保存数据
        this->DVS->set_image_depth_current(depth_new);
        this->DVS->set_image_gray_current(img_new); 
        // 准备
        if(this->DVS->flag_first)
        {  
            double lambda, epsilon;
            Mat img_old, depth_old, camera_intrinsic, pose_desired;
            // 获取参数
            get_parameters_VS(lambda, epsilon, img_old, depth_old, camera_intrinsic, pose_desired);
            this->DVS->init_VS(lambda, epsilon, img_old, depth_old, img_new, camera_intrinsic, pose_desired);
            this->DVS->flag_first = false;
        }

        Mat camera_velocity = this->DVS->get_camera_velocity(); 

        this->DVS->save_data(camera_pose);
        // ROS_INFO("cyh");  
        // cout << "img_old = \n" <<  img_old.rowRange(0,10).colRange(0,5) << endl;
        // cout << "img_new = \n" <<  img_new.rowRange(0,10).colRange(0,5) << endl;
        // cout << "depth_old = \n" <<  depth_old.rowRange(0,10).colRange(0,5) << endl;
        // cout << "depth_new = \n" <<  depth_new.rowRange(0,10).colRange(0,5) << endl;
        // cout << "camera_velocity = \n" << camera_velocity << endl;
        cout << "iteration_num = " << this->DVS->iteration_num << endl;
        cout << "error = " << ((double)*(this->DVS->data_vs.error_feature_.end<double>() - 1)) << endl;

        // 判断是否成功并做速度转换
        bool flag = this->DVS->is_success();
        if(this->DVS->iteration_num > 2000)
            flag = true;
        if(flag)
        {
            this->flag_success_ = true;
            this->DVS->write_data();  
            this->camera_velocity_base_ = 0 * camera_velocity;
            this->start_VS = false;
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















    // cout << "desired_color: width = " << this->DVS->image_gray_desired_.cols << 
    //         ", heigh = " << this->DVS->image_gray_desired_.rows << endl;
    // cout << "desired_depth: width = " << this->DVS->image_depth_desired_.cols << 
    //         ", heigh = " << this->DVS->image_depth_desired_.rows << endl;
    // cout << "current_color: width = " << this->DVS->image_gray_current_.cols << 
    //         ", heigh = " << this->DVS->image_gray_current_.rows << endl;
    // cout << "current_depth: width = " << this->DVS->image_depth_current_.cols << 
    //         ", heigh = " << this->DVS->image_depth_current_.rows << endl;

        // // 速度转换
        // camera_pose  = (Mat_<double>(4,4) << 1, 0, 0, 1, 0, 0, -1, 2, 0, 1, 0, 3, 0, 0, 0, 1);
        // camera_velocity = (Mat_<double>(6,1) << 4, 5, 6, 1, 2, 3);
        // cout << "camera_pose" << camera_pose << endl;
        // cout << "camera_velocity" << camera_velocity.t() << endl;
        // camera_velocity = velocity_camera_to_base(camera_velocity, camera_pose);
        // cout << "camera_velocity" << camera_velocity.t() << endl;

    // cout << "img_new = " << img_new.colRange(0,5).rowRange(0,5) << endl;
    // cout << "depth_new = " << depth_new.colRange(0,5).rowRange(0,5) << endl;
    // imshow("Gray", img_new);
    // imshow("Depth", depth_new);
    // waitKey(1);

    // cout << "color_width = " << image_color_msg->width << endl;
    // cout << "color_height = " << image_color_msg->height << endl;
    // cout << "depth_width = " << image_depth_msg->width << endl;
    // cout << "depth_height = " << image_depth_msg->height << endl;

    // cout << "resolution_x = " << resolution_x << endl;
    // cout << "resolution_y = " << resolution_y << endl;
    // cout << "lambda = " << lambda << endl;
    // cout << "epsilon = " << epsilon << endl;
    // cout << "img_old: row = " << img_old.rows << ", col = " << img_old.cols << endl;
    // cout << "depth_old: row = " << depth_old.rows << ", col = " << depth_old.cols << endl;
    // cout << "img_init: row = " << img_init.rows << ", col = " << img_init.cols << endl;
    // cout << "camera_intrinsic = " << camera_intrinsic << endl;
    // cout << "pose_desired = " << pose_desired << endl;
