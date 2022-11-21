# include "ros_DVS.h"

Ros_DVS::Ros_DVS() : Ros_VS()
{
    // 准备
    double lambda, epsilon; 
    int resolution_x, resolution_y;
    Mat img_old, depth_old, img_init, camera_intrinsic, pose_desired;

    // 获取参数
    get_parameters(resolution_x, resolution_y, lambda, epsilon, img_old, depth_old, img_init, camera_intrinsic, pose_desired);

    // 初始化
    Direct_Visual_Servoing DVS_temp(resolution_x, resolution_y);
    this->DVS = DVS_temp;
    this->DVS.init_VS(lambda, epsilon, img_old, depth_old, img_init, camera_intrinsic, pose_desired);

}


void Ros_DVS::Callback(const ImageConstPtr& image_color_msg, const ImageConstPtr& image_depth_msg)
{
    ROS_INFO("cyh");
    // Mat depth_new, img_new, pose;
    // /**************************/
    // // depth_new = ;
    // // img_new = ;
    // // pose = ;
    // /**************************/
    // this->DVS.set_image_depth_current(depth_new);
    // this->DVS.set_image_gray_current(img_new);
    // Mat camera_velocity = this->DVS.get_camera_velocity();
    // this->DVS.save_data(pose);
    // if(this->DVS.is_success())
    // {
    //     this->flag_success = true;
    //     this->DVS.write_data();  
    // }
    // else
    // {
    //     this->flag_success = false;
    // }
}




    // cout << "resolution_x = " << resolution_x << endl;
    // cout << "resolution_y = " << resolution_y << endl;
    // cout << "lambda = " << lambda << endl;
    // cout << "epsilon = " << epsilon << endl;
    // cout << "img_old: row = " << img_old.rows << ", col = " << img_old.cols << endl;
    // cout << "depth_old: row = " << depth_old.rows << ", col = " << depth_old.cols << endl;
    // cout << "img_init: row = " << img_init.rows << ", col = " << img_init.cols << endl;
    // cout << "camera_intrinsic = " << camera_intrinsic << endl;
    // cout << "pose_desired = " << pose_desired << endl;