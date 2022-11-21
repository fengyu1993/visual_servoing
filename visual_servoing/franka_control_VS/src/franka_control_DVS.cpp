
#include "direct_visual_servoing.h"
#include <iostream>
#include <opencv2/opencv.hpp>

int main()
{
    // test Direct_Visual_Servoing class
    Mat camera_intrinsic = (Mat_<double>(3,3) << 1000, 0, 320, 0, 1000, 240, 0, 0, 1);
    Mat img_old, img_new, depth_old, depth_new;
    Mat camera_velocity;

    // img_old = imread("/home/cyh/Work/visual_servoing_ws/src/visual_servoing/image_process/resource/Lauren_old_5.jpg", IMREAD_GRAYSCALE);
    // img_new = imread("/home/cyh/Work/visual_servoing_ws/src/visual_servoing/image_process/resource/Lauren_new_5.jpg", IMREAD_GRAYSCALE);  
    img_old = imread("/home/cyh/Work/visual_servoing_ws/src/visual_servoing/image_process/resource/Lauren_old_640_480.jpg", IMREAD_GRAYSCALE);
    img_new = imread("/home/cyh/Work/visual_servoing_ws/src/visual_servoing/image_process/resource/Lauren_new_640_480.jpg", IMREAD_GRAYSCALE);    
    // img_old = imread("/home/cyh/Work/visual_servoing_ws/src/visual_servoing/image_process/resource/Lauren_old_200_150.jpg", IMREAD_GRAYSCALE);
    // img_new = imread("/home/cyh/Work/visual_servoing_ws/src/visual_servoing/image_process/resource/Lauren_new_200_150.jpg", IMREAD_GRAYSCALE);    
    
    img_old.convertTo(img_old, CV_64FC1);
    img_new.convertTo(img_new, CV_64FC1);

    img_old = img_old / 255.0;
    img_new = img_new / 255.0;
    depth_old = img_old * 100 + 1;
    depth_new = img_new * 100 + 1;

    Mat pose = (Mat_<double>(7,1) << 1.0, 5.0, 9.0, 0.25, 0.36, 0.5, 0.8);

    if(img_old.empty() || img_new.empty())
    {
        cerr <<"no image" << endl;
        return 0;
    }else
    {
        Direct_Visual_Servoing DVS(img_old.cols, img_old.rows);
        DVS.init_VS(5e-2, 0.1, img_old, depth_old, img_new, camera_intrinsic, pose);
        for(int i = 0; i < 10; i++)
        {
            DVS.set_image_depth_current(depth_new);
            DVS.set_image_gray_current(img_new);
            camera_velocity = DVS.get_camera_velocity();
            cout << "camera_velocity = \n" << camera_velocity.t() << endl;
            DVS.save_data(pose*i);
        }    
        // DVS.write_data();    
    }
    

    return 1;
}















