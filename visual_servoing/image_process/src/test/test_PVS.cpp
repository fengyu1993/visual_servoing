
#include "polarimetric_visual_servoing.h"
#include <iostream>
#include <opencv2/opencv.hpp>



// test Polarimetric_Visual_Servoing class
int main()
{
    Mat image_I_0_desired = imread("/home/cyh/Work/visual_servoing_ws/src/visual_servoing/image_process/resource/ball_0_resize.png", IMREAD_UNCHANGED); 
    Mat image_I_45_desired = imread("/home/cyh/Work/visual_servoing_ws/src/visual_servoing/image_process/resource/ball_45_resize.png", IMREAD_UNCHANGED);
    Mat image_I_90_desired = imread("/home/cyh/Work/visual_servoing_ws/src/visual_servoing/image_process/resource/ball_90_resize.png", IMREAD_UNCHANGED);
    Mat image_I_135_desired = imread("/home/cyh/Work/visual_servoing_ws/src/visual_servoing/image_process/resource/ball_135_resize.png", IMREAD_UNCHANGED);
    Mat image_I_0_current = imread("/home/cyh/Work/visual_servoing_ws/src/visual_servoing/image_process/resource/apple_0_resize.png", IMREAD_UNCHANGED);
    Mat image_I_45_current = imread("/home/cyh/Work/visual_servoing_ws/src/visual_servoing/image_process/resource/apple_45_resize.png", IMREAD_UNCHANGED);
    Mat image_I_90_current = imread("/home/cyh/Work/visual_servoing_ws/src/visual_servoing/image_process/resource/apple_90_resize.png", IMREAD_UNCHANGED);
    Mat image_I_135_current = imread("/home/cyh/Work/visual_servoing_ws/src/visual_servoing/image_process/resource/apple_135_resize.png", IMREAD_UNCHANGED);
   
    image_I_0_desired.convertTo(image_I_0_desired, CV_64FC1);
    image_I_45_desired.convertTo(image_I_45_desired, CV_64FC1);
    image_I_90_desired.convertTo(image_I_90_desired, CV_64FC1);
    image_I_135_desired.convertTo(image_I_135_desired, CV_64FC1);
    image_I_0_current.convertTo(image_I_0_current, CV_64FC1);
    image_I_45_current.convertTo(image_I_45_current, CV_64FC1);
    image_I_90_current.convertTo(image_I_90_current, CV_64FC1);
    image_I_135_current.convertTo(image_I_135_current, CV_64FC1);

    Mat image_Z_desired = (image_I_0_desired + image_I_45_desired + image_I_90_desired + image_I_135_desired) / 400.0;
    Mat image_Z_current = (image_I_0_current + image_I_45_current + image_I_90_current + image_I_135_current) / 400.0;

    Mat camera_intrinsic = (Mat_<double>(3,3) << 1000, 0, 128, 0, 1000, 96, 0, 0, 1);

    double eta = 1.5; 
    double phi_pol = 0.0; 
    double k = 2.0;

    Mat pose = (Mat_<double>(6,1) << 1.0, 5.0, 9.0, 0.25, 0.36, 0.5);

    Polarimetric_Visual_Servoing PVS(image_I_0_desired.cols, image_I_0_desired.rows);

    PVS.init_VS(5e-2, 0.1, eta, phi_pol, k, image_I_0_desired, image_I_45_desired, image_I_90_desired, image_I_135_desired, image_Z_desired,
    image_I_0_current, image_I_45_current, image_I_90_current, image_I_135_current, camera_intrinsic, pose);
 
    PVS.get_Phong_model_init();

    PVS.get_polar_data_desired();

    PVS.get_interaction_matrix_desired();

    Mat camera_velocity;   
    for(int i = 0; i < 10; i++)
    {
        PVS.set_image_depth_current(image_Z_current);
        PVS.set_image_gray_current(image_I_0_current, image_I_45_current, image_I_90_current, image_I_135_current);
        PVS.get_polar_data_current();
        camera_velocity = PVS.get_camera_velocity();
        cout << "camera_velocity = \n" << camera_velocity << endl;
        PVS.save_data(pose*i);
    }    

    PVS.write_data();    
    cout << "cyh_end" << endl;

    return 1;
}




  // Mat flag = I > 3;
    // cout << "flag = " << endl << flag << endl;
    // Mat idx;
	// cv::findNonZero(flag, idx);
    // cout << "idx = " << endl << idx << endl;
    
    // for(int i = 0; i < idx.cols * idx.rows; i++)
    // {
    //     I.at<double>(idx.at<Point>(i).y, idx.at<Point>(i).x) = I.at<double>(idx.at<Point>(i).y, idx.at<Point>(i).x) + 0.5;
    // }
    
    // cout << "I = " << endl << I << endl;

    // Mat B = PVS.get_L_n((Mat_<double>(3,1) << 1.0, 5.0, 9.0));
    // cout << "B = " << endl << B << endl;









//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 
    //  Size dsize = Size(320, 240);
    // resize(img_old, img_old, dsize, 0, 0, INTER_AREA);
    // resize(img_new, img_new, dsize, 0, 0, INTER_AREA);
    // imwrite("/home/cyh/Work/visual_servoing_ws/src/visual_servoing/image_process/resource/Lauren_new_320_240.jpg", img_new);
    // imwrite("/home/cyh/Work/visual_servoing_ws/src/visual_servoing/image_process/resource/Lauren_old_320_240.jpg", img_old);

       // Size dsize = Size(512, 512);
        // resize(img_old, img_old, dsize, 0, 0, INTER_AREA);
        // img_old.resize(512, 512);
	    // Mat img_new = img_old.clone();
        // Mat affine_matrix = getRotationMatrix2D(Point2f(img_new.cols / 2, img_new.rows / 2), 90 * -1, 1.0);//𸀙ó𸀈𸀕𸀢𸁁×𷿿𸀐𸀩𸀧ó
	    // warpAffine(img_old, img_new, affine_matrix, img_new.size());

        // imshow("grey", img_old);
        // imshow("grey", img_new);
        // cout << "img_new = " << img_new.rows << ", " << img_new.cols << endl;
        // cout << "img_old = " << img_old.rows << ", " << img_old.cols << endl;
        // imwrite("/home/cyh/Work/visual_servoing_ws/src/visual_servoing/image_process/resource/Lauren_new.jpg", img_new);
        // imwrite("/home/cyh/Work/visual_servoing_ws/src/visual_servoing/image_process/resource/Lauren_old.jpg", img_old);

    // Size dsize = Size(640, 480);
    // resize(img_old, img_old, dsize, 0, 0, INTER_AREA);
    // resize(img_new, img_new, dsize, 0, 0, INTER_AREA);
    // imwrite("/home/cyh/Work/visual_servoing_ws/src/visual_servoing/image_process/resource/Lauren_new_640_480.jpg", img_new);
    // imwrite("/home/cyh/Work/visual_servoing_ws/src/visual_servoing/image_process/resource/Lauren_old_640_480.jpg", img_old);
