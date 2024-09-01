
#include "polarimetric_visual_servoing.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <unsupported/Eigen/CXX11/Tensor>

// function [V, n_desired, n_current, S] = get_Phong_model(row, col, camera_intrinsic)
//     V = ones(row, col, 3); % 需乘以内参矩阵，再归一化
//     for i = 1 : row
//         for j = 1 : col
//             V(i, j, :) = (camera_intrinsic \ [j; i; 1]);
//             V(i, j, :) = V(i, j, :) ./ norm(reshape(V(i, j, :), 1,[]));
//         end
//     end
//     n_desired = ones(row, col, 3); 
//     for i = 1 : col
//         for j = 1 : row
//             n_desired(i, j, :) = [0; 0; -1];
//         end
//     end
//     n_current = n_desired;
//     S = ones(row, col, 3); 
//     for i = 1 : col
//         for j = 1 : row
//             S(i, j, :) = [1; 0; -3] / norm([1; 0; -3]);
//         end
//     end
//     S = n_current;
// end


void get_Phong_model(int row, int col, Mat camera_intrinsic, Mat& V, Mat& n_desired, Mat& n_current, Mat& S)
{

}



// test Polarimetric_Visual_Servoing class
int main()
{
    Mat image_I_0_desired_ = imread("/home/cyh/Work/visual_servoing_ws/src/visual_servoing/image_process/resource/ball_0_resize.png", IMREAD_GRAYSCALE); 
    Mat image_I_45_desired_ = imread("/home/cyh/Work/visual_servoing_ws/src/visual_servoing/image_process/resource/ball_45_resize.png", IMREAD_GRAYSCALE);
    Mat image_I_90_desired_ = imread("/home/cyh/Work/visual_servoing_ws/src/visual_servoing/image_process/resource/ball_90_resize.png", IMREAD_GRAYSCALE);
    Mat image_I_135_desired_ = imread("/home/cyh/Work/visual_servoing_ws/src/visual_servoing/image_process/resource/ball_135_resize.png", IMREAD_GRAYSCALE);
    Mat image_I_0_current_ = imread("/home/cyh/Work/visual_servoing_ws/src/visual_servoing/image_process/resource/apple_0_resize.png", IMREAD_GRAYSCALE);
    Mat image_I_45_current_ = imread("/home/cyh/Work/visual_servoing_ws/src/visual_servoing/image_process/resource/apple_45_resize.png", IMREAD_GRAYSCALE);
    Mat image_I_90_current_ = imread("/home/cyh/Work/visual_servoing_ws/src/visual_servoing/image_process/resource/apple_90_resize.png", IMREAD_GRAYSCALE);
    Mat image_I_135_current_ = imread("/home/cyh/Work/visual_servoing_ws/src/visual_servoing/image_process/resource/apple_135_resize.png", IMREAD_GRAYSCALE);
   
    image_I_0_desired_.convertTo(image_I_0_desired_, CV_64FC1);
    image_I_45_desired_.convertTo(image_I_45_desired_, CV_64FC1);
    image_I_90_desired_.convertTo(image_I_90_desired_, CV_64FC1);
    image_I_135_desired_.convertTo(image_I_135_desired_, CV_64FC1);
    image_I_0_current_.convertTo(image_I_0_current_, CV_64FC1);
    image_I_45_current_.convertTo(image_I_45_current_, CV_64FC1);
    image_I_90_current_.convertTo(image_I_90_current_, CV_64FC1);
    image_I_135_current_.convertTo(image_I_135_current_, CV_64FC1);

    Mat image_Z_desired = (image_I_0_desired_ + image_I_45_desired_ + image_I_90_desired_ + image_I_135_desired_) / 400;
    Mat image_Z_current = (image_I_0_current_ + image_I_45_current_ + image_I_90_current_ + image_I_135_current_) / 400;

    Mat camera_intrinsic = (Mat_<double>(3,3) << 1000, 0, 128, 0, 1000, 96, 0, 0, 1);

    double eta = 1.5; 
    double phi_pol = 0.0; 
    double lambda = 0.5;
    double k = 2.0;
    int size[3] = {image_I_0_desired_.rows, image_I_0_desired_.cols, 3};

    Mat V(3, size, CV_8UC1, cv::Scalar(0));
    cv::Mat mat3D(3, size, CV_32FC1, cv::Scalar(0));
    // , n_desired, n_current, S;

    // get_Phong_model(image_I_0_desired_.rows, image_I_0_desired_.cols, camera_intrinsic, V, n_desired, n_current, S);


    // Polarimetric_Visual_Servoing PVS(image_I_0_desired_.cols, image_I_0_desired_.rows);

        // DVS.init_VS(5e-2, 0.1, img_old, depth_old, img_new, camera_intrinsic, pose);
        // for(int i = 0; i < 10; i++)
        // {
        //     DVS.set_image_depth_current(depth_new);
        //     DVS.set_image_gray_current(img_new);
        //     camera_velocity = DVS.get_camera_velocity();
        //     cout << "camera_velocity = \n" << camera_velocity.t() << endl;
        //     DVS.save_data(pose*i);
        // }    
        // DVS.write_data();    
    cout << "cyh" << endl;

    return 1;
}
















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
        // Mat affine_matrix = getRotationMatrix2D(Point2f(img_new.cols / 2, img_new.rows / 2), 90 * -1, 1.0);//ÇóµÃÐý×ª¾ØÕó
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
