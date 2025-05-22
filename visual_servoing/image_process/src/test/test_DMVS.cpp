
#include "direct_microscopic_visual_servoing.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <chrono> 

void get_image_gradient_x(const Mat& image, Mat& I_x);
void get_image_gradient_x_old(const Mat& image, Mat& I_x);
void get_image_gradient_y(const Mat& image, Mat& I_x);
void get_image_gradient_y_old(const Mat& image, Mat& I_x);

int main()
{
    cv::Mat I(2000, 2000, CV_64FC1);
    cv::randn(I, cv::Scalar(0), cv::Scalar(1));

    Mat I_x_matlab;
    auto start = std::chrono::high_resolution_clock::now();
    get_image_gradient_x_old(I, I_x_matlab);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration_matlab_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    cv::Mat I_x_mat;
    start = std::chrono::high_resolution_clock::now();
    get_image_gradient_x(I, I_x_mat);
    end = std::chrono::high_resolution_clock::now();
    auto duration_mat_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    Mat I_y_matlab;
    start = std::chrono::high_resolution_clock::now();
    get_image_gradient_y_old(I, I_y_matlab);
    end = std::chrono::high_resolution_clock::now();
    duration_matlab_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    cv::Mat I_y_mat;
    start = std::chrono::high_resolution_clock::now();
    get_image_gradient_y(I, I_y_mat);
    end = std::chrono::high_resolution_clock::now();
    duration_mat_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();


    // cout << "I = " << endl << I << endl;
    // cout << "error = " << endl << I_y_mat - I_y_matlab << endl;
    cout << "duration_matlab_ms = " << duration_matlab_ms << endl;
    cout << "duration_mat_ms = " << duration_mat_ms << endl;

    // // test Direct_Visual_Servoing class
    // Mat camera_intrinsic = (Mat_<double>(3,3) << 1000, 0, 320, 0, 1000, 240, 0, 0, 1);
    // Mat img_old, img_new, depth_old, depth_new;
    // Mat camera_velocity;

    // img_old = imread("/home/cyh/Work/visual_servoing_ws/src/visual_servoing/image_process/resource/Lauren_old_640_480.jpg", IMREAD_GRAYSCALE);
    // img_new = imread("/home/cyh/Work/visual_servoing_ws/src/visual_servoing/image_process/resource/Lauren_new_640_480.jpg", IMREAD_GRAYSCALE);    
    
    // img_old.convertTo(img_old, CV_64FC1);
    // img_new.convertTo(img_new, CV_64FC1);


    // Mat pose = (Mat_<double>(7,1) << 1.0, 5.0, 9.0, 0.25, 0.36, 0.5, 0.8);

    // if(img_old.empty() || img_new.empty())
    // {
    //     cerr <<"no image" << endl;
    //     return 0;
    // }else
    // {
    //     Direct_Visual_Servoing DVS(img_old.cols, img_old.rows);
    //     DVS.init_VS(5e-2, 0.1, img_old, depth_old, img_new, camera_intrinsic, pose);
    //     for(int i = 0; i < 10; i++)
    //     {
    //         DVS.set_image_depth_current(depth_new);
    //         DVS.set_image_gray_current(img_new);
    //         camera_velocity = DVS.get_camera_velocity();
    //         cout << "camera_velocity = \n" << camera_velocity.t() << endl;
    //         DVS.save_data(pose*i);
    //     }    
    //     DVS.write_data();    
    // }
    

    return 1;
}

void get_image_gradient_x(const Mat& image, Mat& I_x)
{
    I_x = cv::Mat::zeros(image.rows, image.cols, CV_64FC1);
    Mat div_col = Mat::ones(image.rows, image.cols, CV_64FC1) * 2;
    div_col.col(0).setTo(cv::Scalar(1.0));
    div_col.col(image.cols-1).setTo(cv::Scalar(1.0));
    // 中间部分：(i+1)列 - (i-1)列
    cv::subtract(image.colRange(2, image.cols), image.colRange(0, image.cols - 2), I_x.colRange(1, image.cols - 1)); // 中间部分直接相减
    cv::subtract(image.col(1), image.col(0), I_x.col(0));
    cv::subtract(image.col(image.cols - 1), image.col(image.cols - 2), I_x.col(image.cols - 1));
    cv::divide(I_x, div_col, I_x);
}

void get_image_gradient_y(const Mat& image, Mat& I_y)
{
    I_y = cv::Mat::zeros(image.rows, image.cols, CV_64FC1);
    Mat div_row = Mat::ones(image.rows, image.cols, CV_64FC1) * 2;
    div_row.row(0).setTo(cv::Scalar(1.0));
    div_row.row(image.rows-1).setTo(cv::Scalar(1.0));
    // 中间部分：(i+1)行 - (i-1)行
    cv::subtract(image.rowRange(2, image.rows), image.rowRange(0, image.rows - 2), I_y.rowRange(1, image.rows - 1)); // 中间部分直接相减
    cv::subtract(image.row(1), image.row(0), I_y.row(0));
    cv::subtract(image.row(image.rows - 1), image.row(image.rows - 2), I_y.row(image.rows - 1));
    cv::divide(I_y, div_row, I_y);
}

void get_image_gradient_x_old(const Mat& image, Mat& I_x)
{
    I_x = cv::Mat::zeros(image.rows, image.cols, CV_64FC1);
    int up, down;
    for(int i = 0; i < image.cols; i++)
    {
        up = i+1;
        down = i-1;
        if (up > image.cols-1) 
            up = image.cols-1;
        if (down < 0) 
            down = 0;
        Mat temp = (image.col(up) - image.col(down))/ (up - down);
        temp.copyTo(I_x.col(i)); 
    }
}


void get_image_gradient_y_old(const Mat& image, Mat& I_y)
{
    I_y = Mat::zeros(image.rows, image.cols, CV_64FC1);
    int up, down;
    for(int i = 0; i < image.rows; i++)
    {
        up = i+1;
        down = i-1;
        if (up > image.rows-1) 
            up = image.rows-1;
        if (down < 0) 
            down = 0;
        Mat temp = (image.row(up) - image.row(down)) / (up - down); 
        temp.copyTo(I_y.row(i)); 
    }
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
