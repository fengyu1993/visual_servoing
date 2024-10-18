#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main()
{
    // read images
    // Mat polar_offset = imread("/home/cyh/Work/visual_servoing_ws/src/visual_servoing/robot_control_VS/param/polar_img_horizon.png", cv::IMREAD_GRAYSCALE);
    // Mat gray_image = imread("/home/cyh/Work/visual_servoing_ws/src/visual_servoing/robot_control_VS/param/gray_img_horizon.png", cv::IMREAD_GRAYSCALE);
    Mat polar_image = imread("/home/cyh/Work/visual_servoing_ws/src/visual_servoing/robot_control_VS/param/polar_img_vertical.png", cv::IMREAD_GRAYSCALE);
    Mat gray_image = imread("/home/cyh/Work/visual_servoing_ws/src/visual_servoing/robot_control_VS/param/gray_img_vertical.png", cv::IMREAD_GRAYSCALE);
    // resize
    Mat gray_image_resize_temp = gray_image.colRange(170, gray_image.cols-80);
    Mat polar_image_resize_temp = polar_image.rowRange(20, polar_image.rows-60);
    cv::Size targetSize(200, 150); 
    cv::Mat gray_image_resize, polar_image_resize;
    cv::resize(gray_image_resize_temp, gray_image_resize, targetSize, 0, 0, cv::INTER_LINEAR);
	cv::resize(polar_image_resize_temp, polar_image_resize, targetSize, 0, 0, cv::INTER_LINEAR);


    // show images
    namedWindow("polar", cv::WINDOW_AUTOSIZE);
    namedWindow("gray", cv::WINDOW_AUTOSIZE);
    imshow("polar", polar_image_resize);
    imshow("gray", gray_image_resize);
    //
    waitKey(0);
    destroyAllWindows();
    //
    return 0;
}