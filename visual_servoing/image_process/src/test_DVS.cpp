#include "discrete_orthogonal_moment.h"
#include "direct_visual_servoing.h"
#include <iostream>
#include <opencv2/opencv.hpp>

int main()
{
    // test Direct_Visual_Servoing class
    // Direct_Visual_Servoing DVS(5e-2, 0.1);

    // test reshape function
    float a[8] = { 1,2,3,4,5,6,7,8 };
    cv::Mat m = cv::Mat(2, 4, CV_32FC1, a);
    std::cout << "m = "<< std::endl << m << std::endl;
    cv::Mat v = m.reshape(0, m.rows*m.cols);
    std::cout << "v = "<< std::endl << v << std::endl; // different from matlab


    return 1;
}