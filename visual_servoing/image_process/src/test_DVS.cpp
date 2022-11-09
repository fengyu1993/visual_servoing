#include "discrete_orthogonal_moment.h"
#include "direct_visual_servoing.h"
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;

int main()
{
    // test Direct_Visual_Servoing class
    Mat camera_intrinsic = (Mat_<double>(3,3) << 1000, 0, 320, 0, 1000, 240, 0, 0, 1);
    Mat img_old, img_new, depth_old, depth_new;
    Mat camera_velocity;

    img_old = imread("/home/cyh/Work/visual_servoing_ws/src/visual_servoing/image_process/resource/Lauren_old.jpg", IMREAD_GRAYSCALE);
    img_new = imread("/home/cyh/Work/visual_servoing_ws/src/visual_servoing/image_process/resource/Lauren_new.jpg", IMREAD_GRAYSCALE);
    depth_old = img_old.clone();
    depth_new = img_new.clone();

    if(img_old.empty() || img_new.empty())
    {
        cerr <<"no image" << endl;
        return 0;
    }else
    {
        Direct_Visual_Servoing DVS(5e-2, 0.1, camera_intrinsic, img_old.rows, img_old.cols);
        DVS.set_image_depth_current(depth_new);
        DVS.set_image_gray_current(img_new);
        DVS.set_image_depth_desired(depth_old);
        DVS.set_image_gray_desired(img_old);
        camera_velocity = DVS.get_camera_velocity();
    }
    

    return 1;
}
















//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
       // Size dsize = Size(512, 512);
        // resize(img_old, img_old, dsize, 0, 0, INTER_AREA);
        // img_old.resize(512, 512);
	    // Mat img_new = img_old.clone();
        // Mat affine_matrix = getRotationMatrix2D(Point2f(img_new.cols / 2, img_new.rows / 2), 90 * -1, 1.0);//�����ת����
	    // warpAffine(img_old, img_new, affine_matrix, img_new.size());

        // imshow("grey", img_old);
        // imshow("grey", img_new);
        // cout << "img_new = " << img_new.rows << ", " << img_new.cols << endl;
        // cout << "img_old = " << img_old.rows << ", " << img_old.cols << endl;
        // imwrite("/home/cyh/Work/visual_servoing_ws/src/visual_servoing/image_process/resource/Lauren_new.jpg", img_new);
        // imwrite("/home/cyh/Work/visual_servoing_ws/src/visual_servoing/image_process/resource/Lauren_old.jpg", img_old);
