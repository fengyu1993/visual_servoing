
#include "direct_visual_servoing.h"
#include <iostream>
#include <opencv2/opencv.hpp>

int main()
{
    // test Direct_Visual_Servoing class
    Mat camera_intrinsic = (Mat_<double>(3,3) << 1000, 0, 320, 0, 1000, 240, 0, 0, 1);
    Mat img_old, img_new, depth_old, depth_new;
    Mat camera_velocity;

    img_old = imread("/home/cyh/Work/visual_servoing_ws/src/visual_servoing/image_process/resource/Lauren_old_5.jpg", IMREAD_GRAYSCALE);
    img_new = imread("/home/cyh/Work/visual_servoing_ws/src/visual_servoing/image_process/resource/Lauren_new_5.jpg", IMREAD_GRAYSCALE);  
    img_old.convertTo(img_old, CV_64FC1);
    img_new.convertTo(img_new, CV_64FC1);
    img_old = img_old / 255.0;
    img_new = img_new / 255.0;
    depth_old = img_old * 100 + 1;
    depth_new = img_new * 100 + 1;

    if(img_old.empty() || img_new.empty())
    {
        cerr <<"no image" << endl;
        return 0;
    }else
    {
        Direct_Visual_Servoing DVS(img_old.rows, img_old.cols);
        DVS.init_VS(5e-2, 0.1, img_old, depth_old, camera_intrinsic);
        DVS.set_image_depth_current(depth_new);
        DVS.set_image_gray_current(img_new);
        camera_velocity = DVS.get_camera_velocity();

        cout << "camera_velocity = \n" << camera_velocity.t() << endl;
        
    }
    

    return 1;
}
















//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
