
#include "direct_microscopic_visual_servoing.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <chrono> 

void get_image_gradient_x(const Mat& image, Mat& I_x);
void get_image_gradient_x_old(const Mat& image, Mat& I_x);
void get_image_gradient_y(const Mat& image, Mat& I_x);
void get_image_gradient_y_old(const Mat& image, Mat& I_x);
Mat skewSymmetric(const Mat& v);

int main()
{
    Mat Toc = (cv::Mat_<double>(4, 4) << 
        1, 2, 3, 1,
        4, 5, 6, 2,
        7, 8, 9, 5,
        0, 0, 0, 1);
    Mat Ad_Toc = Mat::zeros(6, 6, CV_64F);
    Toc(cv::Rect(0, 0, 3, 3)).copyTo(Ad_Toc(cv::Rect(0, 0, 3, 3)));
	Toc(cv::Rect(0, 0, 3, 3)).copyTo(Ad_Toc(cv::Rect(3, 3, 3, 3)));
	cv::Mat t_x_R = skewSymmetric(Toc(cv::Rect(3, 0, 1, 3))) * Toc(cv::Rect(0, 0, 3, 3));
    // cv::gemm(skewSymmetric(Toc(cv::Rect(3, 0, 1, 3))), Toc(cv::Rect(0, 0, 3, 3)), 1, cv::noArray(), 0, t_x_R); 
 	
    t_x_R.copyTo(Ad_Toc(cv::Rect(3, 0, 3, 3))); 

    cout << "Toc" << endl << Toc << endl;
    cout << "Ad_Toc" << endl << Ad_Toc << endl;

    Mat D = (cv::Mat_<double>(3, 3) << 
        1, 2, 3,
        4, 5, 6,
        7, 8, 9);
    Mat F = (cv::Mat_<double>(3, 3) << 
        1, 2, 3,
        4, 5, 6,
        7, 8, 9);

    cout << "D * F" << endl << D * F << endl;
    
    int rows = 6;
    int cols = 5;

    // 创建一个 rows x cols 的矩阵（int 类型）
    cv::Mat mat(rows, cols, CV_64FC1);

    // 使用 parallel_for_ 和 Lambda 表达式进行并行赋值
    cv::parallel_for_(cv::Range(0, mat.rows), [&](const cv::Range& range) {
        for (int r = range.start; r < range.end; ++r) {
            double* rowPtr = mat.ptr<double>(r);
            for (int c = 0; c < mat.cols; ++c) {
                rowPtr[c] = r;  // 每个元素赋值为行号
            }
        }
    });

    int totalElements = mat.total();
    cv::Mat B = cv::Mat::zeros(totalElements, 3, CV_64FC1);  // 3列，其中第2列为待赋值

    // 获取 B 的第2列（列索引为1）
    cv::Mat B_col2 = B.col(1);

    // 扁平化 A 为一行（totalElements 个元素）
    // cv::Mat mat_flat = mat.reshape(0, 1);  // 1 行，totalElements 列

    // 使用 parallel_for_ 并行赋值
    cv::parallel_for_(cv::Range(0, totalElements), [&](const cv::Range& range) {
        for (int i = range.start; i < range.end; ++i) {
            B_col2.at<double>(i) = mat.at<double>(i);
        }
    });

    cout << "mat" <<  endl << mat << endl;

    std::cout << "B" << endl << B << std::endl;
    cout << "s" << endl << mat.reshape(0, mat.rows*mat.cols) << endl;


    std::cout << "结果矩阵：\n" << mat << std::endl;

    cv::parallel_for_(cv::Range(0, mat.rows), [&](const cv::Range& range) {
        for (int r = range.start; r < range.end; ++r) {
            double* rowPtr = mat.ptr<double>(r);
            for (int c = 0; c < cols; ++c) {
                rowPtr[c] = c;
            }
        }
    });

    std::cout << "结果矩阵：\n" << mat << std::endl;


    // cv::Mat I(2000, 2000, CV_64FC1);
    // cv::randn(I, cv::Scalar(0), cv::Scalar(1));

    // Mat I_x_matlab;
    // auto start = std::chrono::high_resolution_clock::now();
    // get_image_gradient_x_old(I, I_x_matlab);
    // auto end = std::chrono::high_resolution_clock::now();
    // auto duration_matlab_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    // cv::Mat I_x_mat;
    // start = std::chrono::high_resolution_clock::now();
    // get_image_gradient_x(I, I_x_mat);
    // end = std::chrono::high_resolution_clock::now();
    // auto duration_mat_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    // Mat I_y_matlab;
    // start = std::chrono::high_resolution_clock::now();
    // get_image_gradient_y_old(I, I_y_matlab);
    // end = std::chrono::high_resolution_clock::now();
    // duration_matlab_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    // cv::Mat I_y_mat;
    // start = std::chrono::high_resolution_clock::now();
    // get_image_gradient_y(I, I_y_mat);
    // end = std::chrono::high_resolution_clock::now();
    // duration_mat_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();


    // cout << "I = " << endl << I << endl;
    // cout << "error = " << endl << I_y_mat - I_y_matlab << endl;
    // cout << "duration_matlab_ms = " << duration_matlab_ms << endl;
    // cout << "duration_mat_ms = " << duration_mat_ms << endl;


    return 1;
}

Mat skewSymmetric(const Mat& v) {
    CV_Assert(v.rows == 3 && v.cols == 1);

    double vx = v.at<double>(0);
    double vy = v.at<double>(1);
    double vz = v.at<double>(2);

    cv::Mat skew = cv::Mat::zeros(3, 3, CV_64FC1);

    skew.at<double>(0, 0) = 0;     skew.at<double>(0, 1) = -vz;   skew.at<double>(0, 2) = vy;
    skew.at<double>(1, 0) = vz;    skew.at<double>(1, 1) = 0;     skew.at<double>(1, 2) = -vx;
    skew.at<double>(2, 0) = -vy;   skew.at<double>(2, 1) = vx;    skew.at<double>(2, 2) = 0;

    return skew;
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
