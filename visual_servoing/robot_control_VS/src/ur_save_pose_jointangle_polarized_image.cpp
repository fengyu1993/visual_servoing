#include <iostream>
#include <chrono>
#include <mutex>
#include <memory>
#include <iomanip>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <opencv2/opencv.hpp>
#include <robot_control_VS/PolarizedImages.h>

typedef message_filters::sync_policies::ApproximateTime<robot_control_VS::PolarizedImages, 
        robot_control_VS::PolarizedImages> SyncPolicy;

using namespace std;
using namespace cv;

const string file_name = "DVS_Polarized/";

class PolarizedCameraNode
{
private:
    ros::NodeHandle nh_;
    
    message_filters::Subscriber<robot_control_VS::PolarizedImages> image_polar_color_sub_;
    message_filters::Subscriber<robot_control_VS::PolarizedImages> image_polar_gray_sub_;
    
    

    tf::TransformListener tf_listener_;

    std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    Mat img_polar_color_0, img_polar_color_45, img_polar_color_90, img_polar_color_135, img_polar_color_S0;
    Mat img_polar_gray_0, img_polar_gray_45, img_polar_gray_90, img_polar_gray_135, img_polar_gray_S0;

    Mat S0_color_display_, S0_gray_display_, S0_color_save_; 

    Mat joint_group_positions_;
    Mat T_base_camera_;

    std::mutex image_mutex_;

    const string save_path_ = "/home/cyh/Work/visual_servoing_ws/src/visual_servoing/robot_control_VS/param/";

public:
    PolarizedCameraNode(){
        int resolution_x, resolution_y;
        nh_.param("resolution_x", resolution_x, 1224);
        nh_.param("resolution_y", resolution_y, 1024);

        image_polar_color_sub_.subscribe(nh_, "camera/polarized_Iall_color", 1);
        image_polar_gray_sub_.subscribe(nh_, "camera/polarized_Iall_gray", 1);

        sync_.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), image_polar_color_sub_, image_polar_gray_sub_));
        sync_->registerCallback(boost::bind(&PolarizedCameraNode::imageCallback, this, _1, _2));

        joint_group_positions_ = Mat::zeros(1, 6, CV_64FC1);

        cv::namedWindow("Polar_color", cv::WINDOW_AUTOSIZE);
        cv::namedWindow("Polar_gray", cv::WINDOW_AUTOSIZE);

        ROS_INFO("PolarizedCameraNode initialized successfully.");
    }

    void writeData()
    {
        if (T_base_camera_.empty()) {
            ROS_WARN("No camera pose data to save!");
            return;
        }

        string time_str = getDateTimeString();
        string filename = save_path_ + file_name + time_str + "_pose_data.yaml";
        
        cv::FileStorage fs(filename, cv::FileStorage::WRITE);
        fs << "joint_angle" << joint_group_positions_;
        fs << "camera_pose" << T_base_camera_;
        fs.release();
        
        ROS_INFO("Data saved to: %s", filename.c_str());
    }

    void writeImage()
    {
        std::lock_guard<std::mutex> lock(image_mutex_);
        if (S0_color_save_.empty() || img_polar_color_0.empty()) {
            ROS_WARN("No images to save!");
            return;
        }

        string dir = save_path_ + file_name;
        imwrite(dir + "image_polar_init.png", S0_color_save_);
        imwrite(dir + "image_polar_color_0.png", img_polar_color_0);
        imwrite(dir + "image_polar_color_45.png", img_polar_color_45);
        imwrite(dir + "image_polar_color_90.png", img_polar_color_90);
        imwrite(dir + "image_polar_color_135.png", img_polar_color_135);
        
        ROS_INFO("Images saved to DVS_Polarized directory.");
    }

    void saveJointPositions()
    {
        sensor_msgs::JointStateConstPtr joint_msg = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", nh_, ros::Duration(1.0));
        if (joint_msg != nullptr && joint_msg->position.size() >= 6) {
                std::vector<std::string> target_joint_names = {
                        "shoulder_pan_joint",
                        "shoulder_lift_joint",
                        "elbow_joint",
                        "wrist_1_joint",
                        "wrist_2_joint",
                        "wrist_3_joint"
                    };
                    for (int i = 0; i < 6; ++i) {
                        auto it = std::find(joint_msg->name.begin(), joint_msg->name.end(), target_joint_names[i]);
                        if (it != joint_msg->name.end()) {
                            int index = std::distance(joint_msg->name.begin(), it);
                            joint_group_positions_.at<double>(0, i) = joint_msg->position[index];
                        } else {
                            ROS_ERROR_ONCE("No found joint angle: %s !!!", target_joint_names[i].c_str());
                        }
                    }
                    std::cout << "joint_group_positions: \n" << joint_group_positions_ * 180.0 / M_PI << std::endl;
                } else {
                    ROS_WARN("!!! check /joint_states topic !!!");
                }
    }

    Mat getTransformMatrix(const tf::StampedTransform& transform)
    {
        Mat T = Mat::eye(4, 4, CV_64FC1);
        
        tf::Matrix3x3 R_tf = transform.getBasis();
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                T.at<double>(i, j) = R_tf[i][j];
            }
        }

        tf::Vector3 t_tf = transform.getOrigin();
        T.at<double>(0, 3) = t_tf.getX();
        T.at<double>(1, 3) = t_tf.getY();
        T.at<double>(2, 3) = t_tf.getZ();

        return T;
    }

    void saveCameraPose()
    {
        tf::StampedTransform transform;
        try {
            tf_listener_.waitForTransform("base_link", "camera_polar_frame", ros::Time(0), ros::Duration(1.0));
            tf_listener_.lookupTransform("base_link", "camera_polar_frame", ros::Time(0), transform);
            T_base_camera_ = getTransformMatrix(transform);
            ROS_INFO_STREAM("T_base_camera = \n" << T_base_camera_);
        } catch (tf::TransformException& ex) {
            ROS_ERROR("TF Error: %s", ex.what());
        }
    }

    string getDateTimeString()
    {
        auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        std::stringstream ss;
        ss << std::put_time(std::localtime(&t), "%Y_%m_%d_%H_%M_%S");
        return ss.str();
    }

    void run()
    {
        ros::Rate loop_rate(30);
        while (ros::ok())
        {
            Mat show_color, show_gray;
            {
                std::lock_guard<std::mutex> lock(image_mutex_);
                if (!S0_color_display_.empty()) show_color = S0_color_display_.clone();
                if (!S0_gray_display_.empty()) show_gray = S0_gray_display_.clone();
            }

            if (!show_color.empty()) imshow("Polar_color", show_color);
            if (!show_gray.empty()) imshow("Polar_gray", show_gray);

            if((char)waitKey(10) == 32) // Spacebar
            {
                saveJointPositions();
                saveCameraPose();
                std::cout << "Save data successfully" << endl;
                std::cout << "Press space to start write ..." << std::endl;
                if((char)waitKey() == 32)
                {
                    writeData();
                    writeImage();
                    std::cout << "Save image successfully" << endl;
                }
            }

            loop_rate.sleep();
        }
    }

    void imageCallback(const robot_control_VS::PolarizedImagesConstPtr& color_msg, 
                       const robot_control_VS::PolarizedImagesConstPtr& gray_msg)
    {
        try {
            img_polar_color_0 = cv_bridge::toCvCopy(color_msg->image_0, "bgr8")->image;
            img_polar_color_45 = cv_bridge::toCvCopy(color_msg->image_45, "bgr8")->image;
            img_polar_color_90 = cv_bridge::toCvCopy(color_msg->image_90, "bgr8")->image;
            img_polar_color_135 = cv_bridge::toCvCopy(color_msg->image_135, "bgr8")->image;
            img_polar_color_S0 = cv_bridge::toCvCopy(color_msg->image_S0, "bgr8")->image;

            img_polar_gray_0 = cv_bridge::toCvCopy(gray_msg->image_0, "mono8")->image;
            img_polar_gray_45 = cv_bridge::toCvCopy(gray_msg->image_45, "mono8")->image;
            img_polar_gray_90 = cv_bridge::toCvCopy(gray_msg->image_90, "mono8")->image;
            img_polar_gray_135 = cv_bridge::toCvCopy(gray_msg->image_135, "mono8")->image;
            img_polar_gray_S0 = cv_bridge::toCvCopy(gray_msg->image_S0, "mono8")->image;
            
            {
                std::lock_guard<std::mutex> lock(image_mutex_);
                img_polar_color_S0.copyTo(S0_color_display_);
                img_polar_gray_S0.copyTo(S0_gray_display_);               
                img_polar_color_S0.copyTo(S0_color_save_); 
            }
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "save_pose_image");
    
    ros::AsyncSpinner spinner(1);
    spinner.start();

    PolarizedCameraNode node;
    node.run();

    spinner.stop();
    return 0;
}

