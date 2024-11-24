#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <rosbag/bag.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <string>

using namespace std;
using namespace sensor_msgs;

string location = "/home/cyh/Work/visual_servoing_ws/src/visual_servoing/image_process/resource/data/";
string time_now;

string get_date_time()
{
	auto to_string = [](const std::chrono::system_clock::time_point& t)->std::string
	{
		auto as_time_t = std::chrono::system_clock::to_time_t(t);
		struct tm tm;
#if defined(WIN32) || defined(_WINDLL)
		localtime_s(&tm, &as_time_t);  //win api，线程安全，而std::localtime线程不安全
#else
		localtime_r(&as_time_t, &tm);//linux api，线程安全
#endif

		std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(t.time_since_epoch());
		char buf[128];
		snprintf(buf, sizeof(buf), "%04d_%02d_%02d_%02d_%02d_%02d",
			tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
		return buf;
	};

	std::chrono::system_clock::time_point t = std::chrono::system_clock::now();
	return to_string(t);
}

void rosbag_record_Callback(const ImageConstPtr& image_color_msg)
{
    rosbag::Bag bag;

    bag.open(location + time_now + "_rosbag_record.bag", rosbag::bagmode::Write);

    bag.write("/VS/polarized_image", ros::Time::now(), image_color_msg);

    bag.close();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rosbag_record");
    ros::NodeHandle n;

    ros::Subscriber sub_polized = n.subscribe("/VS/polarized_image", 1, rosbag_record_Callback);

    time_now = get_date_time();

    ros::spin();
    return 0;
}