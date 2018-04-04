#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_sequencer.h>
#include <sensor_msgs/Image.h>

using namespace message_filters;

void callback(const sensor_msgs::Image::ConstPtr& msg)
{
	ROS_INFO("string1: [%d]", msg->height);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "time_synchronizer");

	ros::NodeHandle nh;

	message_filters::Subscriber<sensor_msgs::Image> sub(nh, "image1", 1);

	// delay and update
	message_filters::TimeSequencer<sensor_msgs::Image> seq(sub, ros::Duration(1), ros::Duration(0.1), 10);
	seq.registerCallback(callback);

	ros::spin();

	return 0;
}
