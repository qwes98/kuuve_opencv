#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <sensor_msgs/Image.h>

using namespace message_filters;

void callback(const sensor_msgs::Image::ConstPtr& msg1, const sensor_msgs::Image::ConstPtr& msg2)
{
	ROS_INFO("string1: [%d], string2: [%d]", msg1->height, msg2->height);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "time_synchronizer");

	ros::NodeHandle nh;

	message_filters::Subscriber<sensor_msgs::Image> sub1(nh, "image1", 1);
	message_filters::Subscriber<sensor_msgs::Image> sub2(nh, "image2", 1);

	typedef sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
	// ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub1, sub2);
	sync.registerCallback(boost::bind(&callback, _1, _2));
	 
	// message_filters::TimeSynchronizer<std_msgs::String, std_msgs::String> sync(sub1, sub2, 10);
	// sync.registerCallback(boost::bind(&callback, _1, _2));
			

	ros::spin();

	return 0;
}
