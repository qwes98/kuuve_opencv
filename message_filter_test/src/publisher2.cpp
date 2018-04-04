#include <ros/ros.h>
#include <sensor_msgs/Image.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "publisher2");

	ros::NodeHandle nh;

	ros::Publisher pub = nh.advertise<sensor_msgs::Image>("image2", 100);
	ros::Rate loop_rate(10);

	int count = 0;
	while(ros::ok()) {
		sensor_msgs::Image msg;
		msg.height = count;

		ROS_INFO("%d", msg.height);

		pub.publish(msg);
	
		loop_rate.sleep();
		count++;
	}

	return 0;
}
