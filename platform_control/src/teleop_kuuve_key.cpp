#include <iostream>
#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

using namespace std;

int main(int argc, char** argv){
	double speed;

	ros::init(argc, argv, "teleop_kuuve_key");

	ros::NodeHandle nh;

	ros::Publisher ackermann_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("ackermann", 10);
	ros::Rate loop_rate(5);
    if(nh.getParam("/teleop_kuuve_key/speed", speed)){} 
	else{speed=1.0;}
	//nh.param<double>("speed", speed, 3);

	ackermann_msgs::AckermannDriveStamped ackermann_data;
	ackermann_data.drive.steering_angle = 0;
	ackermann_data.drive.speed = 0;
	while(ros::ok()){
		cout << "speed is " << speed << endl;
		ackermann_data.drive.steering_angle += 1;
//		ackermann_data.drive.speed += 0.5;
		ackermann_data.drive.speed = speed;
		if(ackermann_data.drive.steering_angle >= 28) ackermann_data.drive.steering_angle = -28;
//		if(ackermann_data.drive.speed >= 15) ackermann_data.drive.speed = 0;
		cout << "send:" << ackermann_data.drive.steering_angle << endl;
		ackermann_pub.publish(ackermann_data);
		ros::spinOnce();
		loop_rate.sleep();	
	}
	return 0;
}
