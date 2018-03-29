#ifndef LANEDETECTORNODE_H
#define LANEDETECTORNODE_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <memory>
#include "lane_detector/LaneDetector.h"

class LaneDetectorNode
{
public:
	LaneDetectorNode();
	void imageCallback(const sensor_msgs::ImageConstPtr& image);

	void getRosParamForConstValue(int& width, int& height, int& steer_max_angle);
	void getRosParamForUpdate();

	void printData(std_msgs::String control_msg);
	void printData();

	std_msgs::String makeControlMsg(int steer);

private:
	// ros variables
	// main
	ros::NodeHandle nh_;
	ros::Publisher control_pub_;
	ros::Subscriber image_sub_;

	int throttle_ = 1515;

	std::unique_ptr<LaneDetector> lanedetector_ptr_;
};

#endif
