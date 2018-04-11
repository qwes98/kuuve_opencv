#ifndef CROSSWALKSTOPNODE_H
#define CROSSWALKSTOPNODE_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <signal.h>
#include <memory>
#include "crosswalk_stop/CrosswalkStop.h"
#include "lane_detector/ConditionalCompile.h"

class CrosswalkStopNode
{
public:
    CrosswalkStopNode();

	void imageCallback(const sensor_msgs::ImageConstPtr& image);

private:
	void getRosParamForConstValue(int& width, int& height, int& steer_max_angle, int& stop_distance);
	void getRosParamForUpdate();

	cv::Mat parseRawimg(const sensor_msgs::ImageConstPtr& image);

#if DEBUG
	sensor_msgs::ImagePtr getDetectColorImg();
	sensor_msgs::ImagePtr getDetectBinaryImg();
#endif

#if RC_CAR
	std_msgs::String makeControlMsg(int steer);
	void printData(std_msgs::String control_msg);
#elif SCALE_PLATFORM
	ackermann_msgs::AckermannDriveStamped makeControlMsg();
	void printData();
#endif

private:
	ros::NodeHandle nh_;
	ros::Publisher control_pub_;
#if DEBUG
	ros::Publisher true_color_pub_;
	ros::Publisher binary_pub_;
#endif
	ros::Subscriber image_sub_;

	int throttle_ = 0;

    bool cross_detected = false;
    bool mission_cleared = false;

    std::unique_ptr<CrosswalkStop> crosswalkstop_ptr_;
};

#endif
