#ifndef LANEDETECTORNODE_H
#define LANEDETECTORNODE_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <signal.h>
#include <memory>
#include "lane_detector/LaneDetector.h"
#include "lane_detector/ConditionalCompile.h"

typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
typedef message_filters::Synchronizer<MySyncPolicy> ImgSynchronizer;
typedef message_filters::Subscriber<sensor_msgs::Image> ImgSubscriber;

class LaneDetectorNode
{
public:
	LaneDetectorNode();

	void imageCallback(const sensor_msgs::ImageConstPtr& left_img, const sensor_msgs::ImageConstPtr& right_img);

private:
	void getRosParamForConstValue(int& width, int& height, int& steer_max_angle, int& left_img_offset, int& right_img_offset, float& full_width_rate);
	void getRosParamForUpdate();

	cv::Mat parseRawimg(const sensor_msgs::ImageConstPtr& image);

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
	std::unique_ptr<ImgSubscriber> left_img_sub_;
	std::unique_ptr<ImgSubscriber> right_img_sub_;
	std::unique_ptr<ImgSynchronizer> sub_sync_;

	int throttle_ = 0;

	std::unique_ptr<LaneDetector> lanedetector_ptr_;
};

#endif
