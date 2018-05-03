#ifndef NARROWDRIVENODE_H
#define NARROWDRIVENODE_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <signal.h>
#include <memory>
#include "vision_static_avoidance/VisionPathPlanner.h"
#include "lane_detector/ConditionalCompile.h"

class NarrowDriveNode
{
public:
  NarrowDriveNode();

	void imageCallback(const sensor_msgs::ImageConstPtr& image);

private:
	void getRosParamForConstValue(int& width, int& height, int& steer_max_angle, int& detect_line_count, int& sustaining_time);
	void getRosParamForUpdate();

	cv::Mat parseRawimg(const sensor_msgs::ImageConstPtr& image);

#if DEBUG
	sensor_msgs::ImagePtr getDetectColorImg();
	sensor_msgs::ImagePtr getDetectFinalBinImg();
	sensor_msgs::ImagePtr getDetectGrayBinImg();
	sensor_msgs::ImagePtr getDetectHsvSBinImg();
	std_msgs::String getPrintlog();
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
	ros::Publisher final_bin_pub_;
	ros::Publisher bin_from_gray_pub_;
	ros::Publisher bin_from_hsv_s_pub_;
	ros::Publisher printlog_pub_;
#endif
	ros::Subscriber image_sub_;

	int throttle_ = 0;

  std::unique_ptr<VisionPathPlanner> visionpathplanner_ptr_;
};

#endif
