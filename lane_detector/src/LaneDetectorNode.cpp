#include <cmath>
#include <string>
#include <stdexcept>
#include "lane_detector/LaneDetectorNode.h"

using namespace std;
using namespace cv;

LaneDetectorNode::LaneDetectorNode()
{
	nh_ = ros::NodeHandle("~");
	/* if NodeHangle("~"), then (write -> /lane_detector/write)	*/
#if RC_CAR
	control_pub_ = nh_.advertise<std_msgs::String>("write", 100);
#elif	SCALE_PLATFORM
	control_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("ackermann", 100);
#endif

#if DEBUG
	true_color_pub_ = nh_.advertise<sensor_msgs::Image>("truecolor", 10);
	binary_pub_ = nh_.advertise<sensor_msgs::Image>("binary", 10);
	printlog_pub_ = nh_.advertise<std_msgs::String>("printlog", 10);
#endif

#if WEBCAM
	image_sub_ = nh_.subscribe("/usb_cam/image_raw", 100, &LaneDetectorNode::imageCallback, this);
#elif	PROSILICA_GT_CAM
	image_sub_ = nh_.subscribe("/camera/image_raw", 100, &LaneDetectorNode::imageCallback, this);
#endif

	int resize_width = 0;
	int resize_height = 0;
	int steer_max_angle = 0;

	getRosParamForConstValue(resize_width, resize_height, steer_max_angle);

	lanedetector_ptr_ = unique_ptr<LaneDetector>(new LaneDetector(resize_width, resize_height, steer_max_angle));

	getRosParamForUpdate();
}

void LaneDetectorNode::imageCallback(const sensor_msgs::ImageConstPtr& image)
{
	Mat raw_img;
	try{
		raw_img = parseRawimg(image);
	} catch(const cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return ;
	} catch(const std::runtime_error& e) {
		cerr << e.what() << endl;
	}

    getRosParamForUpdate();

    int steer_control_value = lanedetector_ptr_->laneDetecting(raw_img);

#if	RC_CAR
	std_msgs::String control_msg = makeControlMsg(steer_control_value);
	printData(control_msg);
#elif	SCALE_PLATFORM
	ackermann_msgs::AckermannDriveStamped control_msg = makeControlMsg();
	printData();
#endif

	control_pub_.publish(control_msg);

#if DEBUG
	true_color_pub_.publish(getDetectColorImg());
	binary_pub_.publish(getDetectBinaryImg());
	printlog_pub_.publish(getPrintlog());
#endif
}

#if DEBUG
sensor_msgs::ImagePtr LaneDetectorNode::getDetectColorImg()
{
	const Mat& image = lanedetector_ptr_->getRoiColorImg();
	return cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
}

sensor_msgs::ImagePtr LaneDetectorNode::getDetectBinaryImg()
{
	const Mat& image = lanedetector_ptr_->getRoiBinaryImg();
	return cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
}

std_msgs::String LaneDetectorNode::getPrintlog()
{
	string log;
 	log += "#### Algorithm Time #### \n";
	log += (string)"it took : " + to_string(lanedetector_ptr_->getOnceDetectTime()) + "ms, " + "avg: " + to_string(lanedetector_ptr_->getAvgDetectTime()) + " fps : " + to_string(1000 / lanedetector_ptr_->getAvgDetectTime()) + '\n';
	log += "#### Control #### \n";
	log += "steering angle: " + to_string(lanedetector_ptr_->getRealSteerAngle()) + '\n';
	log += "throttle: " + to_string(throttle_) + '\n';
	log += "#### Ros Param #### \n";
	log += "bin_thres: " + to_string(lanedetector_ptr_->getBinaryThres()) + '\n';
	log += "steer_max_angle: " + to_string(lanedetector_ptr_->getSteerMaxAngle()) + '\n';
	log += "yaw_factor: " + to_string(lanedetector_ptr_->getYawFactor() * 100) + "% -> " + to_string(lanedetector_ptr_->getYawFactor()) + '\n';
	log += "lateral_factor: " + to_string(lanedetector_ptr_->getLateralFactor() * 100) + "% -> " + to_string(lanedetector_ptr_->getLateralFactor()) + '\n';
	log += "---------------------------------\n";

	std_msgs::String log_msg;
	log_msg.data = log;
	return log_msg;
}

#endif

Mat LaneDetectorNode::parseRawimg(const sensor_msgs::ImageConstPtr& image)
{
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);

	Mat raw_img = cv_ptr->image;

	if (raw_img.empty()) {
		throw std::runtime_error("frame is empty!");
	}

	return raw_img;
}

void LaneDetectorNode::getRosParamForConstValue(int& width, int& height, int& steer_max_angle)
{
	nh_.getParam("resize_width", width);
	nh_.getParam("resize_height", height);
	nh_.getParam("steer_max_angle", steer_max_angle);
}

void LaneDetectorNode::getRosParamForUpdate()
{
	int paramArr[4];
	nh_.getParam("bin_thres", paramArr[0]);
	nh_.getParam("detect_y_offset", paramArr[1]);
	nh_.getParam("yaw_factor", paramArr[2]);
	nh_.getParam("lateral_factor", paramArr[3]);
	nh_.getParam("throttle", throttle_);

	lanedetector_ptr_->setBinaryThres(paramArr[0]);
	lanedetector_ptr_->setDetectYOffset(paramArr[1]);
	lanedetector_ptr_->setYawFactor((double)paramArr[2] / 100);
	lanedetector_ptr_->setLateralFactor((double)paramArr[3] / 100);
}

#if RC_CAR
std_msgs::String LaneDetectorNode::makeControlMsg(int steer)
{
	std_msgs::String control_msg;
	control_msg.data = string(to_string(steer)) + "," + string(to_string(throttle_)) + ","; // Make message
	return control_msg;
}

void LaneDetectorNode::printData(std_msgs::String control_msg)
{
 	cout << "#### Algorithm Time ####" << endl;
	cout << "it took : " << lanedetector_ptr_->getOnceDetectTime() << "ms, " << "avg: " << lanedetector_ptr_->getAvgDetectTime() << " fps : " << 1000 / lanedetector_ptr_->getAvgDetectTime() << endl;
	cout << "#### Control ####" << endl;
	cout << "steering angle: " << lanedetector_ptr_->getRealSteerAngle() << endl;
	cout << "control msg: " << control_msg.data << endl;
	cout << "#### Ros Param ####" << endl;
	cout << "bin_thres: " << lanedetector_ptr_->getBinaryThres() << endl;
	cout << "steer_max_angle: " << lanedetector_ptr_->getSteerMaxAngle() << endl;
	cout << "yaw_factor: " << lanedetector_ptr_->getYawFactor() * 100 << "% -> " << lanedetector_ptr_->getYawFactor() << endl;
	cout << "lateral_factor: " << lanedetector_ptr_->getLateralFactor() * 100 << "% -> " << lanedetector_ptr_->getLateralFactor() << endl;
	cout << "---------------------------------" << endl;
}

#elif SCALE_PLATFORM
ackermann_msgs::AckermannDriveStamped LaneDetectorNode::makeControlMsg()
{
	ackermann_msgs::AckermannDriveStamped control_msg;
	//control_msg.drive.steering_angle = steer_control_value;
	control_msg.drive.steering_angle = lanedetector_ptr_->getRealSteerAngle();
	control_msg.drive.speed = throttle_;
	return control_msg;
}

void LaneDetectorNode::printData()
{
 	cout << "#### Algorithm Time ####" << endl;
	cout << "it took : " << lanedetector_ptr_->getOnceDetectTime() << "ms, " << "avg: " << lanedetector_ptr_->getAvgDetectTime() << " fps : " << 1000 / lanedetector_ptr_->getAvgDetectTime() << endl;
	cout << "#### Control ####" << endl;
	cout << "steering angle: " << lanedetector_ptr_->getRealSteerAngle() << endl;
	cout << "throttle: " << throttle_ << endl;
	cout << "#### Ros Param ####" << endl;
	cout << "bin_thres: " << lanedetector_ptr_->getBinaryThres() << endl;
	cout << "steer_max_angle: " << lanedetector_ptr_->getSteerMaxAngle() << endl;
	cout << "yaw_factor: " << lanedetector_ptr_->getYawFactor() * 100 << "% -> " << lanedetector_ptr_->getYawFactor() << endl;
	cout << "lateral_factor: " << lanedetector_ptr_->getLateralFactor() * 100 << "% -> " << lanedetector_ptr_->getLateralFactor() << endl;
	cout << "---------------------------------" << endl;
}
#endif
