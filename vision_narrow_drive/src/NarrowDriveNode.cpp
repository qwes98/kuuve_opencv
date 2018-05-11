#include <cmath>
#include <string>
#include <stdexcept>
#include "vision_narrow_drive/NarrowDriveNode.h"

using namespace std;
using namespace cv;

NarrowDriveNode::NarrowDriveNode()
{
	nh_ = ros::NodeHandle("~");
	/* if NodeHangle("~"), then (write -> /lane_detector/write)	*/
#if RC_CAR
	control_pub_ = nh_.advertise<std_msgs::String>("write", 10);
#elif	SCALE_PLATFORM
	control_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("ackermann", 10);
#endif

#if DEBUG
	true_color_pub_ = nh_.advertise<sensor_msgs::Image>("truecolor", 10);
	final_bin_pub_ = nh_.advertise<sensor_msgs::Image>("final_bin", 10);
	bin_from_gray_pub_ = nh_.advertise<sensor_msgs::Image>("gray_bin", 10);
	bin_from_hsv_s_pub_ = nh_.advertise<sensor_msgs::Image>("hsv_s_bin", 10);
	printlog_pub_ = nh_.advertise<std_msgs::String>("printlog", 10);
#endif

#if WEBCAM
	image_sub_ = nh_.subscribe("/usb_cam/image_raw", 1, &NarrowDriveNode::imageCallback, this);
#elif	PROSILICA_GT_CAM
	image_sub_ = nh_.subscribe("/camera/image_raw", 1, &NarrowDriveNode::imageCallback, this);
#endif

	int resize_width = 0;
	int resize_height = 0;
	int steer_max_angle = 0;
	int detect_line_count = 0;
	int sustaining_time = 0;

	getRosParamForConstValue(resize_width, resize_height, steer_max_angle, detect_line_count, sustaining_time);

	visionpathplanner_ptr_ = unique_ptr<VisionPathPlanner>(new VisionPathPlanner(resize_width, resize_height, steer_max_angle, detect_line_count, sustaining_time));

	getRosParamForUpdate();
}

void NarrowDriveNode::imageCallback(const sensor_msgs::ImageConstPtr& image)
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

	int steer_control_value = visionpathplanner_ptr_->laneDetecting(raw_img);

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
	final_bin_pub_.publish(getDetectFinalBinImg());
	bin_from_gray_pub_.publish(getDetectGrayBinImg());
	bin_from_hsv_s_pub_.publish(getDetectHsvSBinImg());
	printlog_pub_.publish(getPrintlog());
#endif

}

#if DEBUG
sensor_msgs::ImagePtr NarrowDriveNode::getDetectColorImg()
{
	const Mat& image = visionpathplanner_ptr_->getRoiColorImg();
	return cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
}

sensor_msgs::ImagePtr NarrowDriveNode::getDetectFinalBinImg()
{
	const Mat& image = visionpathplanner_ptr_->getRoiBinaryImg();
	return cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
}

sensor_msgs::ImagePtr NarrowDriveNode::getDetectGrayBinImg()
{
	const Mat& image = visionpathplanner_ptr_->getRoiGrayBinImg();
	return cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
}

sensor_msgs::ImagePtr NarrowDriveNode::getDetectHsvSBinImg()
{
	const Mat& image = visionpathplanner_ptr_->getRoiHsvSBinImg();
	return cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
}

std_msgs::String NarrowDriveNode::getPrintlog()
{
	string log;
 	log += "#### Algorithm Time #### \n";
	log += (string)"it took : " + to_string(visionpathplanner_ptr_->getOnceDetectTime()) + "ms, " + "avg: " + to_string(visionpathplanner_ptr_->getAvgDetectTime()) + " fps : " + to_string(1000 / visionpathplanner_ptr_->getAvgDetectTime()) + '\n';
	log += "#### Control #### \n";
	log += "steering angle: " + to_string(visionpathplanner_ptr_->getRealSteerAngle()) + '\n';
	log += "throttle: " + to_string(throttle_) + '\n';
	log += "#### Ros Param #### \n";
	log += "gray_bin_thres: " + to_string(visionpathplanner_ptr_->getGrayBinThres()) + '\n';
	log += "hsv_s_bin_thres: " + to_string(visionpathplanner_ptr_->getHsvSBinThres()) + '\n';
	log += "detect_line_count: " + to_string(visionpathplanner_ptr_->getDetectLineCount()) + '\n';
	for(int i = 0; i < visionpathplanner_ptr_->getDetectLineCount(); i++)
		log += "detect_y_offset_" + to_string(i+1) + ": " + to_string(visionpathplanner_ptr_->getDetectYOffset(i)) + '\n';
	log += "left_detect_offset: " + to_string(visionpathplanner_ptr_->getLeftDetectOffset()) + '\n';
	log += "right_detect_offset: " + to_string(visionpathplanner_ptr_->getRightDetectOffset()) + '\n';
	log += "steer_max_angle: " + to_string(visionpathplanner_ptr_->getSteerMaxAngle()) + '\n';
	log += "yaw_factor: " + to_string(visionpathplanner_ptr_->getYawFactor() * 100) + "% -> " + to_string(visionpathplanner_ptr_->getYawFactor()) + '\n';
	log += "lateral_factor: " + to_string(visionpathplanner_ptr_->getLateralFactor() * 100) + "% -> " + to_string(visionpathplanner_ptr_->getLateralFactor()) + '\n';
	log += "sustaining_time: " + to_string(visionpathplanner_ptr_->getSustainingTime()) + '\n';
	log += "change_pixel_thres: " + to_string(visionpathplanner_ptr_->getChangePixelThres()) + '\n';
	log += "---------------------------------\n";

	std_msgs::String log_msg;
	log_msg.data = log;
	return log_msg;
}

#endif

Mat NarrowDriveNode::parseRawimg(const sensor_msgs::ImageConstPtr& image)
{
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);

	Mat raw_img = cv_ptr->image;

	if (raw_img.empty()) {
		throw std::runtime_error("frame is empty!");
	}

	return raw_img;
}

void NarrowDriveNode::getRosParamForConstValue(int& width, int& height, int& steer_max_angle, int& detect_line_count, int& sustaining_time)
{
	nh_.getParam("resize_width", width);
	nh_.getParam("resize_height", height);
	nh_.getParam("steer_max_angle", steer_max_angle);
	nh_.getParam("detect_line_count", detect_line_count);
	nh_.getParam("sustaining_time", sustaining_time);
}

void NarrowDriveNode::getRosParamForUpdate()
{
	int paramArr[7];
	nh_.getParam("gray_bin_thres", paramArr[0]);
	nh_.getParam("hsv_s_bin_thres", paramArr[1]);
	nh_.getParam("left_detect_offset", paramArr[2]);
	nh_.getParam("right_detect_offset", paramArr[3]);
	nh_.getParam("yaw_factor", paramArr[4]);
	nh_.getParam("lateral_factor", paramArr[5]);
	nh_.getParam("change_pixel_thres", paramArr[6]);
	nh_.getParam("throttle", throttle_);

	visionpathplanner_ptr_->setGrayBinThres(paramArr[0]);
	visionpathplanner_ptr_->setHsvSBinThres(paramArr[1]);
	visionpathplanner_ptr_->setLeftDetectOffset(paramArr[2]);
	visionpathplanner_ptr_->setRightDetectOffset(paramArr[3]);
	visionpathplanner_ptr_->setYawFactor((double)paramArr[4] / 100);
	visionpathplanner_ptr_->setLateralFactor((double)paramArr[5] / 100);
	visionpathplanner_ptr_->setChangePixelThres(paramArr[6]);

	int detect_line_count = visionpathplanner_ptr_->getDetectLineCount();
	for(int i = 0; i < detect_line_count; i++) {
		int y_offset = 0;
		nh_.getParam("detect_y_offset_" + to_string(i+1), y_offset);
		visionpathplanner_ptr_->setDetectYOffset(y_offset, i);

		// visionpathplanner_ptr_->setTimeAfterDetectObs(paramArr[6], i);
	}
}

#if RC_CAR
std_msgs::String NarrowDriveNode::makeControlMsg(int steer)
{
	std_msgs::String control_msg;
	control_msg.data = string(to_string(steer)) + "," + string(to_string(throttle_)) + ","; // Make message
	return control_msg;
}

void NarrowDriveNode::printData(std_msgs::String control_msg)
{
 	cout << "#### Algorithm Time ####" << endl;
	cout << "it took : " << visionpathplanner_ptr_->getOnceDetectTime() << "ms, " << "avg: " << visionpathplanner_ptr_->getAvgDetectTime() << " fps : " << 1000 / visionpathplanner_ptr_->getAvgDetectTime() << endl;
	cout << "#### Control ####" << endl;
	cout << "steering angle: " << visionpathplanner_ptr_->getRealSteerAngle() << endl;
	cout << "control msg: " << control_msg.data << endl;
	cout << "#### Ros Param ####" << endl;
	cout << "gray_bin_thres: " << visionpathplanner_ptr_->getGrayBinThres() << endl;
	cout << "hsv_s_bin_thres: " << visionpathplanner_ptr_->getHsvSBinThres() << endl;
	cout << "detect_line_count: " << visionpathplanner_ptr_->getDetectLineCount() << endl;
	for(int i = 0; i < visionpathplanner_ptr_->getDetectLineCount(); i++)
		cout << "detect_y_offset_" << i+1 << ": " << visionpathplanner_ptr_->getDetectYOffset(i) << endl;
	cout << "left_detect_offset: " << visionpathplanner_ptr_->getLeftDetectOffset() << endl;
	cout << "right_detect_offset: " << visionpathplanner_ptr_->getRightDetectOffset() << endl;
	cout << "steer_max_angle: " << visionpathplanner_ptr_->getSteerMaxAngle() << endl;
	cout << "yaw_factor: " << visionpathplanner_ptr_->getYawFactor() * 100 << "% -> " << visionpathplanner_ptr_->getYawFactor() << endl;
	cout << "lateral_factor: " << visionpathplanner_ptr_->getLateralFactor() * 100 << "% -> " << visionpathplanner_ptr_->getLateralFactor() << endl;
	cout << "sustaining_time: " << visionpathplanner_ptr_->getSustainingTime() << endl;
	cout << "change_pixel_thres: " << visionpathplanner_ptr_->getChangePixelThres() << endl;
	cout << "---------------------------------" << endl;
}

#elif SCALE_PLATFORM
ackermann_msgs::AckermannDriveStamped NarrowDriveNode::makeControlMsg()
{
	ackermann_msgs::AckermannDriveStamped control_msg;
	//control_msg.drive.steering_angle = steer_control_value;
	control_msg.drive.steering_angle = visionpathplanner_ptr_->getRealSteerAngle();
	control_msg.drive.speed = throttle_;
	return control_msg;
}

void NarrowDriveNode::printData()
{
 	cout << "#### Algorithm Time ####" << endl;
	cout << "it took : " << visionpathplanner_ptr_->getOnceDetectTime() << "ms, " << "avg: " << visionpathplanner_ptr_->getAvgDetectTime() << " fps : " << 1000 / visionpathplanner_ptr_->getAvgDetectTime() << endl;
	cout << "#### Control ####" << endl;
	cout << "steering angle: " << visionpathplanner_ptr_->getRealSteerAngle() << endl;
	cout << "throttle: " << throttle_ << endl;
	cout << "#### Ros Param ####" << endl;
	cout << "gray_bin_thres: " << visionpathplanner_ptr_->getGrayBinThres() << endl;
	cout << "hsv_s_bin_thres: " << visionpathplanner_ptr_->getHsvSBinThres() << endl;
	cout << "detect_line_count: " << visionpathplanner_ptr_->getDetectLineCount() << endl;
	for(int i = 0; i < visionpathplanner_ptr_->getDetectLineCount(); i++)
		cout << "detect_y_offset_" << i+1 << ": " << visionpathplanner_ptr_->getDetectYOffset(i) << endl;
	cout << "left_detect_offset: " << visionpathplanner_ptr_->getLeftDetectOffset() << endl;
	cout << "right_detect_offset: " << visionpathplanner_ptr_->getRightDetectOffset() << endl;
	cout << "steer_max_angle: " << visionpathplanner_ptr_->getSteerMaxAngle() << endl;
	cout << "yaw_factor: " << visionpathplanner_ptr_->getYawFactor() * 100 << "% -> " << visionpathplanner_ptr_->getYawFactor() << endl;
	cout << "lateral_factor: " << visionpathplanner_ptr_->getLateralFactor() * 100 << "% -> " << visionpathplanner_ptr_->getLateralFactor() << endl;
	cout << "sustaining_time: " << visionpathplanner_ptr_->getSustainingTime() << endl;
	cout << "change_pixel_thres: " << visionpathplanner_ptr_->getChangePixelThres() << endl;
	cout << "---------------------------------" << endl;
}
#endif
