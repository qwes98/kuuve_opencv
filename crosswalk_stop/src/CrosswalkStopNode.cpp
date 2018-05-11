#include <cmath>
#include <string>
#include <stdexcept>
#include "crosswalk_stop/CrosswalkStopNode.h"

using namespace std;
using namespace cv;

CrosswalkStopNode::CrosswalkStopNode()
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
	image_sub_ = nh_.subscribe("/usb_cam/image_raw", 1, &CrosswalkStopNode::imageCallback, this);
#elif	PROSILICA_GT_CAM
	image_sub_ = nh_.subscribe("/camera/image_raw", 1, &CrosswalkStopNode::imageCallback, this);
#endif

	int resize_width = 0;
	int resize_height = 0;
	int steer_max_angle = 0;
	int detect_line_count = 0;
	int stop_distance = 0;
	int stop_time = 0;

	getRosParamForConstValue(resize_width, resize_height, steer_max_angle, detect_line_count, stop_distance, stop_time);

	crosswalkstop_ptr_ = unique_ptr<CrosswalkStop>(new CrosswalkStop(resize_width, resize_height, steer_max_angle, detect_line_count, (double)stop_distance / 100, stop_time));

	getRosParamForUpdate();
}

void CrosswalkStopNode::imageCallback(const sensor_msgs::ImageConstPtr& image)
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

	int steer_control_value = crosswalkstop_ptr_->laneDetecting(raw_img);

  if(!cross_detected_ && crosswalkstop_ptr_->detectCrosswalk()) {
		// TODO: have to modify for braking
		throttle_ = 0;
		cross_detected_ = true;
	}


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

	if(cross_detected_ && !mission_cleared_) {
		ros::Duration(crosswalkstop_ptr_->getStopTime()).sleep();	// sleep for 3 seconds
		mission_cleared_ = true;
	}
}

#if DEBUG
sensor_msgs::ImagePtr CrosswalkStopNode::getDetectColorImg()
{
	const Mat& image = crosswalkstop_ptr_->getRoiColorImg();
	return cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
}

sensor_msgs::ImagePtr CrosswalkStopNode::getDetectFinalBinImg()
{
	const Mat& image = crosswalkstop_ptr_->getRoiBinaryImg();
	return cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
}

sensor_msgs::ImagePtr CrosswalkStopNode::getDetectGrayBinImg()
{
	const Mat& image = crosswalkstop_ptr_->getRoiGrayBinImg();
	return cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
}

sensor_msgs::ImagePtr CrosswalkStopNode::getDetectHsvSBinImg()
{
	const Mat& image = crosswalkstop_ptr_->getRoiHsvSBinImg();
	return cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
}

std_msgs::String CrosswalkStopNode::getPrintlog()
{
	string log;
 	log += "#### Algorithm Time #### \n";
	log += (string)"it took : " + to_string(crosswalkstop_ptr_->getOnceDetectTime()) + "ms, " + "avg: " + to_string(crosswalkstop_ptr_->getAvgDetectTime()) + " fps : " + to_string(1000 / crosswalkstop_ptr_->getAvgDetectTime()) + '\n';
	log += "#### Control #### \n";
	log += "steering angle: " + to_string(crosswalkstop_ptr_->getRealSteerAngle()) + '\n';
	log += "throttle: " + to_string(throttle_) + '\n';
	log += "#### Ros Param #### \n";
	log += "gray_bin_thres: " + to_string(crosswalkstop_ptr_->getGrayBinThres()) + '\n';
	log += "hsv_s_bin_thres: " + to_string(crosswalkstop_ptr_->getHsvSBinThres()) + '\n';
	log += "detect_line_count: " + to_string(crosswalkstop_ptr_->getDetectLineCount()) + '\n';
	for(int i = 0; i < crosswalkstop_ptr_->getDetectLineCount(); i++)
		log += "detect_y_offset_" + to_string(i+1) + ": " + to_string(crosswalkstop_ptr_->getDetectYOffset(i)) + '\n';
	log += "left_detect_offset: " + to_string(crosswalkstop_ptr_->getLeftDetectOffset()) + '\n';
	log += "right_detect_offset: " + to_string(crosswalkstop_ptr_->getRightDetectOffset()) + '\n';
	log += "steer_max_angle: " + to_string(crosswalkstop_ptr_->getSteerMaxAngle()) + '\n';
	log += "yaw_factor: " + to_string(crosswalkstop_ptr_->getYawFactor() * 100) + "% -> " + to_string(crosswalkstop_ptr_->getYawFactor()) + '\n';
	log += "lateral_factor: " + to_string(crosswalkstop_ptr_->getLateralFactor() * 100) + "% -> " + to_string(crosswalkstop_ptr_->getLateralFactor()) + '\n';
	log += "stop_distance: " + to_string(crosswalkstop_ptr_->getStopDistance()) + '\n';
	log += "stop_time: " + to_string(crosswalkstop_ptr_->getStopTime()) + '\n';
	log += "---------------------------------\n";

	std_msgs::String log_msg;
	log_msg.data = log;
	return log_msg;
}

#endif

Mat CrosswalkStopNode::parseRawimg(const sensor_msgs::ImageConstPtr& image)
{
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);

	Mat raw_img = cv_ptr->image;

	if (raw_img.empty()) {
		throw std::runtime_error("frame is empty!");
	}

	return raw_img;
}

void CrosswalkStopNode::getRosParamForConstValue(int& width, int& height, int& steer_max_angle, int& detect_line_count, int& stop_distance, int& stop_time)
{
	nh_.getParam("resize_width", width);
	nh_.getParam("resize_height", height);
	nh_.getParam("steer_max_angle", steer_max_angle);
	nh_.getParam("detect_line_count", detect_line_count);
	nh_.getParam("stop_distance", stop_distance);
	nh_.getParam("stop_time", stop_time);
}

void CrosswalkStopNode::getRosParamForUpdate()
{
	int paramArr[8];
	nh_.getParam("gray_bin_thres", paramArr[0]);
	nh_.getParam("hsv_s_bin_thres", paramArr[1]);
	nh_.getParam("left_detect_offset", paramArr[2]);
	nh_.getParam("right_detect_offset", paramArr[3]);
	nh_.getParam("yaw_factor", paramArr[4]);
	nh_.getParam("lateral_factor", paramArr[5]);
	nh_.getParam("roi_top_location", paramArr[6]);
	nh_.getParam("roi_bottom_location", paramArr[7]);
	nh_.getParam("throttle", throttle_);

	crosswalkstop_ptr_->setGrayBinThres(paramArr[0]);
	crosswalkstop_ptr_->setHsvSBinThres(paramArr[1]);
	crosswalkstop_ptr_->setLeftDetectOffset(paramArr[2]);
	crosswalkstop_ptr_->setRightDetectOffset(paramArr[3]);
	crosswalkstop_ptr_->setYawFactor((double)paramArr[4] / 100);
	crosswalkstop_ptr_->setLateralFactor((double)paramArr[5] / 100);
	crosswalkstop_ptr_->setRoiTopLocation(paramArr[6]);
	crosswalkstop_ptr_->setRoiBottomLocation(paramArr[7]);

	int detect_line_count = crosswalkstop_ptr_->getDetectLineCount();
	for(int i = 0; i < detect_line_count; i++) {
		int y_offset = 0;
		nh_.getParam("detect_y_offset_" + to_string(i+1), y_offset);
		crosswalkstop_ptr_->setDetectYOffset(y_offset, i);
	}
}

#if RC_CAR
std_msgs::String CrosswalkStopNode::makeControlMsg(int steer)
{
	std_msgs::String control_msg;
	control_msg.data = string(to_string(steer)) + "," + string(to_string(throttle_)) + ","; // Make message
	return control_msg;
}

void CrosswalkStopNode::printData(std_msgs::String control_msg)
{
 	cout << "#### Algorithm Time ####" << endl;
	cout << "it took : " << crosswalkstop_ptr_->getOnceDetectTime() << "ms, " << "avg: " << crosswalkstop_ptr_->getAvgDetectTime() << " fps : " << 1000 / crosswalkstop_ptr_->getAvgDetectTime() << endl;
	cout << "#### Control ####" << endl;
	cout << "steering angle: " << crosswalkstop_ptr_->getRealSteerAngle() << endl;
	cout << "control msg: " << control_msg.data << endl;
	cout << "#### Ros Param ####" << endl;
	cout << "gray_bin_thres: " << crosswalkstop_ptr_->getGrayBinThres() << endl;
	cout << "hsv_s_bin_thres: " << crosswalkstop_ptr_->getHsvSBinThres() << endl;
	cout << "detect_line_count: " << crosswalkstop_ptr_->getDetectLineCount() << endl;
	for(int i = 0; i < crosswalkstop_ptr_->getDetectLineCount(); i++)
		cout << "detect_y_offset_" << i+1 << ": " << crosswalkstop_ptr_->getDetectYOffset(i) << endl;
	cout << "left_detect_offset: " << crosswalkstop_ptr_->getLeftDetectOffset() << endl;
	cout << "right_detect_offset: " << crosswalkstop_ptr_->getRightDetectOffset() << endl;
	cout << "steer_max_angle: " << crosswalkstop_ptr_->getSteerMaxAngle() << endl;
	cout << "yaw_factor: " << crosswalkstop_ptr_->getYawFactor() * 100 << "% -> " << crosswalkstop_ptr_->getYawFactor() << endl;
	cout << "lateral_factor: " << crosswalkstop_ptr_->getLateralFactor() * 100 << "% -> " << crosswalkstop_ptr_->getLateralFactor() << endl;
	cout << "stop_distance: " << crosswalkstop_ptr_->getStopDistance() << endl;
	cout << "stop_time: " << crosswalkstop_ptr_->getStopTime() << endl;
	cout << "---------------------------------" << endl;
}

#elif SCALE_PLATFORM
ackermann_msgs::AckermannDriveStamped CrosswalkStopNode::makeControlMsg()
{
	ackermann_msgs::AckermannDriveStamped control_msg;
	//control_msg.drive.steering_angle = steer_control_value;
	control_msg.drive.steering_angle = crosswalkstop_ptr_->getRealSteerAngle();
	control_msg.drive.speed = throttle_;
	return control_msg;
}

void CrosswalkStopNode::printData()
{
 	cout << "#### Algorithm Time ####" << endl;
	cout << "it took : " << crosswalkstop_ptr_->getOnceDetectTime() << "ms, " << "avg: " << crosswalkstop_ptr_->getAvgDetectTime() << " fps : " << 1000 / crosswalkstop_ptr_->getAvgDetectTime() << endl;
	cout << "#### Control ####" << endl;
	cout << "steering angle: " << crosswalkstop_ptr_->getRealSteerAngle() << endl;
	cout << "throttle: " << throttle_ << endl;
	cout << "#### Ros Param ####" << endl;
	cout << "gray_bin_thres: " << crosswalkstop_ptr_->getGrayBinThres() << endl;
	cout << "hsv_s_bin_thres: " << crosswalkstop_ptr_->getHsvSBinThres() << endl;
	cout << "detect_line_count: " << crosswalkstop_ptr_->getDetectLineCount() << endl;
	for(int i = 0; i < crosswalkstop_ptr_->getDetectLineCount(); i++)
		cout << "detect_y_offset_" << i+1 << ": " << crosswalkstop_ptr_->getDetectYOffset(i) << endl;
	cout << "left_detect_offset: " << crosswalkstop_ptr_->getLeftDetectOffset() << endl;
	cout << "right_detect_offset: " << crosswalkstop_ptr_->getRightDetectOffset() << endl;
	cout << "steer_max_angle: " << crosswalkstop_ptr_->getSteerMaxAngle() << endl;
	cout << "yaw_factor: " << crosswalkstop_ptr_->getYawFactor() * 100 << "% -> " << crosswalkstop_ptr_->getYawFactor() << endl;
	cout << "lateral_factor: " << crosswalkstop_ptr_->getLateralFactor() * 100 << "% -> " << crosswalkstop_ptr_->getLateralFactor() << endl;
	cout << "stop_distance: " << crosswalkstop_ptr_->getStopDistance() << endl;
	cout << "stop_time: " << crosswalkstop_ptr_->getStopTime() << endl;
	cout << "---------------------------------" << endl;
}
#endif
