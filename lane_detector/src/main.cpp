#include <ros/ros.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <signal.h>
#include <cmath>
#include <string>
#include <memory>
#include "lane_detector/LaneDetector.h"
#include "lane_detector/ConditionalCompile.h"

using namespace std;
using namespace cv;

class LaneDetectorNode
{
public:
	LaneDetectorNode();
	void imageCallback(const sensor_msgs::ImageConstPtr& image);

	void getRosParamForConstValue(int& width, int& height, int& steer_max_angle)
	{
		nh_.getParam("resize_width", width);
		nh_.getParam("resize_height", height);
		nh_.getParam("steer_max_angle", steer_max_angle);
	}

	void getRosParamForUpdate()
	{
		int paramArr[3];
		nh_.getParam("bin_thres", paramArr[0]);
		nh_.getParam("detect_y_offset", paramArr[1]);
		nh_.getParam("control_factor", paramArr[2]);
		nh_.getParam("throttle", throttle_);

		lanedetector_ptr_->setBinaryThres(paramArr[0]);
		lanedetector_ptr_->setDetectYOffset(paramArr[1]);
		lanedetector_ptr_->setControlFactor((double)paramArr[2] / 100);
	}

	void printData(std_msgs::String control_msg)
	{
		cout << "it took : " << lanedetector_ptr_->getOnceDetectTime() << "ms, " << "avg: " << lanedetector_ptr_->getAvgDetectTime() << " fps : " << 1000 / lanedetector_ptr_->getAvgDetectTime() << endl;
		cout << "#### Control ####" << endl;
		// TODO: modify to getOptimizedSteer() function
		cout << "steering angle: " << lanedetector_ptr_->getRealSteerAngle() << endl;
		cout << "control msg: " << control_msg.data << endl;
		cout << "#### Ros Param ####" << endl;
		cout << "bin_thres: " << lanedetector_ptr_->getBinaryThres() << endl;
		cout << "steer_max_angle: " << lanedetector_ptr_->getSteerMaxAngle() << endl;
		cout << "control_factor: " << lanedetector_ptr_->getControlFactor() * 100 << "% -> " << lanedetector_ptr_->getControlFactor() << endl;
		cout << "---------------------------------" << endl;
	}

	void printData()
	{
		cout << "it took : " << lanedetector_ptr_->getOnceDetectTime() << "ms, " << "avg: " << lanedetector_ptr_->getAvgDetectTime() << " fps : " << 1000 / lanedetector_ptr_->getAvgDetectTime() << endl;
		cout << "#### Control ####" << endl;
		// TODO: modify to getOptimizedSteer() function
		cout << "steering angle: " << lanedetector_ptr_->getRealSteerAngle() << endl;
		cout << "throttle: " << throttle_ << endl;
		cout << "#### Ros Param ####" << endl;
		cout << "bin_thres: " << lanedetector_ptr_->getBinaryThres() << endl;
		cout << "steer_max_angle: " << lanedetector_ptr_->getSteerMaxAngle() << endl;
		cout << "control_factor: " << lanedetector_ptr_->getControlFactor() * 100 << "% -> " << lanedetector_ptr_->getControlFactor() << endl;
		cout << "---------------------------------" << endl;
	}

	std_msgs::String makeControlMsg(int steer)
	{
		std_msgs::String control_msg;
		control_msg.data = string(to_string(steer)) + "," + string(to_string(throttle_)) + ","; // Make message
		return control_msg;
	}

private:
	// ros variables
	// main
	ros::NodeHandle nh_;
	ros::Publisher control_pub_;
	ros::Subscriber image_sub_;

	int throttle_ = 1515;

	unique_ptr<LaneDetector> lanedetector_ptr_;
};

LaneDetectorNode::LaneDetectorNode()
{
	nh_ = ros::NodeHandle("~");
	// if NodeHangle("~"), then (write -> /lane_detector/write)
#if RC_CAR
	control_pub_ = nh_.advertise<std_msgs::String>("write", 100);
#elif	SCALE_PLATFORM
	control_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("ackermann", 100);
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
		cv_bridge::CvImagePtr cv_ptr;
		try {
			cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
		} catch(cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return ;
		}

		Mat raw_img = cv_ptr->image;

		if (raw_img.empty())
		{
			cout << "frame is empty!" << endl;
			return;
		}

		getRosParamForUpdate();

		// TODO: have to modify
		int steer_control_value = lanedetector_ptr_->laneDetecting(raw_img);

#if RC_CAR
		std_msgs::String control_msg = makeControlMsg(steer_control_value);
		printData(control_msg);
#elif	SCALE_PLATFORM
		ackermann_msgs::AckermannDriveStamped control_msg;
		//control_msg.drive.steering_angle = steer_control_value;
		control_msg.drive.steering_angle = lanedetector_ptr_->getRealSteerAngle();
		control_msg.drive.speed = throttle_;
		printData();
#endif
		control_pub_.publish(control_msg);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "lane_detector");

#if 1
	//LaneDetector lane_detector(960/2, 540/2, 45);
	LaneDetectorNode lane_detector_node;

#else
	VideoCapture cap(1);
	//cap.open("cameraimage_color_camera3.mp4");

	if (!cap.isOpened())
	{
		cout << "Not opened cap" << endl;
		return -1;
	}
#endif

	ros::spin();
	return 0;
}
