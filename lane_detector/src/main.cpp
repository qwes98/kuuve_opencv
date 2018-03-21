#include "lane_detector/LaneDetector.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <cmath>
#include <string>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

using namespace std;
using namespace cv;

class LaneDetectorNode
{
public:
	LaneDetectorNode();
	void imageCallback(const sensor_msgs::ImageConstPtr& image);
	void getRosParam()
	{
		nh_.getParam("resize_width", width);
		nh_.getParam("resize_height", height);
		nh_.getParam("bin_thres", binary_threshold_);
		nh_.getParam("control_factor", control_factor_);
		nh_.getParam("throttle", throttle_);
	}

	void printData(double ms, double avg, std_msgs::String control_msg)
	{
		cout << "it took : " << ms << "ms." << "avg: " << avg << " fps : " << 1000 / avg << endl;
		cout << "#### Control ####" << endl;
		cout << "steering angle: " << (int)angle_ << endl;
		cout << "control msg: " << control_msg.data << endl;
		cout << "#### Ros Param ####" << endl;
		cout << "bin_thres: " << binary_threshold_ << endl;
		cout << "control_factor: " << control_factor_ << endl;
		cout << "---------------------------------" << endl;
	}

	void getRoiBinaryImg()
	{
		Mat roi_gray_img;
		Mat roi_color_img = resized_img_(Rect(0, height / 2, width, height / 2));
		cvtColor(roi_color_img, roi_gray_img, COLOR_BGR2GRAY);
		threshold(roi_gray_img, roi_binary_img_, binary_threshold_, 255, THRESH_BINARY);
	}

	void detectLeftInitialPoint()
	{
		last_left_point_.x = lanedetector_.find_L0_x(roi_binary_img_, detect_y_offset_, &detect_first_left_point_ , last_left_point_.x);
		last_left_point_.y = roi_binary_img_.rows * detect_y_offset_ / 100;
	}

	void detectRightInitialPoint()
	{
		last_right_point_.x = lanedetector_.find_R0_x(roi_binary_img_, detect_y_offset_, &detect_first_right_point_ , last_right_point_.x);
		last_right_point_.y = roi_binary_img_.rows * detect_y_offset_ / 100;
	}

	void updateNextPoint()
	{
		cur_right_point_.x = lanedetector_.find_RN_x(roi_binary_img_, last_right_point_.x, detect_y_offset_, PIXEL_THRESHOLD);
		cur_right_point_.y = roi_binary_img_.rows * detect_y_offset_ / 100;
		last_right_point_.x = cur_right_point_.x;
		cur_left_point_.x = lanedetector_.find_LN_x(roi_binary_img_, last_left_point_.x, detect_y_offset_, PIXEL_THRESHOLD);
		cur_left_point_.y = roi_binary_img_.rows * detect_y_offset_ / 100;
		last_left_point_.x = cur_left_point_.x;
	}

	double calculateAngle()
	{
		int dx = lane_middle_.x - roi_binary_img_.cols / 2;
		int dy = roi_binary_img_.rows - lane_middle_.y;
		return atan2(dx, dy) * 180 / CV_PI;
	}

	double calculateDetectionTime(int64 start_time, int64 finish_time)
	{
		return (finish_time - start_time) * 1000 / getTickFrequency();
	}

	bool detectOnlyOneLine()
	{
		return abs(cur_right_point_.x - cur_left_point_.x) < 15;
	}

	void visualizeLine()
	{
		line(resized_img_, cur_right_point_ + Point(0, height / 2), cur_left_point_ + Point(0, height / 2), Scalar(0, 255, 0), 5);
		line(resized_img_, lane_middle_ + Point(0, height / 2), Point(resized_img_.cols / 2, resized_img_.rows), Scalar(0, 0, 255), 5);
	}

	void showImg()
	{
		imshow("binary img", roi_binary_img_);
		imshow("frame", resized_img_);
		waitKey(3);
	}

	int calculateSteerValue()
	{
		int angle_for_msg = 0;	// For parsing double value to int
		// arduino steering range: 1100 < steer < 1900
		if(angle_ < control_factor_ && angle_ > (-1) * control_factor_)
			angle_for_msg = static_cast<int>(1500 + 400 / control_factor_ * angle_);
		else if(angle_ >= control_factor_) {
			angle_for_msg = 1900;
			angle_ = control_factor_;		// For print angle on console
		}
		else if(angle_ <= (-1) * control_factor_) {
			angle_for_msg = 1100;
			angle_ = (-1) * control_factor_;
		}

		return angle_for_msg;
	}

	std_msgs::String makeControlMsg(int steer_value)
	{
		std_msgs::String control_msg;
		control_msg.data = string(to_string(steer_value)) + "," + string(to_string(throttle_)) + ","; // Make message
		return control_msg;
	}

private:
	// 화면 resize값
	// main
	int width = 960/2;
	int height = 540/2;

	// 한 직선 보는 임계값
	const int PIXEL_THRESHOLD = 5;

	// 라인 y 좌표 비율 (0~100)
	const int detect_y_offset_ = 30;

private:
	// ros variables
	ros::NodeHandle nh_;
	ros::Publisher control_pub_;
	ros::Subscriber image_sub_;

	// could modify by using rosparam
	int binary_threshold_ = 110;
	int control_factor_ = 25;
	int throttle_ = 1515;

	int frame_count_ = 0;	// for getting average fps


	Mat resized_img_;		// resized image by (width, height)
	Mat roi_binary_img_;

  //TODO: 0->true
	int detect_first_right_point_ = 0;
	int detect_first_left_point_ = 0;

	Point last_right_point_;
	Point last_left_point_;

	// 왼쪽 좌표, 오른쪽 좌표
	Point cur_right_point_;
	Point cur_left_point_;

	Point lane_middle_;

	double angle_;

	LaneDetector lanedetector_;

	double sum_of_detection_time_ = 0;
};

LaneDetectorNode::LaneDetectorNode()
{
	nh_ = ros::NodeHandle("~");
	// NodeHangle("~") -> (write -> /lane_detector/write)
	control_pub_ = nh_.advertise<std_msgs::String>("write", 100);
	image_sub_ = nh_.subscribe("/usb_cam/image_raw", 100, &LaneDetectorNode::imageCallback, this);

	last_right_point_.x = 0;
	last_left_point_.x = 0;
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

		// Opencv code start
		int64 start_time = getTickCount();

		Mat raw_img = cv_ptr->image;

		if (raw_img.empty())
		{
			cout << "frame is empty!" << endl;
			return;
		}

		frame_count_++;

		getRosParam();

		resize(raw_img, resized_img_, Size(width, height));

		///////
		getRoiBinaryImg();

		// 초기점 구하기
		if (detect_first_left_point_ == 0) {
			detectLeftInitialPoint();
		}

		if (detect_first_right_point_ == 0) {
			detectRightInitialPoint();
		}

		// 포인트 구하기
		updateNextPoint();

		lane_middle_ = Point((cur_right_point_.x + cur_left_point_.x) / 2, roi_binary_img_.rows * detect_y_offset_ / 100);

		angle_ = calculateAngle();

		int64 finish_time = getTickCount();

		double ms = calculateDetectionTime(start_time, finish_time);

		sum_of_detection_time_ += ms;
		double avg = sum_of_detection_time_ / frame_count_;

		if (detectOnlyOneLine())
		{
			detect_first_left_point_ = 0;
			detect_first_right_point_ = 0;
		}

		visualizeLine();
		showImg();

		////////////////////////////
		int steer_value = calculateSteerValue();
		std_msgs::String control_msg = makeControlMsg(steer_value);

		printData(ms, avg, control_msg);
		cout << "width: " << width << endl;
		cout << "heigth: " << height << endl;

		control_pub_.publish(control_msg);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "lane_detector");

#if 1
	LaneDetectorNode lane_detector;

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
