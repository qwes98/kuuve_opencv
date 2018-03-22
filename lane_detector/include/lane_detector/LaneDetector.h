#ifndef LANEDETECTOR_H
#define LANEDETECTOR_H

#include "opencv2/opencv.hpp"
#include "LinePointDetector.h"

// TODO: have to erase
using namespace cv;

class LaneDetector
{
	// 화면 resize값
	const int RESIZE_WIDTH_;
	const int RESIZE_HEIGHT_;

	// 한 직선 보는 임계값
	const int LINE_PIXEL_THRESHOLD = 5;

	// 라인 y 좌표 비율 (0~100)
	int detect_y_offset_ = 30;

	// LaneDetector
	int binary_threshold_ = 110;
	const int STEER_MAX_ANGLE_;
  double control_factor_ = 1.0;

	// LaneDetector below all
	Mat resized_img_;		// resized image by (width, height)
	Mat roi_binary_img_;

	Point last_right_point_;
	Point last_left_point_;

	// 왼쪽 좌표, 오른쪽 좌표
	Point cur_right_point_;
	Point cur_left_point_;

	Point lane_middle_;

	double angle_;

	int frame_count_ = 0;	// for getting average fps
	double sum_of_detect_time_ = 0;
	double once_detect_time_ = 0;
	double detect_avg_time_ = 0;

  LinePointDetector line_point_detector_;
public:

LaneDetector(int width, int height, int steer_max_angle);

void setBinaryThres(int bin_thres) { binary_threshold_ = bin_thres; }
void setDetectYOffset(int detect_y_offset) { detect_y_offset_ = detect_y_offset; }
void setControlFactor(double control_factor) { control_factor_ = control_factor; }


int getWidth() const { return RESIZE_WIDTH_; }
int getHeight() const { return RESIZE_HEIGHT_; }
int getBinaryThres() const { return binary_threshold_; }
int getDetectYOffset() const { return detect_y_offset_; }
int getSteerMaxAngle() const { return STEER_MAX_ANGLE_; }
double getControlFactor() const { return control_factor_; }

double getOnceDetectTime() const { return once_detect_time_; }
double getAvgDetectTime() const { return detect_avg_time_; }

int testFunction(cv::Mat raw_img);


	void getRoiBinaryImg(Point left_top, Size roi_size)
	{
		Mat roi_gray_img;
		Mat roi_color_img = resized_img_(Rect(left_top.x, left_top.y, roi_size.width, roi_size.height));
		cvtColor(roi_color_img, roi_gray_img, COLOR_BGR2GRAY);
		threshold(roi_gray_img, roi_binary_img_, binary_threshold_, 255, THRESH_BINARY);
	}

	void resetLeftPoint()
	{
		last_left_point_.x = line_point_detector_.find_L0_x(roi_binary_img_, detect_y_offset_, last_left_point_.x);
		last_left_point_.y = roi_binary_img_.rows * detect_y_offset_ / 100;
	}

	void resetRightPoint()
	{
		last_right_point_.x = line_point_detector_.find_R0_x(roi_binary_img_, detect_y_offset_, last_right_point_.x);
		last_right_point_.y = roi_binary_img_.rows * detect_y_offset_ / 100;
	}

	void updateNextPoint()
	{
		cur_right_point_.x = line_point_detector_.find_RN_x(roi_binary_img_, last_right_point_.x, detect_y_offset_, LINE_PIXEL_THRESHOLD);
		cur_right_point_.y = roi_binary_img_.rows * detect_y_offset_ / 100;
		last_right_point_.x = cur_right_point_.x;

		cur_left_point_.x = line_point_detector_.find_LN_x(roi_binary_img_, last_left_point_.x, detect_y_offset_, LINE_PIXEL_THRESHOLD);
		cur_left_point_.y = roi_binary_img_.rows * detect_y_offset_ / 100;
		last_left_point_.x = cur_left_point_.x;
	}

	double calculateAngle()
	{
		int dx = lane_middle_.x - roi_binary_img_.cols / 2;
		int dy = roi_binary_img_.rows - lane_middle_.y;
		return atan2(dx, dy) * 180 / CV_PI;
	}

	void calculateOnceDetectTime(int64 start_time, int64 finish_time)
	{
		once_detect_time_ = (finish_time - start_time) * 1000 / getTickFrequency();
	}

	void calculateAvgDetectTime()
	{
		sum_of_detect_time_ += once_detect_time_;
		detect_avg_time_ = sum_of_detect_time_ / frame_count_;
	}

	bool detectOnlyOneLine()
	{
		return abs(cur_right_point_.x - cur_left_point_.x) < 15;
	}

	void visualizeLine()
	{
		line(resized_img_, cur_right_point_ + Point(0, RESIZE_HEIGHT_ / 2), cur_left_point_ + Point(0, RESIZE_HEIGHT_ / 2), Scalar(0, 255, 0), 5);
		line(resized_img_, lane_middle_ + Point(0, RESIZE_HEIGHT_ / 2), Point(resized_img_.cols / 2, resized_img_.rows), Scalar(0, 0, 255), 5);
	}

	void showImg()
	{
		imshow("binary img", roi_binary_img_);
		imshow("frame", resized_img_);
		waitKey(3);
	}

  // TODO: add params(middle, max)
	int calculateSteerValue()
	{
		int steer_control_value = 0;	// For parsing double value to int
		// arduino steering range: 1100 < steer < 1900
		if(angle_ * control_factor_ < STEER_MAX_ANGLE_ && angle_ * control_factor_ > (-1) * STEER_MAX_ANGLE_)
			steer_control_value = static_cast<int>(1500 + 400 / STEER_MAX_ANGLE_ * (angle_ * control_factor_));
		else if(angle_ * control_factor_ >= STEER_MAX_ANGLE_) {
			steer_control_value = 1900;
			angle_ = STEER_MAX_ANGLE_ / control_factor_;		// For print angle on console
		}
		else if(angle_ * control_factor_ <= (-1) * STEER_MAX_ANGLE_) {
			steer_control_value = 1100;
			angle_ = (-1) * STEER_MAX_ANGLE_ / control_factor_;
		}

		return steer_control_value;
	}

  int getRealSteerAngle() { return angle_ * control_factor_; }

	// For testFunction
	bool haveToResetLeftPoint() { return line_point_detector_.getLeftResetFlag(); }
	bool haveToResetRightPoint() { return line_point_detector_.getRightResetFlag(); }

	Point detectLaneCenter() { return Point((cur_right_point_.x + cur_left_point_.x) / 2, roi_binary_img_.rows * detect_y_offset_ / 100); }

	void reservePointReset()
	{
		line_point_detector_.setLeftResetFlag(true);
		line_point_detector_.setRightResetFlag(true);
	}
};

#endif

LaneDetector::LaneDetector(int width, int height, int steer_max_angle)
  : RESIZE_WIDTH_(width), RESIZE_HEIGHT_(height), STEER_MAX_ANGLE_(steer_max_angle)
{
	// LaneDetector
	last_right_point_.x = 0;
	last_left_point_.x = 0;
}

int LaneDetector::testFunction(cv::Mat raw_img)
{
		int64 start_time = getTickCount();

		frame_count_++;

		resize(raw_img, resized_img_, Size(RESIZE_WIDTH_, RESIZE_HEIGHT_));

		getRoiBinaryImg(Point(0, RESIZE_HEIGHT_ / 2), Size(RESIZE_WIDTH_, RESIZE_HEIGHT_ / 2));

		if(haveToResetLeftPoint()) {
			resetLeftPoint();
		}

		if (haveToResetRightPoint()) {
			resetRightPoint();
		}

		updateNextPoint();

		lane_middle_ = detectLaneCenter();

		angle_ = calculateAngle();

		int64 finish_time = getTickCount();

		calculateOnceDetectTime(start_time, finish_time);

		calculateAvgDetectTime();

		if (detectOnlyOneLine()) {
			reservePointReset();
		}

		visualizeLine();
		showImg();

		return calculateSteerValue();
}
