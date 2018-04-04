#ifndef LANEDETECTOR_H
#define LANEDETECTOR_H

#include "opencv2/opencv.hpp"
#include "ConditionalCompile.h"
#include "LinePointDetector.h"

class LaneDetector
{

public:
	LaneDetector(const int width, const int height, const int steer_max_angle, const int left_img_offset, const int right_img_offset, const float full_width_rate);

	void setBinaryThres(const int bin_thres);
	void setDetectYOffset(const int detect_y_offset);
	void setControlFactor(const double control_factor);

	int getWidth() const;
	int getHeight() const;
	int getBinaryThres() const;
	int getDetectYOffset() const;
	int getSteerMaxAngle() const;
	int getRealSteerAngle() const;
	double getControlFactor() const;
	double getOnceDetectTime() const;
	double getAvgDetectTime() const;

	// void getRoiBinaryImg(const cv::Point& left_top, const cv::Size& roi_size);

	// main wrapper function
	int laneDetecting(const cv::Mat& raw_left_img, const cv::Mat& raw_right_img);

protected:
	void resetLeftPoint();
	void resetRightPoint();

	void updateNextPoint();

	double calculateAngle();
	void calculateOnceDetectTime(const int64 start_time, const int64 finish_time);
	void calculateAvgDetectTime();

	bool detectOnlyOneLine() const;

	void visualizeLine() const;
	void showImg() const;

	int calculateSteerValue(const int center_steer_control_value, const int max_steer_control_value);

	bool haveToResetLeftPoint() const;
	bool haveToResetRightPoint() const;

	cv::Point detectLaneCenter() const;

	void reservePointReset();

	void getConcatBinImg();

	// wrapper function for lane detection
	// these functions are called on `laneDetecting` function
	void preprocessImg(const cv::Mat& raw_left_img, const cv::Mat& raw_right_img);
	void findLanePoints();
	void findSteering();
	void calDetectingTime(const int64 start_time, const int64 finish_time);
	void visualizeAll();

protected:
	// 화면 resize값
	const int RESIZE_WIDTH_ = 480;
	const int RESIZE_HEIGHT_ = 270;

	// 한 직선 보는 임계값
	const int LINE_PIXEL_THRESHOLD = 5;

	// 라인 y 좌표 비율 (0~100)
	int detect_y_offset_ = 30;

	// LaneDetector
	int binary_threshold_ = 210;
  	double control_factor_ = 1.0;
#if RC_CAR
	const int STEER_MAX_ANGLE_ = 45;
#elif SCALE_PLATFORM
	const int STEER_MAX_ANGLE_ = 27;
#endif

	// TODO: add on ros launch new three const value
	// 왼쪽,오른쪽 카메라에서 들어온 영상을 관심 영역만 자른다.
	// offset을 조절해서 겹치는 부분을 잘라낸다.
	const int LEFT_IMG_OFFSET_ = 210;
	const int RIGHT_IMG_OFFSET_ = 260;

	// RESIZE_WIDTH_와 비교했을때 full image의 width값 비율
	const float FULL_WIDTH_RATE_ = 6/4;

	// LaneDetector below all
	cv::Mat full_img_;
	cv::Mat resized_left_img_;		// resized image by (width, height)
	cv::Mat resized_right_img_;

	cv::Mat roi_binary_img_;

	cv::Point last_right_point_;
	cv::Point last_left_point_;

	cv::Point cur_right_point_;
	cv::Point cur_left_point_;

	cv::Point lane_middle_;

	double angle_;

	int frame_count_ = 0;	// for getting average fps
	double sum_of_detect_time_ = 0;
	double once_detect_time_ = 0;
	double detect_avg_time_ = 0;

  	LinePointDetector line_point_detector_;
};

#endif
