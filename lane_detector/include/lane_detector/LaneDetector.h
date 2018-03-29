#ifndef LANEDETECTOR_H
#define LANEDETECTOR_H

#include "opencv2/opencv.hpp"
#include "LinePointDetector.h"

class LaneDetector
{

public:
	LaneDetector(const int width, const int height, const int steer_max_angle);

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

	void getRoiBinaryImg(const cv::Point& left_top, const cv::Size& roi_size);

	int laneDetecting(const cv::Mat& raw_img);

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

	// wrapper function for lane detection
	// these functions are called on `laneDetecting` function
	void preprocessImg(const cv::Mat& raw_img);
	void findLanePoints();
	void findSteering();
	void calDetectingTime(const int64 start_time, const int64 finish_time);
	void visualizeAll();

protected:
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
	cv::Mat resized_img_;		// resized image by (width, height)
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
