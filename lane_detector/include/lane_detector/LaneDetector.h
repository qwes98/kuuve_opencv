#ifndef LANEDETECTOR_H
#define LANEDETECTOR_H

#include "opencv2/opencv.hpp"
#include "ConditionalCompile.h"
#include "LinePointDetector.h"

class LaneDetector
{

public:
	LaneDetector(const int width, const int height, const int steer_max_angle);

	void setBinaryThres(const int bin_thres);
	void setDetectYOffset(const int detect_y_offset);
	void setYawFactor(const double yaw_factor);
	void setLateralFactor(const double lateral_factor);

	int getWidth() const;
	int getHeight() const;
	int getBinaryThres() const;
	int getDetectYOffset() const;
	int getSteerMaxAngle() const;
	int getRealSteerAngle() const;
	double getYawFactor() const;
	double getLateralFactor() const;
	double getOnceDetectTime() const;
	double getAvgDetectTime() const;
	cv::Mat getRoiColorImg() const;
	cv::Mat getRoiBinaryImg() const;

	virtual void cvtToRoiBinaryImg(const cv::Point& left_top, const cv::Size& roi_size);

	// main wrapper function
	int laneDetecting(const cv::Mat& raw_img);

protected:
	void resetLeftPoint();
	void resetRightPoint();

	void updateNextPoint();

	double calculateYawError();
	double calculateLateralError();
	void calculateOnceDetectTime(const int64 start_time, const int64 finish_time);
	void calculateAvgDetectTime();

	bool detectedOnlyOneLine() const;

	void visualizeLine() const;
	virtual void showImg() const;

	int calculateSteerValue(const int center_steer_control_value, const int max_steer_control_value);

	bool haveToResetLeftPoint() const;
	bool haveToResetRightPoint() const;

	cv::Point detectLaneCenter() const;

	void reservePointReset();

	// wrapper function for lane detection
	// these functions are called on `laneDetecting` function
	virtual void preprocessImg(const cv::Mat& raw_img);
	virtual void findLanePoints();
	virtual void findSteering();
	void calDetectingTime(const int64 start_time, const int64 finish_time);
	virtual void visualizeAll();

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
	double yaw_factor_ = 0.5;
	double lateral_factor_ = 0.5;
#if RC_CAR
	const int STEER_MAX_ANGLE_ = 45;
#elif SCALE_PLATFORM
	const int STEER_MAX_ANGLE_ = 27;
#endif

	// LaneDetector below all
	cv::Mat resized_img_;		// resized image by (width, height)
	cv::Mat roi_binary_img_;

	cv::Point last_right_point_;
	cv::Point last_left_point_;

	cv::Point cur_right_point_;
	cv::Point cur_left_point_;

	cv::Point lane_middle_;

	double yaw_error_;
	double lateral_error_;

	double steer_angle_;	// calculated real steer angle

	int frame_count_ = 0;	// for getting average fps
	double sum_of_detect_time_ = 0;
	double once_detect_time_ = 0;
	double detect_avg_time_ = 0;

  	LinePointDetector line_point_detector_;
};

#endif
