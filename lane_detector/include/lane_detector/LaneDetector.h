#ifndef LANEDETECTOR_H
#define LANEDETECTOR_H

#include "opencv2/opencv.hpp"
#include "ConditionalCompile.h"
#include "LinePointDetector.h"
#include "MyException.h"
#include <memory>

class LaneDetector
{

public:
//TODO: caller have to pass detect_line_count value
	LaneDetector(const int width, const int height, const int steer_max_angle, const int detect_line_count);

	void setGrayBinThres(const int bin_thres);
	bool setDetectYOffset(const int detect_y_offset, const int index) throw(my_out_of_range);
	void setYawFactor(const double yaw_factor, const int index);
	void setLateralFactor(const double lateral_factor, const int index);
	void setRoiTopLocation(const int top_rate);
	void setRoiBottomLocation(const int bottom_rate);
//ADDED
	void setContiDetectPixel(const int continuous_detect_pixel);

	int getWidth() const;
	int getHeight() const;
	int getGrayBinThres() const;
	int getDetectLineCount() const;
	int getDetectYOffset(const int index) const throw(my_out_of_range);
	int getSteerMaxAngle() const;
	int getRealSteerAngle() const;
	int getRoiTopLocation() const;
	int getRoiBottomLocation() const;
//ADDED
	int getContiDetectPixel() const;
	double getYawFactor(const int index) const;
	double getLateralFactor(const int index) const;
	double getOnceDetectTime() const;
	double getAvgDetectTime() const;
	int getControlLineIndex() const;
	void getRoiColorImg(cv::Mat& img);
	void getRoiBinaryImg(cv::Mat& img);

	virtual void cvtToRoiBinaryImg(const cv::Point& left_top, const cv::Size& roi_size) = 0;

	// main wrapper function
	int laneDetecting(const cv::Mat& raw_img);

protected:
	virtual void resetLeftPoint(const int index) throw(my_out_of_range) = 0;
	virtual void resetRightPoint(const int index) throw(my_out_of_range) = 0;

	virtual void updateNextPoint(const int index) throw(my_out_of_range) = 0;

	double calculateYawError(const int index) throw(my_out_of_range);
	double calculateLateralError(const int index) throw(my_out_of_range);
	void calculateOnceDetectTime(const int64 start_time, const int64 finish_time);
	void calculateAvgDetectTime();

	bool detectedOnlyOneLine(const int index) const throw(my_out_of_range);

	void visualizeLine(const int index) const throw(my_out_of_range);
	virtual void showImg() const;

	int calculateSteerValue(const int center_steer_control_value, const int max_steer_control_value);

	bool haveToResetLeftPoint(const int index) const throw(my_out_of_range);
	bool haveToResetRightPoint(const int index) const throw(my_out_of_range);

	bool notDetectedAllLines(const int index) const throw(my_out_of_range);

	virtual cv::Point detectLaneCenter(const int index) throw(my_out_of_range);

	void reservePointReset(const int index) throw(my_out_of_range);

	// wrapper function for lane detection
	// these functions are called on `laneDetecting` function
	void preprocessImg(const cv::Mat& raw_img);
	void findLanePoints() throw(my_out_of_range);
	void findSteering() throw(my_out_of_range);
	void calDetectingTime(const int64 start_time, const int64 finish_time);
	void visualizeAll() throw(my_out_of_range);
	void checkPointReset() throw(my_out_of_range);

protected:
	// 화면 resize값
	const int RESIZE_WIDTH_ = 480;
	const int RESIZE_HEIGHT_ = 270;

	// Roi top and bottom(y) location
	// 0 is top of raw_img and 100 is bottom of raw_img
	int roi_top_location_ = 50;
	int roi_bottom_location_ = 100;

	// 한 직선 보는 임계값
	const int LINE_PIXEL_THRESHOLD = 11;

	// 차선 검출 시 사용하는 수평 라인의 갯수
	const int DETECT_LINE_COUNT_ = 1;

	// 라인 y 좌표 비율 컨테이너(0~100)
	std::unique_ptr<int[]> detect_y_offset_arr_;

	// LaneDetector
	int gray_bin_thres_ = 210;
	std::unique_ptr<double[]> yaw_factor_arr_;
	std::unique_ptr<double[]> lateral_factor_arr_;

	int top_line_index_ = 0;
	int bottom_line_index_ = 1;
	int control_line_index_ = 0;
#if RC_CAR
	const int STEER_MAX_ANGLE_ = 45;
#elif SCALE_PLATFORM
	const int STEER_MAX_ANGLE_ = 27;
#endif

	// LaneDetector below all
	cv::Mat resized_img_;		// resized image by (width, height)
	cv::Mat roi_binary_img_;

	std::unique_ptr<cv::Point[]> last_right_point_arr_;
	std::unique_ptr<cv::Point[]> last_left_point_arr_;

	std::unique_ptr<cv::Point[]> cur_right_point_arr_;
	std::unique_ptr<cv::Point[]> cur_left_point_arr_;

	std::unique_ptr<cv::Point[]> lane_middle_arr_;

	double yaw_error_;
	double lateral_error_;

	double steer_angle_;	// calculated real steer angle

	int frame_count_ = 0;	// for getting average fps
	double sum_of_detect_time_ = 0;
	double once_detect_time_ = 0;
	double detect_avg_time_ = 0;

	std::unique_ptr<LinePointDetector[]> line_point_detector_arr_;
};

#endif
