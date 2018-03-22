#ifndef LINEPOINTDETECTOR_H
#define LINEPOINTDETECTOR_H

#include "opencv2/opencv.hpp"
#include <iostream>

class LinePointDetector
{
	bool left_reset_flag_, right_reset_flag_;

public:

public:
	LinePointDetector()
		: left_reset_flag_(true), right_reset_flag_(true)
	{}
	//TODO: comments of function - Jiwon Park 03/18/18

	bool getLeftResetFlag() const { return left_reset_flag_; }
	bool getRightResetFlag() const { return right_reset_flag_; }

	void setLeftResetFlag(bool left_reset_flag) { left_reset_flag_ = left_reset_flag; }
	void setRightResetFlag(bool right_reset_flag) { right_reset_flag_ = right_reset_flag; }

	/**
	 * 왼쪽 초기점 찾는 함수
	 *
	 * @param binary_img 차선을 찾을 roi 이진화 이미지
	 * @param detect_y_coordinate 차선을 찾을 roi 이미지 상의 y좌표 (0~100)
	 * @param last_point 초기 점을 다시 잡을때 사용하는 전에 인식된 점
	 *
	 */
	int find_L0_x(cv::Mat binary_img, int detect_y_offset, int last_point);

	/**
	 * 오른쪽 초기점 찾는 함수
	 *
	 */
	int find_R0_x(cv::Mat binary_img, int detect_y_offset, int last_point);

	/**
	 * 왼쪽 초기점을 찾은 후 다음점 찾는 함수
	 *
	 */
	 // Left0_x: 이전 지점
	 // THRESHOLD: 노이즈 제거를 위한 값
	int find_LN_x(cv::Mat binary_img, int pre_point, int detect_y_offset, const int line_pixel_threshold);

	/**
	 * 오른쪽 초기점을 찾은 후 다음점 찾는 함수
	 *
	 */
	int find_RN_x(cv::Mat binary_img, int pre_point, int detect_y_offset, const int line_pixel_threshold);

};

#endif
