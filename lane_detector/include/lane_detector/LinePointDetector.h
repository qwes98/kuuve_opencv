#ifndef LINEPOINTDETECTOR_H
#define LINEPOINTDETECTOR_H

#include "opencv2/opencv.hpp"
#include <iostream>

class LinePointDetector
{
	bool left_reset_flag_, right_reset_flag_;

public:
	LinePointDetector()
		: left_reset_flag_(true), right_reset_flag_(true)
	{}

	bool getLeftResetFlag() const;
	bool getRightResetFlag() const;

	void setLeftResetFlag(const bool left_reset_flag);
	void setRightResetFlag(const bool right_reset_flag);

	/**
	 * 왼쪽 초기점 찾는 함수
	 *
	 * @param binary_img 차선을 찾을 roi 이진화 이미지
	 * @param detect_y_offset 차선을 찾을 roi 이미지 상의 y좌표 (0~100)
	 * @param last_point 초기 점을 다시 잡을때 사용하는 전에 인식된 점
	 *
	 */
	int find_L0_x(const cv::Mat& binary_img, const int detect_y_offset, const int last_point);

	/**
	 * 오른쪽 초기점 찾는 함수
	 *
	 */
	int find_R0_x(const cv::Mat& binary_img, const int detect_y_offset, const int last_point);

	/**
	 * 이전에 찾은 왼쪽 점을 사용해 다음점 찾는 함수
	 *
	 * @param binery_img 차선을 찾을 roi 이진화 이미지
	 * @param pre_point 전에 인식된 점
	 * @param detect_y_offset 차선을 찾을 roi 이미지 상의 y좌표 (0~100)
	 * @param line_pixel_threshold 하나의 차선을 나타내는 픽셀 수
	 *
	 */
	int find_LN_x(const cv::Mat& binary_img, const int pre_point, const int detect_y_offset, const int line_pixel_threshold);

	/**
	 * 이전에 찾은 오른쪽 점을 사용해 다음점 찾는 함수
	 *
	 */
	int find_RN_x(const cv::Mat& binary_img, const int pre_point, const int detect_y_offset, const int line_pixel_threshold);

};

#endif
