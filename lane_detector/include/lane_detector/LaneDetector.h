#ifndef LANEDETECTOR_H
#define LANEDETECTOR_H

#include "opencv2/opencv.hpp"
#include <iostream>

class LaneDetector
{
	// int Right0_x, Left0_x;
	bool initialize_left_point, initialize_right_point;

public:
	LaneDetector()
		: initialize_left_point(true), initialize_right_point(true)
	{}
	//TODO: comments of function - Jiwon Park 03/18/18

	//TODO: have to change private funciton
	/**
	 * 왼쪽 초기점 찾는 함수
	 *
	 * @param binary_img 차선을 찾을 roi 이진화 이미지
	 * @param detect_y_coordinate 차선을 찾을 roi 이미지 상의 y좌표 (0~100)
	 * @param last_point 초기 점을 다시 잡을때 사용하는 전에 인식된 점
	 *
	 */
	 // TODO: erase framecount variable
	int find_L0_x(cv::Mat binary_img, int detect_y_offset, int *framecount_L , int last_point);

	/**
	 * 오른쪽 초기점 찾는 함수
	 *
	 */
	int find_R0_x(cv::Mat binary_img, int detect_y_offset, int *framecount_R , int last_point);

	/**
	 * 왼쪽 초기점을 찾은 후 다음점 찾는 함수
	 *
	 */
	 // Left0_x: 이전 지점
	 // THRESHOLD: 노이즈 제거를 위한 값
	int find_LN_x(cv::Mat binary_img, int pre_point, int detect_y_offset,int THRESHOLD);

	/**
	 * 오른쪽 초기점을 찾은 후 다음점 찾는 함수
	 *
	 */
	int find_RN_x(cv::Mat binary_img, int pre_point, int detect_y_offset, int THRESHOLD);

};

#endif
