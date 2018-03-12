#ifndef LINEDETECTOR_H
#define LINEDETECTOR_H

#include "opencv2/opencv.hpp"
#include <iostream>

using namespace cv;
using namespace std;

class LaneDetect
{
	int Right0_x, Left0_x;
	
public:

	int find_R0_x(Mat binary_img, int img_height, int *framecount_R , int Right0_x);
	int find_L0_x(Mat binary_img, int img_height, int *framecount_L , int Left0_x);

	int find_RN_x(Mat binary_img, int Right0_x, int img_height, int THRESHOLD);
	int find_LN_x(Mat binary_img, int Left0_x, int img_height ,int THRESHOLD);

};

#endif
