#include "lane_detector/LaneDetector.h"

using namespace std;
using namespace cv;

//TODO: algorithm comments of function - Jiwon Park 03/18/18

int LaneDetector::find_L0_x(Mat binary_img, int detect_y_offset, int *framecount_L , int last_point)
{
	int l0_x = last_point;

	for (int i = 1; i < binary_img.cols-1; i++)
	{
		if (binary_img.at<uchar>(binary_img.rows * detect_y_offset / 100, i) == 255)
		{
			l0_x = i;
			break;
		}
	}
	*framecount_L = *framecount_L + 1;  // framecount 가 0 이면 이게 실행된다. 즉 한번 실행되면 끝
	return l0_x;
}


int LaneDetector::find_R0_x(Mat binary_img, int detect_y_offset, int *framecount_R ,int last_point)
{
	int r0_x = last_point;

	for (int i = 1; i < binary_img.cols-1; i++)
	{
		if (binary_img.at<uchar>(binary_img.rows * detect_y_offset / 100, binary_img.cols - i) == 255)
		{
			r0_x = binary_img.cols - i;
			break;
		}
	}
	*framecount_R = *framecount_R + 1;
	return r0_x;
}


int LaneDetector::find_LN_x(Mat binary_img, int pre_point, int detect_y_offset, int THRESHOLD)
{
	int Left_N_x = pre_point;

	for (int i = 1; i < binary_img.cols-1; i++)
	{
		// 불연속 선 이라면
		if (!(binary_img.at<uchar>(binary_img.rows * detect_y_offset / 100, pre_point) == 255)
			&& !(binary_img.at<uchar>(binary_img.rows * detect_y_offset / 100 + 1, pre_point) == 255)
			&& !(binary_img.at<uchar>(binary_img.rows * detect_y_offset / 100 + 2, pre_point) == 255)
			&& !(binary_img.at<uchar>(binary_img.rows * detect_y_offset / 100 + 3, pre_point) == 255))
		{
			for (int i = 1; i < binary_img.cols; i++)
			{
				if (binary_img.at<uchar>(binary_img.rows * detect_y_offset / 100, i) == 255)
				{
					Left_N_x = i;
					// cout << "왼쪽 불연속점 입니다" << Left_N_x << endl;
					break;
				}
			}
			break;

		}
		else // 연속선 이라면
		{
			if (binary_img.at<uchar>(binary_img.rows * detect_y_offset / 100, i) == 255)
			{

				if ((i > pre_point + THRESHOLD) || (i < pre_point - THRESHOLD))	continue;

				Left_N_x = i;
				break;
			}
		}

	}

	return Left_N_x;
}



int LaneDetector::find_RN_x(Mat binary_img, int pre_point, int detect_y_offset, int THRESHOLD)
{
	int Right_N_x = pre_point;

	for (int i = 1; i < binary_img.cols-1; i++)
	{
		// 불연속 선 이라면
		if (!(binary_img.at<uchar>(binary_img.rows * detect_y_offset / 100, pre_point) == 255)
			&& !(binary_img.at<uchar>(binary_img.rows * detect_y_offset / 100 + 1, pre_point) == 255)
			&& !(binary_img.at<uchar>(binary_img.rows * detect_y_offset / 100 + 2, pre_point) == 255)
			&& !(binary_img.at<uchar>(binary_img.rows * detect_y_offset / 100 + 3, pre_point) == 255))
		{
			for (int i = 1; i < binary_img.cols; i++)
			{
				if (binary_img.at<uchar>(binary_img.rows * detect_y_offset / 100, binary_img.cols - i) == 255)
				{
					Right_N_x = binary_img.cols - i;
					// cout << "오른쪽 불연속선 입니다" << Right_N_x << endl;
					break;
				}
			}

			break;
		}
		else // 연속선 이라면
		{
			if (binary_img.at<uchar>(binary_img.rows * detect_y_offset / 100, binary_img.cols - i) == 255)
			{
				if ((binary_img.cols - i > pre_point + THRESHOLD) || (binary_img.cols - i < pre_point - THRESHOLD))	continue;

				Right_N_x = binary_img.cols - i;
				break;
			}
		}
	}

	return Right_N_x;
}
