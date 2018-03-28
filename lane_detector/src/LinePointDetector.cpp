#include "lane_detector/LinePointDetector.h"

using namespace std;
using namespace cv;

//TODO: algorithm comments of function - Jiwon Park 03/18/18

bool LinePointDetector::getLeftResetFlag() const { return left_reset_flag_; }
bool LinePointDetector::getRightResetFlag() const { return right_reset_flag_; }

void LinePointDetector::setLeftResetFlag(const bool left_reset_flag) { left_reset_flag_ = left_reset_flag; }
void LinePointDetector::setRightResetFlag(const bool right_reset_flag) { right_reset_flag_ = right_reset_flag; }

int LinePointDetector::find_L0_x(const Mat& binary_img, const int detect_y_offset, const int last_point)
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
	left_reset_flag_ = false;
	return l0_x;
}

int LinePointDetector::find_R0_x(const Mat& binary_img, const int detect_y_offset, const int last_point)
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
	right_reset_flag_ = false;
	return r0_x;
}

int LinePointDetector::find_LN_x(const Mat& binary_img, const int pre_point, const int detect_y_offset, const int line_pixel_threshold)
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

				if ((i > pre_point + line_pixel_threshold) || (i < pre_point - line_pixel_threshold))	continue;

				Left_N_x = i;
				break;
			}
		}

	}

	return Left_N_x;
}

int LinePointDetector::find_RN_x(const Mat& binary_img, const int pre_point, const int detect_y_offset, const int line_pixel_threshold)
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
				if ((binary_img.cols - i > pre_point + line_pixel_threshold) || (binary_img.cols - i < pre_point - line_pixel_threshold))	continue;

				Right_N_x = binary_img.cols - i;
				break;
			}
		}
	}

	return Right_N_x;
}
