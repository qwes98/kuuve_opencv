#include "kuuve_parking/LineDetector.h"

using namespace std;
using namespace cv;

// ���� �ʱ��� ã��
int LaneDetect::find_L0_x(Mat binary_img, int img_height, int *framecount_L , int Right0_x)
{
	int l0_x = Right0_x;

	for (int i = 1; i < binary_img.cols-1; i++)
	{
		if (binary_img.at<uchar>(img_height , i) == 255)
		{
			l0_x = i;
			break;
		}
	}
	*framecount_L = *framecount_L + 1;  // framecount �� 0 �̸� �̰� �����ȴ�. �� �ѹ� �����Ǹ� ��
	return l0_x;
}


int LaneDetect::find_R0_x(Mat binary_img, int img_height , int *framecount_R ,int Left0_x)
{
	int r0_x = Left0_x;

	for (int i = 10; i < binary_img.cols-17; i++)
	{
		if (binary_img.at<uchar>(img_height , binary_img.cols - i) == 255)
		{
			r0_x = binary_img.cols - i;
			break;
		}
	}
	*framecount_R = *framecount_R + 1;
	return r0_x;
}


// �ʱ����� �־��� �� ������ ã��
int LaneDetect::find_LN_x(Mat binary_img, int Left0_x, int img_height , int THRESHOLD)
{
	int Left_N_x = Left0_x;

	for (int i = 1; i < binary_img.cols-1; i++)
	{
		// �ҿ��� �� �̶���
		if (!(binary_img.at<uchar>(binary_img.rows / 2 + img_height, Left0_x) == 255)
			&& !(binary_img.at<uchar>(binary_img.rows / 2 + img_height - 1, Left0_x ) == 255)
			&& !(binary_img.at<uchar>(binary_img.rows / 2 + img_height - 2, Left0_x ) == 255)
			&& !(binary_img.at<uchar>(binary_img.rows / 2 + img_height - 3, Left0_x ) == 255))
		{
			for (int i = 1; i < binary_img.cols; i++)
			{
				if (binary_img.at<uchar>(binary_img.rows / 2 + img_height, i) == 255)
				{
					Left_N_x = i;
					//cout << "���� �ҿ����� �Դϴ�. " << Left_N_x << endl;
					break;
				}
			}
			break;

		}
		else // ���Ӽ� �̶���
		{
			if (binary_img.at<uchar>(binary_img.rows / 2 + img_height, i) == 255)
			{

				if ((i > Left0_x + THRESHOLD) || (i < Left0_x - THRESHOLD))	continue;

				Left_N_x = i;
				break;
			}
		}

	}

	return Left_N_x;
}



int LaneDetect::find_RN_x(Mat binary_img, int Right0_x, int img_height, int THRESHOLD)
{
	int Right_N_x = Right0_x;

	// i = 7 -> 맨 오른쪽 끝점
	// �ҿ��� �� �̶���
	if (!(binary_img.at<uchar>( img_height, Right0_x) == 255)
		&& !(binary_img.at<uchar>( img_height, Right0_x) == 255)
		&& !(binary_img.at<uchar>( img_height, Right0_x + 1) == 255)
		&& !(binary_img.at<uchar>( img_height, Right0_x + 2) == 255)
		&& !(binary_img.at<uchar>( img_height, Right0_x + 3) == 255)
		&& !(binary_img.at<uchar>( img_height, Right0_x + 4) == 255)
		&& !(binary_img.at<uchar>( img_height, Right0_x + 5) == 255)
		&& !(binary_img.at<uchar>( img_height, Right0_x + 6) == 255)
		&& !(binary_img.at<uchar>( img_height, Right0_x + 7) == 255)
		&& !(binary_img.at<uchar>( img_height, Right0_x - 1) == 255)
		&& !(binary_img.at<uchar>( img_height, Right0_x - 2) == 255)
		&& !(binary_img.at<uchar>( img_height, Right0_x - 3) == 255)
		&& !(binary_img.at<uchar>( img_height, Right0_x - 4) == 255)
		&& !(binary_img.at<uchar>( img_height, Right0_x - 5) == 255)
		&& !(binary_img.at<uchar>( img_height, Right0_x - 6) == 255)
		&& !(binary_img.at<uchar>( img_height, Right0_x - 7) == 255)
		&& !(binary_img.at<uchar>( img_height, Right0_x - 8) == 255)
		&& !(binary_img.at<uchar>( img_height, Right0_x - 9) == 255)
		&& !(binary_img.at<uchar>( img_height, Right0_x - 10) == 255)
		&& !(binary_img.at<uchar>( img_height, Right0_x - 11) == 255)
		&& !(binary_img.at<uchar>( img_height, Right0_x - 12) == 255)
		&& !(binary_img.at<uchar>( img_height, Right0_x - 13) == 255)
		&& !(binary_img.at<uchar>( img_height, Right0_x - 14) == 255)
		&& !(binary_img.at<uchar>( img_height, Right0_x - 15) == 255))
	{
		for (int i = 10; i < binary_img.cols - 17; i++)
		{
			if (binary_img.at<uchar>( img_height, binary_img.cols - i) == 255)
			{
				Right_N_x = binary_img.cols - i;
			//	cout << "������ �ҿ����� �Դϴ�. " << Right_N_x << endl;
				break;
			}
		}
	} else // ���Ӽ� �̶���
	{
		for (int i = 10; i < binary_img.cols - 17; i++)
		{
			if (binary_img.at<uchar>( img_height, binary_img.cols - i) == 255)
			{
				if ((binary_img.cols - i > Right0_x + THRESHOLD) || (binary_img.cols - i < Right0_x - THRESHOLD))	continue;

				Right_N_x = binary_img.cols - i;
				break;
			}
		}
	}

	return Right_N_x;
}
