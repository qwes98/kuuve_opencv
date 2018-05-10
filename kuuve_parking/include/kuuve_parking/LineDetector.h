#include <opencv2/opencv.hpp>

class LaneDetect
{
	int Right0_x, Left0_x;


public:

	int reverse_stop_x_offset_ = 42;

	int find_R0_x(cv::Mat binary_img, int img_height, int *framecount_R , int Right0_x);

	int find_L0_x(cv::Mat binary_img, int img_height, int *framecount_L , int Left0_x, int offset);
  int find_next_R0_x(cv::Mat binary_img, int img_height, int *next_framecount_R, int Right0_x, int offset);

	int find_RN_x(cv::Mat binary_img, int Right0_x, int img_height, int THRESHOLD);
	int find_LN_x(cv::Mat binary_img, int Left0_x, int img_height ,int THRESHOLD);



};
