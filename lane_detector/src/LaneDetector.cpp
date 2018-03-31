#include "lane_detector/LaneDetector.h"
#include "lane_detector/LinePointDetector.h"

using namespace std;
using namespace cv;

LaneDetector::LaneDetector(const int width, const int height, const int steer_max_angle)
  : RESIZE_WIDTH_(width), RESIZE_HEIGHT_(height), STEER_MAX_ANGLE_(steer_max_angle), last_right_point_(0, 0), last_left_point_(0, 0)
{}

void LaneDetector::setBinaryThres(const int bin_thres) { binary_threshold_ = bin_thres; }
void LaneDetector::setDetectYOffset(const int detect_y_offset) { detect_y_offset_ = detect_y_offset; }
void LaneDetector::setControlFactor(const double control_factor) { control_factor_ = control_factor; }

int LaneDetector::getWidth() const { return RESIZE_WIDTH_; }
int LaneDetector::getHeight() const { return RESIZE_HEIGHT_; }
int LaneDetector::getBinaryThres() const { return binary_threshold_; }
int LaneDetector::getDetectYOffset() const { return detect_y_offset_; }
int LaneDetector::getSteerMaxAngle() const { return STEER_MAX_ANGLE_; }
double LaneDetector::getControlFactor() const { return control_factor_; }
double LaneDetector::getOnceDetectTime() const { return once_detect_time_; }
double LaneDetector::getAvgDetectTime() const { return detect_avg_time_; }

void LaneDetector::getRoiBinaryImg(const Point& left_top, const Size& roi_size)
{
	Mat roi_gray_img;
	Mat roi_color_img = resized_img_(Rect(left_top.x, left_top.y, roi_size.width, roi_size.height));
	cvtColor(roi_color_img, roi_gray_img, COLOR_BGR2GRAY);
	threshold(roi_gray_img, roi_binary_img_, binary_threshold_, 255, THRESH_BINARY);
}

void LaneDetector::resetLeftPoint()
{
	last_left_point_.x = line_point_detector_.find_L0_x(roi_binary_img_, detect_y_offset_, last_left_point_.x);
	last_left_point_.y = roi_binary_img_.rows * detect_y_offset_ / 100;
}

void LaneDetector::resetRightPoint()
{
	last_right_point_.x = line_point_detector_.find_R0_x(roi_binary_img_, detect_y_offset_, last_right_point_.x);
	last_right_point_.y = roi_binary_img_.rows * detect_y_offset_ / 100;
}

void LaneDetector::updateNextPoint()
{
	cur_right_point_.x = line_point_detector_.find_RN_x(roi_binary_img_, last_right_point_.x, detect_y_offset_, LINE_PIXEL_THRESHOLD);
	cur_right_point_.y = roi_binary_img_.rows * detect_y_offset_ / 100;
	last_right_point_.x = cur_right_point_.x;

	cur_left_point_.x = line_point_detector_.find_LN_x(roi_binary_img_, last_left_point_.x, detect_y_offset_, LINE_PIXEL_THRESHOLD);
	cur_left_point_.y = roi_binary_img_.rows * detect_y_offset_ / 100;
	last_left_point_.x = cur_left_point_.x;
}

double LaneDetector::calculateAngle()
{
	const int dx = lane_middle_.x - roi_binary_img_.cols / 2;
	const int dy = roi_binary_img_.rows - lane_middle_.y;
	return atan2(dx, dy) * 180 / CV_PI;
}

void LaneDetector::calculateOnceDetectTime(const int64 start_time, const int64 finish_time)
{
	once_detect_time_ = (finish_time - start_time) * 1000 / getTickFrequency();
}

void LaneDetector::calculateAvgDetectTime()
{
	sum_of_detect_time_ += once_detect_time_;
	detect_avg_time_ = sum_of_detect_time_ / frame_count_;
}

bool LaneDetector::detectOnlyOneLine() const
{
	return abs(cur_right_point_.x - cur_left_point_.x) < 15;
}

void LaneDetector::visualizeLine() const
{
	line(resized_img_, cur_right_point_ + Point(0, RESIZE_HEIGHT_ / 2), cur_left_point_ + Point(0, RESIZE_HEIGHT_ / 2), Scalar(0, 255, 0), 5);
	line(resized_img_, lane_middle_ + Point(0, RESIZE_HEIGHT_ / 2), Point(resized_img_.cols / 2, resized_img_.rows), Scalar(0, 0, 255), 5);
}

void LaneDetector::showImg() const
{
	imshow("binary img", roi_binary_img_);
	imshow("frame", resized_img_);
	waitKey(3);
}

int LaneDetector::calculateSteerValue(const int center_steer_control_value, const int max_steer_control_value)
{
	int steer_control_value = 0;	// for parsing double value to int
  const int steer_offset = max_steer_control_value - center_steer_control_value;  // center ~ max

	if(angle_ * control_factor_ < STEER_MAX_ANGLE_ && angle_ * control_factor_ > (-1) * STEER_MAX_ANGLE_)
		steer_control_value = static_cast<int>(center_steer_control_value + steer_offset / STEER_MAX_ANGLE_ * (angle_ * control_factor_));
	else if(angle_ * control_factor_ >= STEER_MAX_ANGLE_) {
		steer_control_value = center_steer_control_value + steer_offset;
		angle_ = STEER_MAX_ANGLE_ / control_factor_;		// for print angle on console
	}
	else if(angle_ * control_factor_ <= (-1) * STEER_MAX_ANGLE_) {
		steer_control_value = center_steer_control_value - steer_offset;
		angle_ = (-1) * STEER_MAX_ANGLE_ / control_factor_;
	}

	return steer_control_value;
}

int LaneDetector::getRealSteerAngle() const { return angle_ * control_factor_; }

// for testFunction
bool LaneDetector::haveToResetLeftPoint() const { return line_point_detector_.getLeftResetFlag(); }
bool LaneDetector::haveToResetRightPoint() const { return line_point_detector_.getRightResetFlag(); }

Point LaneDetector::detectLaneCenter() const { return Point((cur_right_point_.x + cur_left_point_.x) / 2, roi_binary_img_.rows * detect_y_offset_ / 100); }

void LaneDetector::reservePointReset()
{
	line_point_detector_.setLeftResetFlag(true);
	line_point_detector_.setRightResetFlag(true);
}

void LaneDetector::preprocessImg(const cv::Mat& raw_img)
{
	resize(raw_img, resized_img_, Size(RESIZE_WIDTH_, RESIZE_HEIGHT_));

	getRoiBinaryImg(Point(0, RESIZE_HEIGHT_ / 2), Size(RESIZE_WIDTH_, RESIZE_HEIGHT_ / 2));
}

void LaneDetector::findLanePoints()
{
	if(haveToResetLeftPoint()) {
		resetLeftPoint();
	}

	if (haveToResetRightPoint()) {
		resetRightPoint();
	}

	updateNextPoint();
}

void LaneDetector::findSteering()
{
	lane_middle_ = detectLaneCenter();

	angle_ = calculateAngle();
}

void LaneDetector::calDetectingTime(const int64 start_time, const int64 finish_time)
{
	calculateOnceDetectTime(start_time, finish_time);

	calculateAvgDetectTime();
}

void LaneDetector::visualizeAll()
{
	visualizeLine();
	showImg();
}

int LaneDetector::laneDetecting(const cv::Mat& raw_img)
{
	const int64 start_time = getTickCount();
	frame_count_++;

	preprocessImg(raw_img);

	findLanePoints();

	findSteering();

	const int64 finish_time = getTickCount();

	calDetectingTime(start_time, finish_time);

	if (detectOnlyOneLine()) {
		reservePointReset();
	}

	visualizeAll();

#if RC_CAR
// arduino steering range: 1100 < steer < 1900
	return calculateSteerValue(1500, 1900);
#elif SCALE_PLATFORM
// platform steering range: -27 < steer < +27
	return calculateSteerValue(0, 27);
#endif
}
