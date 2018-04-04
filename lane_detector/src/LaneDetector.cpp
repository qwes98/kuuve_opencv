#include "lane_detector/LaneDetector.h"
#include "lane_detector/LinePointDetector.h"

using namespace std;
using namespace cv;

LaneDetector::LaneDetector(const int width, const int height, const int steer_max_angle, const int left_img_offset, const int right_img_offset, const float full_width_rate)
  : RESIZE_WIDTH_(width), RESIZE_HEIGHT_(height), STEER_MAX_ANGLE_(steer_max_angle), LEFT_IMG_OFFSET_(left_img_offset), RIGHT_IMG_OFFSET_(right_img_offset), FULL_WIDTH_RATE_(full_width_rate), last_right_point_(0, 0), last_left_point_(0, 0)
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

/*
void LaneDetector::getRoiBinaryImg(const Point& left_top, const Size& roi_size)
{
	Mat roi_gray_img;
	Mat roi_color_img = resized_img_(Rect(left_top.x, left_top.y, roi_size.width, roi_size.height));
	cvtColor(roi_color_img, roi_gray_img, COLOR_BGR2GRAY);
	threshold(roi_gray_img, roi_binary_img_, binary_threshold_, 255, THRESH_BINARY);
}
*/

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
	line(full_img_, cur_right_point_ + Point(0, RESIZE_HEIGHT_ / 2), cur_left_point_ + Point(0, RESIZE_HEIGHT_ / 2), Scalar(0, 255, 0), 5);
	line(full_img_, lane_middle_ + Point(0, RESIZE_HEIGHT_ / 2), Point(full_img_.cols / 2, full_img_.rows), Scalar(0, 0, 255), 5);
}

void LaneDetector::showImg() const
{
	imshow("binary img", roi_binary_img_);
	imshow("full img", full_img_);
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

void LaneDetector::getConcatBinImg()
{
	// 일단 큰 화면 저장 
	// 이때 full_img_ 의 세로길이는 right_img, left_img와 같고
	// 가로길이는 left_img 와 right_img 가로길이의 합이다. 6/4(FULL_WIDTH_RATE_의 default 값) 가 full_img_ 이므로 다른애들은 3/4 이다.
	full_img_ = Mat(resized_left_img_.rows, resized_right_img_.cols * FULL_WIDTH_RATE_, CV_8UC3, Scalar(0));

	//sub_left_img -> full_img_를 반으로 자른 뒤 왼쪽 이미지만 참조하는 변수.. 이 변수에 무언가를 넣으면 full_img_의 왼쪽만 변한다.
	Mat sub_left_img(full_img_, Rect(0, 0, full_img_.cols / 2, full_img_.rows));
    Mat sub_right_img(full_img_, Rect(full_img_.cols / 2, 0, full_img_.cols / 2, full_img_.rows));

	Mat left_frame_cut;
	Mat right_frame_cut;

	// TODO: resize_left_img_.cols -> RESIZE_WIDTH_ & .rows -> RESIZE_HEIGHT_
	try {
		left_frame_cut = resized_left_img_(Rect(resized_left_img_.cols * (1 - FULL_WIDTH_RATE_ / 2) - LEFT_IMG_OFFSET_, 0, resized_left_img_.cols * FULL_WIDTH_RATE_ / 2, resized_left_img_.rows));
	} catch(const cv::Exception& e) {
		cerr << "Awesome 2018 Kuuve vision team member: 강희, 지원, 주한, 인재, 정호, 상원, 정민" << endl;
		cerr << "left image offset must be less than " << resized_left_img_.cols * (1 - FULL_WIDTH_RATE_ / 2) << " !!!!" << endl;
		cerr << "left image offset: " << LEFT_IMG_OFFSET_ << endl;
		cerr << "if you want to increase offset more, you have to decrease full image width" << endl;
	}

	try {
		right_frame_cut = resized_right_img_(Rect(RIGHT_IMG_OFFSET_, 0, resized_right_img_.cols * FULL_WIDTH_RATE_ / 2, resized_right_img_.rows));
	} catch(const cv::Exception& e) {
		cerr << "2018 Kuuve vision team member: 강희, 지원, 주한, 인재, 정호, 상원, 정민" << endl;
		cerr << "right image offset must be less than " << resized_right_img_.cols * (1 - FULL_WIDTH_RATE_ / 2) << " !!!!" << endl;
		cerr << "right image offset: " << RIGHT_IMG_OFFSET_ << endl;
		cerr << "if you want to increase offset more, you have to decrease full image width" << endl;
	}

	// 위에서 관심영역만 자른 이미지를 전체 이미지의 왼쪽을 참조하는 영상에 붙여 넣는다  
	left_frame_cut.copyTo(sub_left_img);
	right_frame_cut.copyTo(sub_right_img);

	//이제부터는 그냥 차선 인식 ( 화면의 아래쪽을 관심영역을 두고 원래 하던대로 차선 인식
	Mat full_gray;
	Mat full_Roi = full_img_(Rect(0, full_img_.rows / 2, full_img_.cols, full_img_.rows / 2));
	cvtColor(full_Roi, full_gray, COLOR_BGR2GRAY);

	double b = threshold(full_gray, roi_binary_img_, binary_threshold_, 255, THRESH_BINARY);
}

void LaneDetector::preprocessImg(const cv::Mat& raw_left_img, const cv::Mat& raw_right_img)
{
	resize(raw_left_img, resized_left_img_, Size(RESIZE_WIDTH_, RESIZE_HEIGHT_));
	resize(raw_right_img, resized_right_img_, Size(RESIZE_WIDTH_, RESIZE_HEIGHT_));

	// getRoiBinaryImg(Point(0, RESIZE_HEIGHT_ / 2), Size(RESIZE_WIDTH_, RESIZE_HEIGHT_ / 2));
	getConcatBinImg();
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

int LaneDetector::laneDetecting(const cv::Mat& raw_left_img, const cv::Mat& raw_right_img)
{
	const int64 start_time = getTickCount();
	frame_count_++;

	preprocessImg(raw_left_img, raw_right_img);

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
