#include "kuuve_parking/ParkingNode.h"

using namespace std;
using namespace cv;


ParkingNode::ParkingNode()
{
	nh_ = ros::NodeHandle("~");

	control_pub = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("ackermann", 100);

	image_sub = nh_.subscribe("/usb_cam/image_raw", 100, &ParkingNode::imageCallback, this);

	getRosParamForInitiation();

	getRosParamForUpdate();

	
	r0_p2 = length - 10; // 15 is from LineDetector's find_RN_x if statement
	r0_p3 = length - 10;

}

void ParkingNode::getRosParamForInitiation()
{
	nh_.getParam("resize_width", width);
	nh_.getParam("resize_height", length);
	nh_.getParam("steer_max_angle", STEER_MAX_ANGLE_);
}

void ParkingNode::getRosParamForUpdate()
{
	nh_.getParam("gray_bin_thres", gray_bin_thres);
	nh_.getParam("hsv_s_bin_thres", hsv_s_bin_thres);
	nh_.getParam("throttle", throttle);
	nh_.getParam("go_back_stop_time", GO_BACK_STOP_TIME);
	nh_.getParam("roi_top_location", roi_top_location);
	nh_.getParam("roi_bottom_location", roi_bottom_location);
	nh_.getParam("detect_y_offset", LINE);
	nh_.getParam("line_length", LINE_LENGTH);
	nh_.getParam("forward_stop_x_offset", forward_stop_x_offset_);
	nh_.getParam("forward_stop_y_offset", forward_stop_y_offset_);
	nh_.getParam("reverse_stop_x_offset", reverse_stop_x_offset_);
	nh_.getParam("reverse_stop_y_offset", reverse_stop_y_offset_);
	nh_.getParam("turning_angle_thres", turning_angle_thres_);
	nh_.getParam("reverse_detect_offset", reverse_detect_offset_);

	int yaw_factor_tmp = 0;
	nh_.getParam("yaw_factor", yaw_factor_tmp);
	yaw_factor = (double)yaw_factor_tmp / 100;

	int lateral_factor_tmp = 0;
	nh_.getParam("lateral_factor", lateral_factor_tmp);
	lateral_factor = (double)lateral_factor_tmp / 100;

	linedetect.reverse_stop_x_offset_ = reverse_stop_x_offset_;


}

int ParkingNode::calculateSteerValue(const int center_steer_control_value, const int max_steer_control_value)
{
	int steer_control_value = 0;// for parsing double value to int
	const int steer_offset = max_steer_control_value - center_steer_control_value;  // center ~ max
	cout << "steer_offset: " << steer_offset << endl;

	int control_value = yaw_error * yaw_factor + lateral_error * lateral_factor;
 if(control_value < STEER_MAX_ANGLE_ && control_value > (-1) * STEER_MAX_ANGLE_) {
	steer_control_value = static_cast<int>(center_steer_control_value + steer_offset / STEER_MAX_ANGLE_ * (yaw_error * yaw_factor) + lateral_error * lateral_factor);
	cout << "steer correct" << endl;
 }
else if(control_value >= STEER_MAX_ANGLE_) {
	steer_control_value = center_steer_control_value + steer_offset;
	yaw_error = STEER_MAX_ANGLE_ / yaw_factor;// 		for print angle on console
}
else if(control_value <= (-1) * STEER_MAX_ANGLE_) {
		steer_control_value = center_steer_control_value - steer_offset;
		yaw_error = (-1) * STEER_MAX_ANGLE_ / yaw_factor;
  }

	return steer_control_value;
}

void ParkingNode::getRoiFrame()
{
	int roi_left_top_y = tmp_frame.rows * (double)roi_top_location / 100;
	int roi_img_height = tmp_frame.rows - tmp_frame.rows * (double)(roi_top_location + (100 - roi_bottom_location)) / 100;

	tmp_frame = tmp_frame(Rect(0, roi_left_top_y, tmp_frame.cols, roi_img_height));
}

void ParkingNode::printData()
{
	cout << "#### Control ####" << endl;
	cout << "steering angle: " << steer_value << endl;
	cout << "throttle: " << throttle << endl;
	cout << "#### Ros Param ####" << endl;
	cout << "gray_bin_thres: " << gray_bin_thres << endl;
	cout << "hsv_s_bin_thres: " << hsv_s_bin_thres << endl;
	cout << "detect_y_offset: " << LINE << endl;
	cout << "line_length: " << LINE_LENGTH << endl;
	cout << "yaw_factor: " << yaw_factor * 100 << "% -> " << yaw_factor << endl;
	cout << "lateral_factor: " << lateral_factor * 100 << "% -> " << lateral_factor << endl;
	cout << "---------------------------------" << endl;
}

bool ParkingNode::readyIsFinished()
{
	return ready_timer_ > 100;
}

void ParkingNode::imageCallback(const sensor_msgs::ImageConstPtr& image)
{
	try{
		tmp_frame = parseRawimg(image);
	} catch(const cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return ;
	} catch(const std::runtime_error& e) {
		cerr << e.what() << endl;
	}

  getRosParamForUpdate();


	int64 t1 = getTickCount();

	temp++;

	resize(tmp_frame, tmp_frame, Size(width, length));
	getRoiFrame();
	rotate(tmp_frame, frame, cv::ROTATE_90_CLOCKWISE);

	// cvtColor(frame, hsv, COLOR_BGR2HSV);
	cvtColor(frame, gray, COLOR_BGR2GRAY);

/*
	vector<Mat> hsv_planes;

	split(hsv, hsv_planes);
	hsv_s = hsv_planes[1];  //s�� ����
	*/

	double bb = threshold(gray, b, gray_bin_thres, 255, THRESH_BINARY);   //110
	// double aa = threshold(hsv_s, a, hsv_s_bin_thres, 255, THRESH_BINARY);

	bi = b; // bgr, hsv ����ȭ �Ȱ� ��ġ��

	if (framecount2_R <1)	r0_p2 = linedetect.find_R0_x(bi, LINE, &framecount2_R , r0_p2);
	if (framecount3_R <1)	r0_p3 = linedetect.find_R0_x(bi, LINE + LINE_LENGTH, &framecount3_R , r0_p3);

	// forward
	if(!go_back) {
	right_P2.x = linedetect.find_RN_x(bi, r0_p2, LINE, THRESHOLD);
	right_P2.y =  LINE;
	r0_p2 = right_P2.x;

	right_P3.x = linedetect.find_RN_x(bi, r0_p3, LINE + LINE_LENGTH, THRESHOLD);
	right_P3.y =  LINE + LINE_LENGTH;
	r0_p3 = right_P3.x;
	
	} else {
	
	right_P2.x = linedetect.find_RN_x(bi, r0_p2, LINE + reverse_detect_offset_, THRESHOLD);
	right_P2.y =  LINE + reverse_detect_offset_;
	r0_p2 = right_P2.x;

	right_P3.x = linedetect.find_RN_x(bi, r0_p3, LINE + reverse_detect_offset_ + LINE_LENGTH, THRESHOLD);
	right_P3.y =  LINE + reverse_detect_offset_ +  LINE_LENGTH;
	r0_p3 = right_P3.x;
	}


	forward_stop_point_.x = right_P2.x - forward_stop_x_offset_ ;
	forward_stop_point_.y = right_P2.y - forward_stop_y_offset_ ;

//ready_timer_ -> rosparam
// go front
	if (go_back == false)
	{
		if (!(bi.at<uchar>(forward_stop_point_.y, forward_stop_point_.x) == 255) && readyIsFinished())	// ready_timer_ -> 처음에는 0, 프레임이 들어올수록 1씩 증가 => for 안정적
		{
			cout << " ���� ���� �ϼ��� " << endl;
			steer_value = 0;
			makeControlMsg(steer_value,0);
			go_back = true;
		}
		cout << "forward!" << endl;
		ready_timer_ = ready_timer_ + 1;

		int dy = abs(LINE + LINE_LENGTH) - abs(LINE);
		int dx = right_P2.x - right_P3.x;
		yaw_error = atan2(dx, dy) * 180 / CV_PI;
		lateral_error = right_P2.x;
		cout << "���� ready_timer_ " << ready_timer_ << "   ���Ⱒ : " << yaw_error << endl;

		steer_value = calculateSteerValue(0, 26);
		makeControlMsg(steer_value, throttle);

		cout << "forward_stop_point_: " << forward_stop_point_ << endl;
		circle(frame, forward_stop_point_, 1, Scalar(0, 0, 255), 5);

	}

	if (go_back == true)
	{
		// 점을 조금 밑으로 내림
		/*
		right_P2.y = bi.rows / 2 + 10;
		right_P3.y = bi.rows / 2 + 10 + LINE_LENGTH;
		*/
		LINE = 400;
		cout << "reverse!" << endl;

		int dy = abs(LINE + LINE_LENGTH) - abs(LINE);
		int dx = right_P3.x - right_P2.x;   // �̰� ������ �� �ݴ��̾��� �Ѵ�.
	    yaw_error = atan2(dx, dy) * 180 / CV_PI;
		lateral_error = right_P2.x;

		steer_value = calculateSteerValue(0, 26);
		makeControlMsg(steer_value, (-1) * throttle);


		if(yaw_error > turning_angle_thres_)
			turning_finish_flag_ = true;

		if(turning_finish_flag_) {
			reverse_stop_point_.x = right_P2.x - reverse_stop_x_offset_ ;
			reverse_stop_point_.y = right_P2.y - reverse_stop_y_offset_ ;

			circle(frame, reverse_stop_point_, 1, Scalar(0, 0, 255), 5);

			if (bi.at<uchar>(reverse_stop_point_.y, reverse_stop_point_.x) == 255) {
				steer_value = 0;
				makeControlMsg(steer_value, 0);
			}

		}

		/*
		go_back_stop_time = go_back_stop_time + 1;
		cout << "������ �Դϴ�.     " << "go_back_time  :" << go_back_stop_time << "     ���Ⱒ : " << yaw_error << endl;
		//������ �����ð� �ϸ� ����
		if (go_back_stop_time > GO_BACK_STOP_TIME)
		{
			// ���� ���带 ���� ���ڽ� ���� ���带 Ų��.
			// go_back_stop_time은 최종 정지하는 시점
			steer_value = 0;
			makeControlMsg(steer_value, 0);
		}
		*/
	}


	int64 t2 = getTickCount();

	double ms = (t2 - t1) * 1000 / getTickFrequency();
	sum_ += ms;
	avg = sum_ / temp;

	//cout << "it took : " << ms << "ms." << "���� : " << avg << " �ʴ� ó�� �����Ӽ� : " << 1000 / avg << "  ���Ⱒ : " << yaw_error << endl;


	line(frame, right_P2 , right_P3 , Scalar(0, 255, 0), 5);

	imshow("gray_binary", b);
	// imshow("hsv_s_binary", a);
	imshow("binary img", bi);
	imshow("frame", frame);

	waitKey(3);

	printData();

	control_pub.publish(control_msg);
}


Mat ParkingNode::parseRawimg(const sensor_msgs::ImageConstPtr& image)
{
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);

	Mat raw_img = cv_ptr->image;

	if (raw_img.empty()) {
		throw std::runtime_error("frame is empty!");
	}

	return raw_img;
}

void ParkingNode::makeControlMsg(int steering, int throttle)
{
	control_msg.drive.steering_angle = steering;
	control_msg.drive.speed = throttle;
}

// void printData()
