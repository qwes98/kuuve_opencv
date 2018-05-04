#include "kuuve_parking/ParkingNode.h"

using namespace std;
using namespace cv;


ParkingNode::ParkingNode()
{

}

void ParkingNode::getRosParamForUpdate()
{
	nh_.getParam("resize_width", width);
	nh_.getParam("resize_height", length);
	nh_.getParam("steer_max_angle", STEER_MAX_ANGLE_);
	nh_.getParam("gray_bin_thres", gray_bin_thres);
	nh_.getParam("hsv_s_bin_thres", hsv_s_bin_thres);
	nh_.getParam("angle_factor", (double)angle_factor / 100);
	nh_.getParam("throttle", throttle_);
	nh_.getParam("go_back_stop_time", GO_BACK_STOP_TIME);
}

int LaneDetector::calculateSteerValue(int angle)
{
	int steer_control_value = 0;	// for parsing double value to int

  if(angle < STEER_MAX_ANGLE_ && angle > (-1) * STEER_MAX_ANGLE_)
  		steer_control_value = static_cast<int>( yaw_error_ * yaw_factor_);
  	else if(angle >= STEER_MAX_ANGLE_) {
  		steer_control_value = STEER_MAX_ANGLE_;
  	}
  	else if(angle <= (-1) * STEER_MAX_ANGLE_) {
  		steer_control_value = (-1) * STEER_MAX_ANGLE_;
  }

	return steer_control_value;
}

void ParkingNode::imageCallback(const sensor_msgs::ImageConstPtr& image)
{
	try{
		frame = parseRawimg(image);
	} catch(const cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return ;
	} catch(const std::runtime_error& e) {
		cerr << e.what() << endl;
	}

  getRosParamForUpdate();


	int64 t1 = getTickCount();

	temp++;

	resize(frame, frame, Size(width, length));
//ADDED 05.04
	Mat roi_frame = frame(Rect(0, 100, frame.rows, frame.cols - 100))
	// frame = RotateImage(frame, 270);
	rotate(frame, roi_frame, cv::ROTATE_90_CLOCKWISE);

	cvtColor(frame, hsv, COLOR_BGR2HSV);
	cvtColor(frame, gray, COLOR_BGR2GRAY);

	vector<Mat> hsv_planes;

	split(hsv, hsv_planes);
	hsv_s = hsv_planes[1];  //s�� ����

	double bb = threshold(gray, b, gray_bin_thres, 255, THRESH_BINARY);   //110
	double aa = threshold(hsv_s, a, hsv_s_bin_thres, 255, THRESH_BINARY);

	bi = a + b; // bgr, hsv ����ȭ �Ȱ� ��ġ��

	if (framecount2_R <1)	r0_p2 = linedetect.find_R0_x(bi, LINE, &framecount2_R , r0_p2);
	if (framecount3_R <1)	r0_p3 = linedetect.find_R0_x(bi, LINE + LINE_LENGTH, &framecount3_R , r0_p3);

	right_P2.x = linedetect.find_RN_x(bi, r0_p2, LINE, THRESHOLD);
	right_P2.y =  LINE;
	r0_p2 = right_P2.x;

	right_P3.x = linedetect.find_RN_x(bi, r0_p3, LINE + LINE_LENGTH, THRESHOLD);
	right_P3.y =  LINE + LINE_LENGTH;
	r0_p3 = right_P3.x;


	stop_Point.x = right_P2.x - stop_x_offset ;
	stop_Point.y = right_P2.y - stop_y_offset ;

//ready -> rosparam
	// ���� ����
	if (times == 0)
	{
		// ready > 50 ������ �������尡 ��������
		// �ʱ� �������� �����Ǵ��� ������ ���� ����.
		// ready �� 50�� �ѱ��������� ���� �غ�. ������ �׶����� �����Ǵ�.
		if (!(bi.at<uchar>(stop_Point.y, stop_Point.x) == 255) && ready > 50)	// ready -> 처음에는 0, 프레임이 들어올수록 1씩 증가 => for 안정적
		{
			cout << " ���� ���� �ϼ��� " << endl;
			times = times + 1;
			// times == 1 -> 정지 flag
			makeControlMsg(0,0);
			go_back = true;
			// 5�ʰ� ����
			cout << "times = " << times << "   ���Ⱒ : " << angle << endl;
		}
		ready = ready + 1;

	}

	//������ ���Ⱒ ����
	if (go_back == false)
	{
		int dy = abs(LINE + LINE_LENGTH) - abs(LINE);
		int dx = right_P2.x - right_P3.x;
		angle = atan2(dx, dy) * 180 / CV_PI;
		cout << "���� ready " << ready << "   ���Ⱒ : " << angle << endl;
		makeControlMsg(calculateSteerValue(angle), throttle);
	}

	//������ ���Ⱒ ����
	if (go_back == true)
	{
		int dy = abs(LINE + LINE_LENGTH) - abs(LINE);
		int dx = right_P3.x - right_P2.x;   // �̰� ������ �� �ݴ��̾��� �Ѵ�.
	    angle = atan2(dx, dy) * 180 / CV_PI;

		makeControlMsg(calculateSteerValue(angle), (-1) * throttle);
		go_back_stop_time = go_back_stop_time + 1;
		cout << "������ �Դϴ�.     " << "go_back_time  :" << go_back_stop_time << "     ���Ⱒ : " << angle << endl;
		//������ �����ð� �ϸ� ����
		if (go_back_stop_time > GO_BACK_STOP_TIME)
		{
			// ���� ���带 ���� ���ڽ� ���� ���带 Ų��.
			// go_back_stop_time은 최종 정지하는 시점
			makeControlMsg(0, 0);
		}
	}


	int64 t2 = getTickCount();

	double ms = (t2 - t1) * 1000 / getTickFrequency();
	sum_ += ms;
	avg = sum_ / temp;

	//cout << "it took : " << ms << "ms." << "���� : " << avg << " �ʴ� ó�� �����Ӽ� : " << 1000 / avg << "  ���Ⱒ : " << angle << endl;


	line(frame, right_P2 , right_P3 , Scalar(0, 255, 0), 5);
	circle(frame, stop_Point, 1, Scalar(0, 0, 255), 5);

	imshow("gray_binary", a);
	imshow("hsv_s_binary", b);
	imshow("binary img", bi);
	imshow("frame", frame);

	waitKey(3);

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
