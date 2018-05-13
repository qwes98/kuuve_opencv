#include "kuuve_parking/ParkingNode.h"

using namespace std;
using namespace cv;


ParkingNode::ParkingNode()
{
	nh_ = ros::NodeHandle("~");

	control_pub = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("ackermann", 10);

	image_sub = nh_.subscribe("/usb_cam/image_raw", 1, &ParkingNode::imageCallback, this);

	getRosParamForInitiation();

	getRosParamForUpdate();


	r0_p2 = length - 15; // 15 is from LineDetector's find_RN_x if statement
	r0_p3 = length - 15;

}

void ParkingNode::getRosParamForInitiation()
{
	nh_.getParam("resize_width", width);
	nh_.getParam("resize_height", length);
	nh_.getParam("steer_max_yaw_error", STEER_MAX_ANGLE_);
	nh_.getParam("obstract_detect_x", obstract_detect_x_location);
	nh_.getParam("obstract_detect_y", obstract_detect_y_location);
	nh_.getParam("object_detect_time", OBJECT_DETECT_TIME);
	nh_.getParam("steady_state", STEADY_STATE);
	nh_.getParam("yaw_error_for_stop_point", YAW_ERROR);
	nh_.getParam("ready1", READY);
	nh_.getParam("ready2", READY2);
	nh_.getParam("ready3", READY3);
	nh_.getParam("ready4", READY4);
	nh_.getParam("next_room_time", NEXT_ROOM_TIME);
	nh_.getParam("next_room_offset", OFFSET);
	nh_.getParam("forward_max_turning_angle_thres", forward_max_turning_angle_thres);
	nh_.getParam("reverse_max_turning_angle_thres", reverse_max_turning_angle_thres);
	nh_.getParam("reverse_stop_angle_thres", reverse_stop_angle_thres);
}

void ParkingNode::getRosParamForUpdate()
{
	nh_.getParam("gray_bin_thres", gray_bin_thres);
	nh_.getParam("hsv_s_bin_thres", hsv_s_bin_thres);
	nh_.getParam("throttle", throttle);
	nh_.getParam("throttle_start_offset", throttle_start_offset);
	nh_.getParam("throttle_start_time_offset", throttle_start_time_offset);
	nh_.getParam("go_back_stop_time", GO_BACK_STOP_TIME);
	nh_.getParam("roi_top_location", roi_top_location);
	nh_.getParam("roi_bottom_location", roi_bottom_location);
	nh_.getParam("detect_y_offset", LINE);
	nh_.getParam("line_length", LINE_LENGTH);
	nh_.getParam("forward_stop_x_offset", forward_stop_x_offset_);
	nh_.getParam("forward_stop_y_offset", forward_stop_y_offset_);
	// nh_.getParam("reverse_stop_x_offset_", reverse_stop_x_offset_);
	// nh_.getParam("reverse_stop_y_offset_", reverse_stop_y_offset_);
	nh_.getParam("turning_angle_thres", turning_angle_thres_);
	nh_.getParam("reverse_detect_offset", reverse_detect_offset_);
	nh_.getParam("gaussian_param", gaussian_param);

	int yaw_factor_tmp = 0;
	nh_.getParam("yaw_factor", yaw_factor_tmp);
	yaw_factor = (double)yaw_factor_tmp / 100;

	int lateral_factor_tmp = 0;
	nh_.getParam("lateral_factor", lateral_factor_tmp);
	lateral_factor = (double)lateral_factor_tmp / 100;

	// linedetect.reverse_stop_x_offset_ = reverse_stop_x_offset_;


}

int ParkingNode::calculateSteerValue(const int center_steer_control_value, const int max_steer_control_value)
{
	int steer_control_value = 0;// for parsing double value to int
	const int steer_offset = max_steer_control_value - center_steer_control_value;  // center ~ max
	// cout << "steer_offset: " << steer_offset << endl;

	int control_value = yaw_error * yaw_factor + lateral_error * lateral_factor;
 if(control_value < STEER_MAX_ANGLE_ && control_value > (-1) * STEER_MAX_ANGLE_) {
	steer_control_value = static_cast<int>(center_steer_control_value + steer_offset / STEER_MAX_ANGLE_ * (yaw_error * yaw_factor) + lateral_error * lateral_factor);
	cout << "steer correct" << endl;
 }
else if(control_value >= STEER_MAX_ANGLE_) {
	steer_control_value = center_steer_control_value + steer_offset;
	yaw_error = STEER_MAX_ANGLE_ / yaw_factor;// 		for print yaw_error on console
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
	cout << "steering yaw_error: " << steer_value << endl;
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

/*
bool ParkingNode::readyIsFinished()
{
	return ready_timer_ > 100;
}
*/

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

	cvtColor(frame, hsv, COLOR_BGR2HSV); // 사물 디택트
	cvtColor(frame, gray, COLOR_BGR2GRAY);

  // 사물 디택트
	vector<Mat> hsv_planes;

	split(hsv, hsv_planes);
	hsv_s = hsv_planes[1];
	// 사물 디택트

	// GaussianBlur(gray, gray, Size(0,0), gaussian_param);
	medianBlur(gray, gray, 3);

	double bb = threshold(gray, b, gray_bin_thres, 255, THRESH_BINARY);   //110
	double aa = threshold(hsv_s, a, hsv_s_bin_thres, 255, THRESH_BINARY);

	bi = b;
  bi_object = a + b; // 사물 디택트를 위함.

  //사물 디택트 위치.
	obstract.x = bi.cols * obstract_detect_x_location / 100 ;
	obstract.y = bi.rows * obstract_detect_y_location / 100 ;
	////////////////////////////

	// 주차노드 켜질때 불안정한것 안정화 될때까지 그냥 방향 제어만 한다.
	steady_state = steady_state + 1;
	/////////////////////////////////

	if (this_room == true)
	{
		if (yaw_error > turning_angle_thres_ && steady_state > STEADY_STATE)
		{
			object_detect_time = object_detect_time + 1;

			if(bi_object.at<uchar>(obstract.y, obstract.x) = 255 && object_detect_time < OBJECT_DETECT_TIME)
			{
				this_room = false;
			}
		}
	}




	if (this_room == true)
	{
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

if(ready_go_back == false)
{
	if (yaw_error > turning_angle_thres_ && steady_state > STEADY_STATE )
		{
			ready_go_back = true;
		}
}
if(ready_go_back && yaw_error < YAW_ERROR)
{
	forward_stop_point_.x = right_P2.x - forward_stop_x_offset_ ;
	forward_stop_point_.y = right_P2.y - forward_stop_y_offset_ ;
	if (times == 0)
	{
		if (bi.at<uchar>(forward_stop_point_.y, forward_stop_point_.x) == 0 && ready > READY)
		{
			cout << "(1번 주차공간) 주차하고 정지하세요 " << endl;
			times = times + 1;
			steer_value = 0;
			makeControlMsg(steer_value,0);
			ros::Duration(3).sleep();

			go_back = true;
			cout << "(1번 주차공간) times = " << times << "   조향각 : " << yaw_error << endl;
			////////// 여기에 후진 명령 내려야 한다.
		}
		ready = ready + 1;
	}
}
//전진시 조향각 계산
if(go_back == false)
{
	int dy = abs(LINE + LINE_LENGTH) - abs(LINE);
	int dx = right_P2.x - right_P3.x;
	yaw_error = atan2(dx, dy) * 180  /CV_PI;
	lateral_error = right_P2.x;
	cout << "!!!!!!!!!! yaw_error: " << yaw_error << " max_turing:  " << forward_max_turning_angle_thres << endl;
	if(yaw_error > forward_max_turning_angle_thres && steady_state > STEADY_STATE) {
		makeControlMsg(26, throttle);
	}
	else if(steady_state < STEADY_STATE + throttle_start_time_offset) {
		steer_value = calculateSteerValue(0,26);
		makeControlMsg(steer_value, throttle + throttle_start_offset);
	}
	else {
		steer_value = calculateSteerValue(0,26);
		makeControlMsg(steer_value, throttle);
	}
	cout << "(1번 주차공간) 전진 ready : " << ready << "    조향각 : " << yaw_error << endl;

}
if(go_back == true)
{
	int dy = abs(LINE + LINE_LENGTH) - abs(LINE);
	int dx = right_P3.x - right_P2.x;
	yaw_error = atan2(dx, dy) * 180 / CV_PI;
	// 후진중 우찌 되면 정지한다.
  // LINE = 400 ; // 이거 바꿔야 한다.
	cout <<"(1번 주차공간) 후진중 입니다.       " << "조향각 : " << yaw_error << endl;
	lateral_error = right_P2.x;

	ready3++;

	if(yaw_error > reverse_max_turning_angle_thres && ready3 > READY3) {
		makeControlMsg(26, (-1)*throttle);
		turning_finish_flag_ = true;
	}
	else if(ready3 < READY3 + throttle_start_time_offset) {
		steer_value = calculateSteerValue(0,26);
		makeControlMsg(steer_value, throttle*(-1) - throttle_start_offset);
	} else {
		steer_value = calculateSteerValue(0,26);
		makeControlMsg(steer_value, throttle*(-1));
	}

	if(turning_finish_flag_)
	{
		if(yaw_error < reverse_stop_angle_thres) {
			steer_value = 0;
			makeControlMsg(steer_value, 0);
			cout << "(1번공간 주차 미션 끝)  인코스 차선인식 노드를 켜주세요"<<endl;
		}
	}
}
//ready_timer_ -> rosparam
// go front
/*
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
   */
 /*
	if (go_back == true)
	{
		// 점을 조금 밑으로 내림
		/*
		right_P2.y = bi.rows / 2 + 10;
		right_P3.y = bi.rows / 2 + 10 + LINE_LENGTH;
		*
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
			reverse_stop_point_.x = right_P2.x - reverse_stop_x_offset__ ;
			reverse_stop_point_.y = right_P2.y - reverse_stop_y_offset__ ;

			circle(frame, reverse_stop_point_, 1, Scalar(0, 0, 255), 5);

			makeControlMsg(26, (-1) * throttle);

			if (bi.at<uchar>(reverse_stop_point_.y, reverse_stop_point_.x) == 255) {
				steer_value = 0;
				makeControlMsg(steer_value, 0);
			}

		}
	}
*/
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
	else
	{
		next_room_time = next_room_time + 1;
		if (next_room_time > NEXT_ROOM_TIME)
		{
			if (framecount_new_2_R < 1)	r0_p2 = linedetect.find_next_R0_x(bi, LINE, &framecount_new_2_R, r0_p2, r0_p2 + OFFSET);
			if (framecount_new_3_R < 1)	r0_p3 = linedetect.find_next_R0_x(bi, LINE + LINE_LENGTH, &framecount_new_3_R, r0_p3, r0_p3 + OFFSET);

	       if(!go_back2)
			 {
				right_P2.x = linedetect.find_RN_x(bi, r0_p2, LINE, THRESHOLD);
				right_P2.y = LINE;
				r0_p2 = right_P2.x;

				right_P3.x = linedetect.find_RN_x(bi, r0_p3, LINE + LINE_LENGTH, THRESHOLD);
				right_P3.y = LINE + LINE_LENGTH;
				r0_p3 = right_P3.x;
			 }
			 else
			 {
				 if(framecount_new_new_2_L < 1) 
				 {
				 	l0_p2 = r0_p2 - 10;
					l0_p2 = linedetect.find_L0_x2(bi, LINE + reverse_detect_offset_, &framecount_new_new_2_L, l0_p2);
				 }
				 if(framecount_new_new_3_L < 1) 
				 {
				 	l0_p3 = r0_p3 - 10;
					l0_p3 = linedetect.find_L0_x2(bi, LINE + reverse_detect_offset_ + LINE_LENGTH, &framecount_new_new_3_L, l0_p3);
				 }
				 if (yaw_error > turning_angle_thres_ && ready4 > READY4)
				 {
					 if(!out_of_room2) {
						l0_p2 = r0_p2 - 10;
						l0_p3 = r0_p3 - 10;
					 }
					 out_of_room2 = true;
				 }
				 if (out_of_room2)
				 { 
					left_P2.x = linedetect.find_LN_x(bi, l0_p2, LINE + reverse_detect_offset_ + 10, THRESHOLD);
					left_P2.y = LINE + reverse_detect_offset_ + 10;
					l0_p2 = left_P2.x;

					left_P3.x = linedetect.find_LN_x(bi, l0_p3, LINE + reverse_detect_offset_ + LINE_LENGTH + 10, THRESHOLD);
					left_P3.y = LINE + reverse_detect_offset_ + LINE_LENGTH + 10;
					l0_p3 = left_P3.x;
					 // cout << "left_P2" << left_P2 << endl;
					 // cout << "left_P3" << left_P3 << endl;
				 }
/////////////
//
				else {
				if (framecount_new_new_2_R < 1)	r0_p2 = linedetect.find_next_R0_x(bi, LINE + reverse_detect_offset_, &framecount_new_new_2_R, l0_p2, l0_p2 + OFFSET);
				if (framecount_new_new_3_R < 1)	r0_p3 = linedetect.find_next_R0_x(bi, LINE + reverse_detect_offset_ + LINE_LENGTH, &framecount_new_new_3_R, l0_p3, l0_p3 + OFFSET);

				right_P2.x = linedetect.find_RN_x(bi, r0_p2, LINE + reverse_detect_offset_, THRESHOLD);
				  right_P2.y =  LINE + reverse_detect_offset_;
				r0_p2 = right_P2.x;

				right_P3.x = linedetect.find_RN_x(bi, r0_p3, LINE + reverse_detect_offset_ + LINE_LENGTH, THRESHOLD);
				right_P3.y =  LINE + reverse_detect_offset_ +  LINE_LENGTH;
				r0_p3 = right_P3.x;
				///////
				 }
			 }

				if (ready_go_back2 == false)
				{
					if (yaw_error > turning_angle_thres_)
					{
						 ready_go_back2 = true;
					}
				}

				if (ready_go_back2 && yaw_error < YAW_ERROR)
				{
					forward_stop_point_.x = right_P2.x - forward_stop_x_offset_;
					forward_stop_point_.y = right_P2.y - forward_stop_y_offset_;


					if (times2 == 0)
					{
						if (!(bi.at<uchar>(forward_stop_point_.y, forward_stop_point_.x) == 255) && ready2 > READY2)
						{
							cout << "(2번 주차공간) 주차하고 정지하세요 " << endl;
							times2 = times2 + 1;
							go_back2 = true;
							steer_value = 0;
							makeControlMsg(steer_value,0);
							ros::Duration(3).sleep();
							cout << "(2번 주차공간)  times = " << times2 << "   조향각 : " << yaw_error << endl;
						}
						ready2 = ready2 + 1;

					}
				}
				//������ ���Ⱒ ����
				if (go_back2 == false)
				{
					int dy = abs(LINE + LINE_LENGTH) - abs(LINE);
					int dx = right_P2.x - right_P3.x;
					yaw_error = atan2(dx, dy) * 180  /CV_PI;
					lateral_error = right_P2.x;
					if(yaw_error > forward_max_turning_angle_thres && steady_state > STEADY_STATE) {
						makeControlMsg(26, throttle);
					}
					else {
						steer_value = calculateSteerValue(0,26);
						makeControlMsg(steer_value, throttle);
					}
					cout << "(2번 주차공간)  전진 ready2 " << ready2 << "   조향각 : " << yaw_error << endl;
					ready2 = ready2 + 1;
				}

				//������ ���Ⱒ ����
				if (go_back2 == true)
				{
					int dy = abs(LINE + LINE_LENGTH) - abs(LINE);
					int dx = 0;
					if(out_of_room2)
						dx = left_P3.x - left_P2.x;   // �̰� ������ �� �ݴ��̾��� �Ѵ�.
					else
						dx = right_P3.x - right_P2.x;   // �̰� ������ �� �ݴ��̾��� �Ѵ�.
					yaw_error = atan2(dx, dy) * 180 / CV_PI;
				  // LINE = 400;
					lateral_error = right_P2.x;

					cout << "(2번 주차공간) 후진중 입니다.          " << "조향각  :" << yaw_error << endl;
					//������ �����ð� �ϸ� ����

					ready4++;

					if(yaw_error > reverse_max_turning_angle_thres && ready4 > READY4) {
						makeControlMsg(26, (-1)*throttle);
						turning_finish_flag_ = true;
					}
					else if(ready4 < READY4 + throttle_start_time_offset) {
						steer_value = calculateSteerValue(0,26);
						makeControlMsg(steer_value, throttle*(-1) - throttle_start_offset);
					} else {
						steer_value = calculateSteerValue(0,26);
						makeControlMsg(steer_value, throttle*(-1));
					}

					if(turning_finish_flag_)
					{
						if(yaw_error < reverse_stop_angle_thres) {
							steer_value = 0;
							makeControlMsg(steer_value, 0);
							cout << "(2번공간 주차 미션 끝)  인코스 차선인식 노드를 켜주세요"<<endl;
						}
					}

				}
			}
			else
			{
				l0_p2 = r0_p2;
				l0_p3 = r0_p3;

				if (framecount_new_2_L <1)	r0_p2 = linedetect.find_L0_x(bi, LINE, &framecount_new_2_L, l0_p2, l0_p2 - OFFSET);
				if (framecount_new_3_L <1)	r0_p3 = linedetect.find_L0_x(bi, LINE + LINE_LENGTH, &framecount_new_3_L, l0_p3, l0_p3 - OFFSET);

				left_P2.x = linedetect.find_LN_x(bi, l0_p2, LINE, THRESHOLD);
				left_P2.y = LINE;
				l0_p2 = left_P2.x;
				r0_p2 = l0_p2;

				left_P3.x = linedetect.find_LN_x(bi, l0_p3, LINE + LINE_LENGTH, THRESHOLD);
				left_P3.y = LINE + LINE_LENGTH;
				l0_p3 = left_P3.x;
				r0_p3 = l0_p3;

			

				int dy = abs(LINE + LINE_LENGTH) - abs(LINE);
				int dx = left_P2.x - left_P3.x;
				yaw_error = atan2(dx, dy) * 180 / CV_PI;
				steer_value = calculateSteerValue(0,26);
				makeControlMsg(steer_value, throttle);
				cout << "(2번 주차공간)  다음 주차공간 GO :  " << next_room_time << "   조향각 : " << yaw_error<< endl;
			}
		}
	int64 t2 = getTickCount();

	double ms = (t2 - t1) * 1000 / getTickFrequency();
	sum_ += ms;
	avg = sum_ / temp;

	cout << "it took : " << ms << "ms." << "���� : " << avg << " �ʴ� ó�� �����Ӽ� : " << 1000 / avg << "  ���Ⱒ : " << yaw_error << endl;

/*
	line(frame, right_P2 , right_P3 , Scalar(0, 255, 0), 5);

	imshow("gray_binary", b);
	// imshow("hsv_s_binary", a);
	imshow("binary img", bi);
	imshow("frame", frame);
*/

		if (this_room == true)
		{
			line(frame, right_P2, right_P3, Scalar(0, 255, 0), 5);
			if (ready_go_back && yaw_error < YAW_ERROR) circle(frame, forward_stop_point_, 1, Scalar(0, 0, 255), 5);
			circle(frame, obstract, 1, Scalar(255, 0, 0), 5);
			imshow("gray_binary", b);
			imshow("hsv_s_binary", a);
			imshow("binary img", bi);
			imshow("frame", frame);
			//imshow("roi", Roi);
		}
		else
		{
			if (next_room_time > NEXT_ROOM_TIME)
			{
				if (ready_go_back2 && yaw_error < YAW_ERROR) circle(frame, forward_stop_point_, 1, Scalar(0, 0, 255), 5);
				if(out_of_room2)
					line(frame, left_P2, left_P3, Scalar(0, 255, 0), 5);
				else
					line(frame, right_P2, right_P3, Scalar(0, 255, 0), 5);
				circle(frame, obstract, 1, Scalar(255, 0, 0), 5);
				imshow("gray_binary", b);
				imshow("hsv_s_binary", a);
				imshow("binary img", bi);
				imshow("frame", frame);
				//imshow("roi", Roi);
			}
			else
			{
				line(frame, left_P2, left_P3, Scalar(0, 255, 0), 5);
				circle(frame, obstract, 1, Scalar(255, 0, 0), 5);
				imshow("gray_binary", b);
				imshow("hsv_s_binary", a);
				imshow("binary img", bi);
				imshow("frame", frame);
				//imshow("roi", Roi);
			}
		}

	waitKey(3);

	//printData();

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
