#include "kuuve_parking/ParkingNode.h"
#include <math.h>

using namespace std;
using namespace cv;


ParkingNode::ParkingNode()
	: as_(nh_, "kuuve_parking", boost::bind(&ParkingNode::actionCallback, this, _1), false)
{
	nh_ = ros::NodeHandle("~");
	as_.start();

	control_pub = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("ackermann", 10);

	image_sub = nh_.subscribe("/parking_image_raw", 1, &ParkingNode::imageCallback, this);

	obstacle_sub = nh_.subscribe("/raw_obstacles", 1, &ParkingNode::obstacleCallback, this);

	getRosParamForInitiation();

	getRosParamForUpdate();


	r0_p2 = length - 15; // 15 is from LineDetector's find_RN_x if statement
	r0_p3 = length - 15;

}

void ParkingNode::actionCallback(const kuuve_parking::MissionPlannerGoalConstPtr& goal)
{
	cout << "kuuve_parking actionCallback called!" << endl;

	mission_start_ = true;

	ros::Rate r(10);

	while(ros::ok()) {
		if(mission_cleared_) {
			kuuve_parking::MissionPlannerResult result;
			as_.setSucceeded(result);
			mission_start_ = false;
			break;
		}
		r.sleep();	// sleep 0.1 sec
	}

}

void ParkingNode::obstacleCallback(const obstacle_detector::Obstacles data)
{
	obstacle_detector::CircleObstacle nearest_circle_obs;
	nearest_circle_obs.center.x = 100;
	for(int i = 0; i < data.circles.size(); i++) {
		if(circleObsIsInRoi(data.circles[i]) && circleObsIsNearThan(data.circles[i], nearest_circle_obs)) {
			//nearest_circle_obs.center.x = data.circles[i].center.x;
			//nearest_circle_obs.center.y = data.circles[i].center.y;
			nearest_circle_obs = data.circles[i];
		}
	}

	obstacle_detector::SegmentObstacle nearest_segment_obs;
	nearest_segment_obs.first_point.x = 100;
	for(int i = 0; i < data.segments.size(); i++) {
		if(segmentObsIsInRoi(data.segments[i]) && segmentObsIsNearThan(data.segments[i], nearest_segment_obs)) {
			nearest_segment_obs = data.segments[i];
		}
	}

	double nearest_obs_dist = calNearestObsDist(nearest_circle_obs, nearest_segment_obs);

	/*
	cout << "nearest_circle_obs: " << nearest_circle_obs.center << endl;
	cout << "nearest_segment_obs: " << nearest_segment_obs.first_point << endl;

	if(nearest_circle_obs.center.x == 100)
		cout << "nearest_circle_obs x must not be 100!!(initial value)" << endl;
	if(nearest_segment_obs.first_point.x == 100)
		cout << "nearest_segment_obs x must not be 100!!(initial value)" << endl;
		*/

	if(nearest_obs_dist < this_room_dist_thres_) // 1번 주차공간에 장애물 있음
		instant_this_room_obs = true;
	else
		instant_this_room_obs = false;
}

bool ParkingNode::circleObsIsInRoi(const obstacle_detector::CircleObstacle& obs)
{
	return (obs.center.y < 0 && obs.center.y > (-1) * obstacle_y_thres_) && (obs.center.x > 0 && obs.center.x < obstacle_x_thres_);
}

bool ParkingNode::circleObsIsNearThan(const obstacle_detector::CircleObstacle& obs1, const obstacle_detector::CircleObstacle& obs2)
{
	return sqrt(obs1.center.x * obs1.center.x + obs1.center.y * obs1.center.y) < sqrt(obs2.center.x * obs2.center.x + obs2.center.y * obs2.center.y);
}

bool ParkingNode::segmentObsIsInRoi(const obstacle_detector::SegmentObstacle& obs)
{
	geometry_msgs::Point center;
	center.x = (obs.first_point.x + obs.last_point.x) / 2;
	center.y = (obs.first_point.y + obs.last_point.y) / 2;
	return (center.y < 0 && center.y > (-1) * obstacle_y_thres_) && (center.x > 0 && center.x < obstacle_x_thres_);
}

bool ParkingNode::segmentObsIsNearThan(const obstacle_detector::SegmentObstacle& obs1, const obstacle_detector::SegmentObstacle& obs2)
{
	geometry_msgs::Point center1;
	center1.x = (obs1.first_point.x + obs1.last_point.x) / 2;
	center1.y = (obs1.first_point.y + obs1.last_point.y) / 2;

	geometry_msgs::Point center2;
	center2.x = (obs2.first_point.x + obs2.last_point.x) / 2;
	center2.y = (obs2.first_point.y + obs2.last_point.y) / 2;

	return sqrt(center1.x * center1.x + center1.y * center1.y) < sqrt(center2.x * center2.x + center2.y * center2.y);
}

double ParkingNode::calNearestObsDist(const obstacle_detector::CircleObstacle& circle_obs, const obstacle_detector::SegmentObstacle& segment_obs)
{
	double dist_from_circle = sqrt(circle_obs.center.x * circle_obs.center.x + circle_obs.center.y * circle_obs.center.y);

	
	geometry_msgs::Point center_of_segment;
	center_of_segment.x = (segment_obs.first_point.x + segment_obs.last_point.x) / 2;
	center_of_segment.y = (segment_obs.first_point.y + segment_obs.last_point.y) / 2;
	double dist_from_segment = sqrt(center_of_segment.x * center_of_segment.x + center_of_segment.y * center_of_segment.y);

	if(dist_from_circle < dist_from_segment)
		return dist_from_circle;
	else
		return dist_from_segment;
}

void ParkingNode::getRosParamForInitiation()
{
	nh_.getParam("resize_width", width);
	nh_.getParam("resize_height", length);
	nh_.getParam("steer_max_yaw_error", STEER_MAX_ANGLE_);
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
	nh_.getParam("obstacle_detect_time", OBSTACLE_DETECT_TIME);
	nh_.getParam("detect_y_offset", LINE);
	nh_.getParam("line_length", LINE_LENGTH);
	nh_.getParam("reverse_max_stay_time", REVERSE_MAX_STAY_TIME);
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
	nh_.getParam("forward_stop_x_offset", forward_stop_x_offset_);
	nh_.getParam("forward_stop_y_offset", forward_stop_y_offset_);
	// nh_.getParam("reverse_stop_x_offset_", reverse_stop_x_offset_);
	// nh_.getParam("reverse_stop_y_offset_", reverse_stop_y_offset_);
	nh_.getParam("turning_angle_thres", turning_angle_thres_);
	nh_.getParam("reverse_detect_offset", reverse_detect_offset_);
	nh_.getParam("gaussian_param", gaussian_param);
	nh_.getParam("obstacle_x_thres", obstacle_x_thres_);
	nh_.getParam("obstacle_y_thres", obstacle_y_thres_);
	nh_.getParam("this_room_dist_thres", this_room_dist_thres_);
	nh_.getParam("left_steer_factor", left_steer_factor_);

	int yaw_factor_tmp = 0;
	nh_.getParam("yaw_factor", yaw_factor_tmp);
	yaw_factor = (double)yaw_factor_tmp / 100;

	int lateral_factor_tmp = 0;
	nh_.getParam("lateral_factor", lateral_factor_tmp);
	lateral_factor = (double)lateral_factor_tmp / 100;

	int first_driving_yaw_factor_tmp = 0;
	nh_.getParam("first_driving_yaw_factor", first_driving_yaw_factor_tmp);
	first_driving_yaw_factor = (double)first_driving_yaw_factor_tmp / 100;

	// linedetect.reverse_stop_x_offset_ = reverse_stop_x_offset_;


}

int ParkingNode::calculateSteerValue(const int center_steer_control_value, const int max_steer_control_value)
{
	int steer_control_value = 0;	// for parsing double value to int
	const int steer_offset = max_steer_control_value - center_steer_control_value;  // center ~ max

	double steer_angle = yaw_factor * yaw_error + lateral_factor * lateral_error;

	double tmp_steer_value = center_steer_control_value + (steer_offset / STEER_MAX_ANGLE_) * steer_angle;

	steer_control_value = (int) (tmp_steer_value >= 0 ? tmp_steer_value + 0.5 : tmp_steer_value - 0.5);
	steer_control_value -= left_steer_factor_;	// parking use left_steer_factor_ in all steer value


	if(steer_control_value > STEER_MAX_ANGLE_) {
		steer_control_value = center_steer_control_value + steer_offset;
	}
	else if(steer_control_value < (-1) * STEER_MAX_ANGLE_) {
		steer_control_value = center_steer_control_value - steer_offset;
	}
	
	cout << "origin steer value: " << (int) (tmp_steer_value > 0 ? tmp_steer_value + 0.5 : tmp_steer_value - 0.5) << endl;
	cout << "left factor added steer value: " << steer_control_value << endl;


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
	if(mission_start_)
	{
		cout << "parking imageCallback" << endl;
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

		////////////////////////////

		// 주차노드 켜질때 불안정한것 안정화 될때까지 그냥 방향 제어만 한다.
		steady_state = steady_state + 1;
		/////////////////////////////////

		if(this_room) {
			if (yaw_error > turning_angle_thres_ && steady_state > STEADY_STATE)
			{
				obstacle_detect_time++;
				if(instant_this_room_obs && obstacle_detect_time < OBSTACLE_DETECT_TIME)
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

			if(!turning_finish_flag_ || reverse_max_stay_time > REVERSE_MAX_STAY_TIME) {
				right_P2.x = linedetect.find_RN_x(bi, r0_p2, LINE + reverse_detect_offset_, THRESHOLD);
				right_P2.y =  LINE + reverse_detect_offset_;
				r0_p2 = right_P2.x;

				right_P3.x = linedetect.find_RN_x(bi, r0_p3, LINE + reverse_detect_offset_ + LINE_LENGTH, THRESHOLD);
				right_P3.y =  LINE + reverse_detect_offset_ +  LINE_LENGTH;
				r0_p3 = right_P3.x;
			}
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
		forward_stop_point1_.x = right_P2.x - forward_stop_x_offset_ ;
		forward_stop_point1_.y = right_P2.y - forward_stop_y_offset_ ;

		forward_stop_point2_.x = right_P2.x - forward_stop_x_offset_ + 1;
		forward_stop_point2_.y = right_P2.y - forward_stop_y_offset_ ;

		forward_stop_point3_.x = right_P2.x - forward_stop_x_offset_ + 2;
		forward_stop_point3_.y = right_P2.y - forward_stop_y_offset_ ;

		if (times == 0)
		{
			if (bi.at<uchar>(forward_stop_point1_.y, forward_stop_point1_.x) == 0 
				&& bi.at<uchar>(forward_stop_point2_.y, forward_stop_point2_.x) == 0 
				&& bi.at<uchar>(forward_stop_point3_.y, forward_stop_point3_.x) == 0 
				&& ready > READY)
			{
				cout << "(1번 주차공간) 주차하고 정지하세요 " << endl;
				times = times + 1;
				steer_value = 0;
				makeControlMsg(steer_value,0);
				control_pub.publish(control_msg);
				ros::Duration(11).sleep();

				go_back = true;
				cout << "(1번 주차공간) times = " << times << endl;
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
		if(yaw_error > forward_max_turning_angle_thres && steady_state > STEADY_STATE) {
			makeControlMsg(26, throttle);
		}
		/*
		else if(steady_state < STEADY_STATE + throttle_start_time_offset) {
			steer_value = calculateSteerValue(0,26);
			makeControlMsg(steer_value, throttle + throttle_start_offset);
		}
		*/
		else {
			steer_value = calculateSteerValue(0,26);
			// makeControlMsg(steer_value * first_driving_yaw_factor, throttle);
			makeControlMsg(steer_value, throttle);
		}
		cout << "(1번 주차공간) 전진 ready : " << ready << endl;

	}
	if(go_back == true)
	{
		int dy = abs(LINE + LINE_LENGTH) - abs(LINE);
		int dx = right_P3.x - right_P2.x;
		yaw_error = atan2(dx, dy) * 180 / CV_PI;
		// 후진중 우찌 되면 정지한다.
		cout <<"(1번 주차공간) 후진중 입니다.       " << endl;
		lateral_error = right_P2.x;

		ready3++;

		if(yaw_error > reverse_max_turning_angle_thres && ready3 > READY3) {
			makeControlMsg(26, (-1)*throttle);
			turning_finish_flag_ = true;
		}
		/*
		else if(ready3 < READY3 + throttle_start_time_offset) {
			steer_value = calculateSteerValue(0,26);
			makeControlMsg(steer_value, throttle*(-1) - throttle_start_offset);
		} */
		else {
			steer_value = calculateSteerValue(0,26);
			makeControlMsg(steer_value, throttle*(-1));
		}

		if(turning_finish_flag_)
		{
			reverse_max_stay_time++;
			if(yaw_error < reverse_stop_angle_thres || mission_cleared_) {
				steer_value = 0;
				makeControlMsg(steer_value, 0);
				cout << "(1번공간 주차 미션 끝)  인코스 차선인식 노드를 켜주세요"<<endl;
				mission_cleared_ = true;
			}
		}
	}
	//ready_timer_ -> rosparam
	// go front
	/*
		if (go_back == false)
		{
			if (!(bi.at<uchar>(forward_stop_point1_.y, forward_stop_point1_.x) == 255) && readyIsFinished())	// ready_timer_ -> 처음에는 0, 프레임이 들어올수록 1씩 증가 => for 안정적
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

			cout << "forward_stop_point1_: " << forward_stop_point1_ << endl;
			circle(frame, forward_stop_point1_, 1, Scalar(0, 0, 255), 5);

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
					 if(!turning_finish_flag_ || reverse_max_stay_time > REVERSE_MAX_STAY_TIME) {
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
						 if (yaw_error > reverse_max_turning_angle_thres && ready4 > READY4)
						 {
							 if(!out_of_room2) {
								l0_p2 = r0_p2 - 10;
								l0_p3 = r0_p3 - 10;
							 }
							 out_of_room2 = true;
						 }
						 if (out_of_room2 )
						 {
							 if(reverse_max_stay_time > REVERSE_MAX_STAY_TIME) {
								left_P2.x = linedetect.find_LN_x(bi, l0_p2, LINE + reverse_detect_offset_ + 0, THRESHOLD);
								left_P2.y = LINE + reverse_detect_offset_ + 0;
								l0_p2 = left_P2.x;

								left_P3.x = linedetect.find_LN_x(bi, l0_p3, LINE + reverse_detect_offset_ + LINE_LENGTH + 0, THRESHOLD);
								left_P3.y = LINE + reverse_detect_offset_ + LINE_LENGTH + 0;
								l0_p3 = left_P3.x;
							 // cout << "left_P2" << left_P2 << endl;
							 // cout << "left_P3" << left_P3 << endl;
							 }
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
						forward_stop_point1_.x = right_P2.x - forward_stop_x_offset_ ;
						forward_stop_point1_.y = right_P2.y - forward_stop_y_offset_ ;

						forward_stop_point2_.x = right_P2.x - forward_stop_x_offset_ + 1;
						forward_stop_point2_.y = right_P2.y - forward_stop_y_offset_ ;

						forward_stop_point3_.x = right_P2.x - forward_stop_x_offset_ + 2;
						forward_stop_point3_.y = right_P2.y - forward_stop_y_offset_ ;

						if (times2 == 0)
						{
							if (bi.at<uchar>(forward_stop_point1_.y, forward_stop_point1_.x) == 0 
								&& bi.at<uchar>(forward_stop_point2_.y, forward_stop_point2_.x) == 0 
								&& bi.at<uchar>(forward_stop_point3_.y, forward_stop_point3_.x) == 0 
								&& ready2 > READY2)
							{
								cout << "(2번 주차공간) 주차하고 정지하세요 " << endl;
								times2 = times2 + 1;
								go_back2 = true;
								steer_value = 0;
								makeControlMsg(steer_value,0);
								control_pub.publish(control_msg);
								ros::Duration(11).sleep();
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
						/*
						else if(ready4 < READY4 + throttle_start_time_offset) {
							steer_value = calculateSteerValue(0,26);
							makeControlMsg(steer_value, throttle*(-1) - throttle_start_offset);
						} */
						else {
							steer_value = calculateSteerValue(0,26);
							makeControlMsg(steer_value, throttle*(-1));
						}

						if(turning_finish_flag_)
						{
							reverse_max_stay_time++;
							if(yaw_error < reverse_stop_angle_thres) {
								steer_value = 0;
								makeControlMsg(steer_value, 0);
								cout << "(2번공간 주차 미션 끝)  인코스 차선인식 노드를 켜주세요"<<endl;
								mission_cleared_ = true;
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
					cout << "(2번 주차공간)  다음 주차공간 GO :  " << endl;
				}
			}
		int64 t2 = getTickCount();

		double ms = (t2 - t1) * 1000 / getTickFrequency();
		sum_ += ms;
		avg = sum_ / temp;

		cout << "it took : " << ms << "ms." << "���� : " << avg << " �ʴ� ó�� �����Ӽ� : " << 1000 / avg << endl;

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
				if (ready_go_back && yaw_error < YAW_ERROR) {
					circle(frame, forward_stop_point1_, 1, Scalar(0, 0, 255), 5);
					circle(frame, forward_stop_point2_, 1, Scalar(0, 0, 255), 5);
					circle(frame, forward_stop_point3_, 1, Scalar(0, 0, 255), 5);
				}
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
					if (ready_go_back2 && yaw_error < YAW_ERROR) { 
						circle(frame, forward_stop_point1_, 1, Scalar(0, 0, 255), 5);
						circle(frame, forward_stop_point2_, 1, Scalar(0, 0, 255), 5);
						circle(frame, forward_stop_point3_, 1, Scalar(0, 0, 255), 5);
					}
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
