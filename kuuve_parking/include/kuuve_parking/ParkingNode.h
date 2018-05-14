#ifndef PARKINGNODE_H
#define PARKINGNODE_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <signal.h>
#include <actionlib/server/simple_action_server.h>
#include <memory>
#include <opencv2/opencv.hpp>
#include "kuuve_parking/LineDetector.h"
#include "lane_detector/ConditionalCompile.h"
#include "kuuve_parking/MissionPlannerAction.h"

class ParkingNode
{
public:
	ParkingNode();

	void actionCallback(const kuuve_parking::MissionPlannerGoalConstPtr& goal);

	void imageCallback(const sensor_msgs::ImageConstPtr& image);

private:
	void printData();

	void getRosParamForInitiation();

	void getRosParamForUpdate();

	void getRoiFrame();

	cv::Mat parseRawimg(const sensor_msgs::ImageConstPtr& image);

  int calculateSteerValue(const int center_steer_control_value, const int max_steer_control_value);

	void makeControlMsg(int steering, int throttle);
	// void printData();

private:
	//bool readyIsFinished();

private:
	ros::NodeHandle nh_;
	ros::Publisher control_pub;
	ros::Subscriber image_sub;

	actionlib::SimpleActionServer<kuuve_parking::MissionPlannerAction> as_;

	bool mission_start_ = false;	// for action
  bool mission_cleared_ = false;

  double yaw_factor = 1.0;
  double lateral_factor = 1.0;

  ackermann_msgs::AckermannDriveStamped control_msg;

  int throttle = 4;
  int throttle_start_offset = 3;
  int throttle_start_time_offset = 3;
  int steer_value;

  int STEER_MAX_ANGLE_ = 26;

  int gray_bin_thres = 180;
  int hsv_s_bin_thres = 180;

  // 0 ~ 100
  int roi_top_location = 0;
  int roi_bottom_location = 100;

	int turning_angle_thres_ = 20;
	bool turning_finish_flag_ = false;


  //ȭ�� resize
  int width = 960/1.5;   //960
  int length = 540/1.5;  // 540

  //�� �������� ���� �Ӱ谪.
	// 한 직선으로 보는 쓰레숄드.
  int THRESHOLD = 15;  //5


  int LINE_LENGTH = 25;         //������ ����
  int LINE = 100;               //������ �Ӹ� ��ġ

  int GO_BACK_STOP_TIME = 200;     //  ������ 200ȸ frame �� ����

  int forward_stop_x_offset_ = 5;
  int forward_stop_y_offset_ = 5;
  //int reverse_stop_x_offset_ = 10;
  //int reverse_stop_y_offset_ = 10;

  int reverse_detect_offset_ = 100;
///////////////////////////////////////////////////
	int obstract_detect_x_location = 80; // 퍼센트이다. 80퍼면 화면의 가로길이의 80퍼의 위치
  int obstract_detect_y_location = 7 ; // 단위는 퍼센트이다.
	int object_detect_time = 0;
	int OBJECT_DETECT_TIME = 10;
  int steady_state = 0; // 0부터 계속 올라간다.
  bool this_room = true; // 일단은 1번 주차공간에 넣는다고 가정한다.
	bool ready_go_back = false;
	bool ready_go_back2 = false;
	bool go_back = false;
	bool go_back2 = false;
  int STEADY_STATE = 50;
	int YAW_ERROR = 7;
	int times = 0;
	int times2 = 0;
	int ready = 0;
	int ready2 = 0;
	int ready3 = 0;
	int ready4 = 0;
	int READY = 10;
	int READY2 = 10;
	int READY3 = 10;
	int READY4 = 10;
	int next_room_time = 0;
	int NEXT_ROOM_TIME = 30;
	int framecount_new_2_L = 0;
	int framecount_new_3_L = 0;
	int framecount_new_2_R = 0;
	int framecount_new_3_R = 0;
	int OFFSET = 40; // next_room_offset
	cv::Point obstract;

	int gaussian_param = 1;

	bool during_reverse_1 = false;
	bool during_reverse_2 = false;

	bool out_of_room2 = false;

	int framecount_new_new_2_L = 0;
	int framecount_new_new_3_L = 0;
	int framecount_new_new_2_R = 0;
	int framecount_new_new_3_R = 0;
/////////////////////////////////////////////

	int forward_max_turning_angle_thres = 5;
	int reverse_max_turning_angle_thres = 5;
	int reverse_stop_angle_thres = 10;

  double avg = 0;
  double sum_ = 0;
  int temp = 0;
  double yaw_error = 0; // 0으로 초기화했음
  double lateral_error;


  int fps = 500;

  cv::Mat tmp_frame;
  cv::Mat frame, gray, bi, bi_object;
  cv::Mat Roi;
  cv::Mat hsv;
  cv::Mat hsv_s;
  cv::Mat a, b;
  cv::Mat frame2;

  int framecount2_R = 0;
  int framecount3_R = 0;

  int r0_p2;
  int r0_p3;
int l0_p2=15;
int l0_p3=15;

  int ready_timer_ = 0;
  int go_back_stop_time = 0;

  cv::Point left_P2;
	cv::Point left_P3;

  cv::Point right_P2;
  cv::Point right_P3;

  cv::Point forward_stop_point_;
	//cv::Point reverse_stop_point_;

  LaneDetect linedetect;

};

#endif
