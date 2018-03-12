/* Test */

// #pragma warning(disable: 4819)

#include "lane_detector/LineDetector.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <cmath>
#include <string>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

//È­¸é resize
#define width 960/2
#define length 540/2

//ÇÑ Á÷¼±À¸·Î º¸´Â ÀÓ°è°ª.
#define PIXEL_THRESHOLD 5  //5



// ¶óÀÎ À§Ä¡
#define LINE1 -25
#define LINE2 -30
#define LINE3 -15


class LaneDetector
{
public:
	LaneDetector();
	void imageCallback(const sensor_msgs::ImageConstPtr& image);

private:
	// Ros variables
	ros::NodeHandle nh;
	ros::Publisher control_pub;
	ros::Subscriber image_sub;


	// Could modify by using rosparam
	int lane_binary_threshold = 110;
	int control_factor = 25;
	int throttle = 1515;

	double avg = 0.0;
	int temp = 0;
	double angle = 0.0;


	int fps = 500;


	Mat frame, gray, bi;
	Mat Roi;
	Mat hsv;
	Mat hsv_s;
	Mat a, b;

	int framecount1_R = 0;
	int framecount1_L = 0;

	int framecount2_R = 0;
	int framecount2_L = 0;

	int framecount3_R = 0;
	int framecount3_L = 0;

	// Ã¹¹øÂ° ÁÂÇ¥ ( Ã¹¹øÂ° ÇÁ·¹ÀÓ )
	int r0_p1 = 0;
	int l0_p1 = 0;

	int r0_p2=0;
	int l0_p2=0;

	int r0_p3=0;
	int l0_p3=0;

	// ¿ÞÂÊÁÂÇ¥, ¿À¸¥ÂÊ ÁÂÇ¥
	Point right_P1;
	Point left_P1;

	Point right_P2;
	Point left_P2;

	Point right_P3;
	Point left_P3;

	Point middle;

	LaneDetect linedetect;

	string tmp_control_value = "";
	string tmp_throttle_value = "";
	std_msgs::String control_msg;

	double sum = 0;
};

LaneDetector::LaneDetector()
{
	nh = ros::NodeHandle("~");
	// NodeHangle("~") -> (write -> /lane_detector/write)
	control_pub = nh.advertise<std_msgs::String>("write", 100);
	image_sub = nh.subscribe("/usb_cam/image_raw", 100, &LaneDetector::imageCallback, this);
}

void LaneDetector::imageCallback(const sensor_msgs::ImageConstPtr& image)
{

//	delete these comments if you get images from camera directly
//	for (;;)
//	{
		cv_bridge::CvImagePtr cv_ptr;
		try {
			cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
		} catch(cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return ;
		}

		int64 t1 = getTickCount();

		temp++;

//		cap >> frame;
		frame = cv_ptr->image;

		resize(frame, frame, Size(width, length));


		
		if (frame.empty())
		{
			cout << "È­¸éÀÌ ºñ¾î ÀÖ¾î¿ä!!" << endl;
			return;
//			break;
		}

		nh.getParam("lane_bin_thres", lane_binary_threshold);
		nh.getParam("control_factor", control_factor);
		nh.getParam("throttle", throttle);

		// For test
		cout << "lane_bin_thres: " << lane_binary_threshold << endl;
		cout << "control_factor: " << control_factor << endl;

		Roi = frame(Rect(0, length / 2, width, length / 2));
		//cvtColor(Roi, hsv, COLOR_BGR2HSV);
		cvtColor(Roi, gray, COLOR_BGR2GRAY);
		/*
		cvtColor(Roi, hsv, COLOR_BGR2HSV);

		vector<Mat> hsv_planes;

		split(hsv, hsv_planes);
		hsv_s = hsv_planes[1];  //s¸¸ µû±â
		*/
		double bb = threshold(gray, b, lane_binary_threshold, 255, THRESH_BINARY);
		//double aa = threshold(hsv_s, a, 110, 255, THRESH_BINARY);

		//bi = a + b; // bgr, hsv ÀÌÁøÈ­ µÈ°Í ÇÕÄ¡±â 
		bi = b;


		// ¾Æ ±×³É °¥¶§´Â ÄÜÀ» Â÷¼±À¸·Î ÀÎ½ÄÇÒ ÇÊ¿ä ¾ø´Ù!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!




		/*
		ÀÌÁ¦ ³»°¡ ÇÒ ÀÏ..
		1. ÀÌÁøÈ­µÈ È­¸é¿¡ ¿Þ¼±¿¡´Â ¿ÞÂÊ Æ÷ÀÎÆ®¸¦, ¿À¸¥¼±¿¡´Â ¿À¸¥ÂÊ Æ÷ÀÎÆ®¸¦ µÐ´Ù.
		2. ¿ÞÂÊ Æ÷ÀÎÆ®, ¿À¸¥ÂÊ Æ÷ÀÎÆ® Áß½É Ã£±â
		3. ³ªÀÇ À§Ä¡¿Í ±× Æ÷ÀÎÆ® À§Ä¡ Â÷ÀÌ - > Á¶Çâ°¢ // Áß½É°ú ¿ÞÆ÷ÀÎÆ®, ¿À¸¥ÂÊ Æ÷ÀÎÆ® °Å¸® - > Á¶Çâ ½ºÇÇµå
		4. Æ÷ÀÎÆ®µéÀÇ À§Ä¡°¡ ÈÅ º¯ÇÏÁö ¾Êµµ·Ï ¿¹¿ÜÃ³¸®.
		*/
		
		// ÃÊ±âÁ¡ ±¸ÇÏ±â
		if (framecount1_L < 1) {
			l0_p1 = linedetect.find_L0_x(bi, bi.rows / 2 + LINE1, &framecount1_L , l0_p1);
			cout << "framecount1_L  " << framecount1_L << endl;
		}
		if (framecount1_R <1)	r0_p1 = linedetect.find_R0_x(bi, bi.rows /2 + LINE1, &framecount1_R , r0_p1);

		if (framecount2_L < 1) 	l0_p2 = linedetect.find_L0_x(bi, bi.rows / 2 + LINE2, &framecount2_L , l0_p2);
		if (framecount2_R <1)	r0_p2 = linedetect.find_R0_x(bi, bi.rows /2 + LINE2, &framecount2_R , r0_p2);

		if (framecount3_L < 1) 	l0_p3 = linedetect.find_L0_x(bi, bi.rows / 2 + LINE3, &framecount3_L , l0_p3);
		if (framecount3_R <1)	r0_p3 = linedetect.find_R0_x(bi, bi.rows /2 + LINE3, &framecount3_R , r0_p3);

		// Æ÷ÀÎÆ® ±¸ÇÏ±â
		right_P1.x = linedetect.find_RN_x(bi, r0_p1, LINE1, PIXEL_THRESHOLD);
		right_P1.y = bi.rows / 2 + LINE1;
		r0_p1 = right_P1.x;
		left_P1.x = linedetect.find_LN_x(bi, l0_p1, LINE1, PIXEL_THRESHOLD);
		left_P1.y = bi.rows / 2 + LINE1;
		l0_p1 = left_P1.x;
		

		right_P2.x = linedetect.find_RN_x(bi, r0_p2, LINE2, PIXEL_THRESHOLD);
		right_P2.y = bi.rows / 2 + LINE2;
		r0_p2 = right_P2.x;
		left_P2.x = linedetect.find_LN_x(bi, l0_p2, LINE2, PIXEL_THRESHOLD);
		left_P2.y = bi.rows / 2 + LINE2;
		l0_p2 = left_P2.x;

		right_P3.x = linedetect.find_RN_x(bi, r0_p3, LINE3, PIXEL_THRESHOLD);
		right_P3.y = bi.rows / 2 + LINE3;
		r0_p3 = right_P3.x;
		left_P3.x = linedetect.find_LN_x(bi, l0_p3, LINE3, PIXEL_THRESHOLD);
		left_P3.y = bi.rows / 2 + LINE3;
		l0_p3 = left_P3.x;

		middle = Point((right_P1.x + left_P1.x) / 2, bi.rows / 2 + LINE1);

		int dy = middle.x - bi.cols / 2;
		int dx = bi.rows - middle.y;
		angle = atan2(dy, dx) * 180 / CV_PI;


		int64 t2 = getTickCount();

		double ms = (t2 - t1) * 1000 / getTickFrequency();
		sum += ms;
		avg = sum / temp;

		cout << "---------------------------------" << endl;
		cout << "it took : " << ms << "ms." << "avg: " << avg << " fps : " << 1000 / avg << endl;

		// ¸¸ÀÏ µÎ Á¡ »çÀÌ °Å¸®°¡ ³Ê¹« °¡±î¿ì¸é ´Ù½Ã ÀÎ½ÄÇØ¶ó - > ÇÑ Â÷¼±À» °°ÀÌ ÀÎ½ÄÇÏ°í ÀÖÀ¸´Ï..
		if (abs(right_P1.x - left_P1.x) < 15)
		{
			framecount1_L = 0;
			framecount1_R = 0;
		}

		if (abs(right_P2.x - left_P2.x) < 15)
		{
			framecount2_L = 0;
			framecount2_R = 0;
		}

		if (abs(right_P3.x - left_P3.x) < 15)
		{
			framecount3_L = 0;
			framecount3_R = 0;
		}

		
		//line(frame, left_P + Point(0,length / 2), right_P+ Point(0, length / 2), Scalar(255, 0, 0), 2);
		line(frame, right_P1 + Point(0, length / 2), left_P1 + Point(0, length / 2), Scalar(0, 255, 0), 5);
		line(frame, right_P2 + Point(0, length / 2), left_P2 + Point(0, length / 2), Scalar(0, 0, 255), 5);
		line(frame, right_P3 + Point(0, length / 2), left_P3 + Point(0, length / 2), Scalar(255, 0, 0), 5);
		line(frame, middle + Point(0, length / 2), Point(frame.cols / 2, frame.rows), Scalar(0, 0, 255), 5);
		//circle(frame, left_P + Point(0, length / 2), 5, Scalar(255, 0, 0), 5);
		//circle(frame, right_P + Point(0, length / 2), 5, Scalar(0, 255, 0), 5); 
		imshow("binary img", bi); 
		imshow("frame", frame);
		waitKey(3);

		ROS_INFO("Angle: %f", angle);

		int angle_for_msg = 0;	// For parsing double value to int
		// arduino steering range: 1100 < steer < 1900
		if(angle < control_factor && angle > (-1) * control_factor)
			angle_for_msg = static_cast<int>(1500 + 400 / control_factor * angle);
		else if(angle >= control_factor)
			angle_for_msg = 1100;
		else if(angle <= (-1) * control_factor)
			angle_for_msg = 1900;

		tmp_control_value = to_string(angle_for_msg);
		tmp_throttle_value = to_string(throttle);
		// cout << "test angle: " << tmp_control_value << endl;

		control_msg.data = tmp_control_value + "," + tmp_throttle_value + ",";	// Make message
		cout << "control msg: " << control_msg.data << endl;

		control_pub.publish(control_msg);

		// Why does this need?
//		if (waitKey(1000 / fps) >= 0) break;


//	}

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "lane_detector");

	LaneDetector lane_detector;
	/* get images from camera directly
	VideoCapture cap(1);
	//cap.open("cameraimage_color_camera3.mp4");

	if (!cap.isOpened())
	{
		cout << "Not opened cap" << endl;
		return -1;
	}
	*/

	ros::spin();
	return 0;
}
