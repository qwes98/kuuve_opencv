#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <signal.h>
#include "kuuve_parking/LineDetector.h"

using namespace std;
using namespace cv;

// ---------새로 추가된 변수들
ros::NodeHandle nh;
ros::Publisher control_pub;
ros::Subscriber image_sub;

double angle_factor = 1.0;

ackermann_msgs::AckermannDriveStamped control_msg;

int throttle = 4;

// -----------------------------

//ȭ�� resize
#define width 960/2   //960
#define length 540/2   // 540

//�� �������� ���� �Ӱ谪.
#define THRESHOLD 5  //5


#define LINE_LENGTH 25         //������ ����
#define LINE 100               //������ �Ӹ� ��ġ

#define GO_BACK_STOP_TIME 200     //  ������ 200ȸ frame �� ����

// stop Point ��ġ   (offset �� Poin2�� ������)
#define stop_x_offset 5
#define stop_y_offset 10


double avg = 0;
double sum_ = 0;
int temp = 0;
double angle;


int fps = 500;

Mat frame, gray, bi;
Mat Roi;
Mat hsv;
Mat hsv_s;
Mat a, b;
Mat frame2;

int framecount2_R = 0;
int framecount3_R = 0;

int r0_p2=0;
int r0_p3=0;

int times = 0;
int ready = 0;
bool go_back = false;
int go_back_stop_time = 0;

Point right_P2;
Point right_P3;

Point stop_Point;

LaneDetect linedetect;

Mat RotateImage(Mat img, int angle)
{
	// ���� �߽ɱ��� ȸ��
	CvPoint2D32f center = cvPoint2D32f(img.cols / 2, img.rows / 2);

	// ������ ������ġ (x,y) ���� ȸ��
	//CvPoint2D32f center = cvPoint2D32f(x, y);
	CvMat* rotation = cvCreateMat(2, 3, CV_32FC1);
	cv2DRotationMatrix(center, double(angle), 1.0, rotation);

	cvWarpAffine(&IplImage(img), &IplImage(img), rotation);

	cvReleaseMat(&rotation);

	return img;
}

void makeControlMsg(int steering, int throttle)
{
	control_msg.drive.steering_angle = steering;
	control_msg.drive.speed = throttle;
}

Mat parseRawimg(const sensor_msgs::ImageConstPtr& image)
{
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);

	Mat raw_img = cv_ptr->image;

	if (raw_img.empty()) {
		throw std::runtime_error("frame is empty!");
	}

	return raw_img;
}

void imageCallback(const sensor_msgs::ImageConstPtr& image)
{
	try{
		frame = parseRawimg(image);
	} catch(const cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return ;
	} catch(const std::runtime_error& e) {
		cerr << e.what() << endl;
	}


	int64 t1 = getTickCount();

	temp++;

	resize(frame, frame, Size(width, length));
	frame = RotateImage(frame, 270);

	cvtColor(frame, hsv, COLOR_BGR2HSV);
	cvtColor(frame, gray, COLOR_BGR2GRAY);

	vector<Mat> hsv_planes;

	split(hsv, hsv_planes);
	hsv_s = hsv_planes[1];  //s�� ����

	double bb = threshold(gray, b, 120, 255, THRESH_BINARY);   //110
	double aa = threshold(hsv_s, a, 150, 255, THRESH_BINARY);

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
		makeControlMsg(angle * angle_factor, throttle);
	}

	//������ ���Ⱒ ����
	if (go_back == true)
	{
		int dy = abs(LINE + LINE_LENGTH) - abs(LINE);
		int dx = right_P3.x - right_P2.x;   // �̰� ������ �� �ݴ��̾��� �Ѵ�.
	    angle = atan2(dx, dy) * 180 / CV_PI;

		makeControlMsg(angle * angle_factor, (-1) * throttle);
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


	control_pub.publish(control_msg);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "parking");

#if 1

	control_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("ackermann", 100);
	image_sub = nh.subscribe("/usb_cam/image_raw", 100, imageCallback);


#else
	VideoCapture cap(1);
	//cap.open("cameraimage_color_camera3.mp4");

	if (!cap.isOpened())
	{
		cout << "Not opened cap" << endl;
		return -1;
	}
#endif

	ros::spin();
	return 0;
}
