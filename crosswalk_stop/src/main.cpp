#include <ros/ros.h>
#include "crosswalk_stop/CrosswalkStopNode.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "crosswalk_stop");

#if 1
	CrosswalkStopNode crosswalk_stop_node;

#else
	VideoCapture cap(0);
	//cap.open("cameraimage_color_camera3.mp4");

	if (!cap.isOpened())
	{
		cout << "Not opened cap" << endl;
		return -1;
	}

	Mat frame;
	cap >> frame;
	imshow("test", frame);
#endif

	ros::spin();
	return 0;
}
