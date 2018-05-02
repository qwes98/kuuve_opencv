#include <ros/ros.h>
#include "vision_static_avoidance/StaticAvoidanceNode.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "vision_static_avoidance");

#if 1
	StaticAvoidanceNode vision_static_avoidance_node;

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
