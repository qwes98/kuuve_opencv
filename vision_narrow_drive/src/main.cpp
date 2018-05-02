#include <ros/ros.h>
#include "vision_narrow_drive/NarrowDriveNode.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "vision_narrow_drive");

#if 1
	NarrowDriveNode vision_narrow_drive_node;

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
