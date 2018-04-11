#include <iostream>
#include "crosswalk_stop/CrosswalkStop.h"

using namespace std;
using namespace cv;

CrosswalkStop::CrosswalkStop(const int width, const int height, const int steer_max_angle, const double stop_distance)
    : LaneDetector(width, height, steer_max_angle), STOP_DISTANCE_(stop_distance)
{}

bool CrosswalkStop::detectCrosswalk()
{
	if (!cross_detected)
	{
// TODO: fix segmentation fault
		if (firstLocationDetected() && secondLocationDetected() && thirdLocationDetected()) {
			cout << "crosswalk detected!" << endl;

            cross_detected = true;
            visualizeCircles();

            return true;
		}

		// crosswalk not detected : blue dot
		// crosswalk detected : red dot
		// show circles in frame only before cross detected
        visualizeCircles();
        // showImg();

        /*
		if(cross_detected) {
			waitKey(3000);	// wait for 3 sec
		}
        */
    }
    return false;
}

double CrosswalkStop::getStopDistance() const { return STOP_DISTANCE_; }

// roi_binary_img_.rows = 0????
bool CrosswalkStop::firstLocationDetected() const { return roi_binary_img_.at<uchar>(roi_binary_img_.rows * STOP_DISTANCE_, roi_binary_img_.cols * 3 / 8) >= binary_threshold_; }

bool CrosswalkStop::secondLocationDetected() const { return roi_binary_img_.at<uchar>(roi_binary_img_.rows * STOP_DISTANCE_, roi_binary_img_.cols * 4 / 8) >= binary_threshold_; }

bool CrosswalkStop::thirdLocationDetected() const { return roi_binary_img_.at<uchar>(roi_binary_img_.rows * STOP_DISTANCE_, roi_binary_img_.cols * 5 / 8) >= binary_threshold_; }

void CrosswalkStop::visualizeCircles() const
{
		// crosswalk not detected : blue dot
		// crosswalk detected : red dot
		// show circles in frame only before cross detected
		circle(resized_img_, Point(roi_binary_img_.cols * 3 / 8, roi_binary_img_.rows * STOP_DISTANCE_) + Point(0, roi_binary_img_.rows), 5, Scalar(255 * (1 - cross_detected), 0, 255 * cross_detected), 5);
		circle(resized_img_, Point(roi_binary_img_.cols * 4 / 8, roi_binary_img_.rows * STOP_DISTANCE_) + Point(0, roi_binary_img_.rows), 5, Scalar(255 * (1 - cross_detected), 0, 255 * cross_detected), 5);
		circle(resized_img_, Point(roi_binary_img_.cols * 5 / 8, roi_binary_img_.rows * STOP_DISTANCE_) + Point(0, roi_binary_img_.rows), 5, Scalar(255 * (1 - cross_detected), 0, 255 * cross_detected), 5);

		circle(roi_binary_img_, Point(roi_binary_img_.cols * 3 / 8, roi_binary_img_.rows * STOP_DISTANCE_) , 5, Scalar(255 * (1 - cross_detected), 0, 255 * cross_detected), 5);
		circle(roi_binary_img_, Point(roi_binary_img_.cols * 4 / 8, roi_binary_img_.rows * STOP_DISTANCE_) , 5, Scalar(255 * (1 - cross_detected), 0, 255 * cross_detected), 5);
		circle(roi_binary_img_, Point(roi_binary_img_.cols * 5 / 8, roi_binary_img_.rows * STOP_DISTANCE_) , 5, Scalar(255 * (1 - cross_detected), 0, 255 * cross_detected), 5);
}
