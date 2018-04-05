#ifndef CROSSWALKSTOP_H
#define CROSSWALKSTOP_H

#include "lane_detector/LaneDetector.h"

class CrosswalkStop: public LaneDetector
{
public:
    CrosswalkStop(const int width, const int height, const int steer_max_angle, const double stop_distance);

    bool detectCrosswalk();

    double getStopDistance() const;

private:
    bool firstLocationDetected() const;
    bool secondLocationDetected() const;
    bool thirdLocationDetected() const;

    void visualizeCircles() const;

private:
    bool cross_detected = false;
    const double STOP_DISTANCE_ = 0.0;
};

#endif