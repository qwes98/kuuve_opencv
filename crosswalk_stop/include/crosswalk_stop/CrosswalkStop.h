#ifndef CROSSWALKSTOP_H
#define CROSSWALKSTOP_H

#include "lane_detector/LaneDetector.h"

class CrosswalkStop: public LaneDetector
{
public:
    CrosswalkStop(const int width, const int height, const int steer_max_angle, const double stop_distance, const int stop_time);

    bool detectCrosswalk();

    double getStopDistance() const;
    int getStopTime() const;

private:
    bool firstLocationDetected() const;
    bool secondLocationDetected() const;
    bool thirdLocationDetected() const;

    void visualizeCircles() const;

private:
    bool cross_detected_ = false;
    const double STOP_DISTANCE_ = 0.0;
    const int STOP_TIME_ = 3; // seconds
};

#endif
