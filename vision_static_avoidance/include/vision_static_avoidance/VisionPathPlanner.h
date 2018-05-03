#ifndef VISIONPATHPLANNER_H
#define VISIONPATHPLANNER_H

#include "lane_detector/InToOutLaneDetector.h"

class VisionPathPlanner: public InToOutLaneDetector
{
public:
//TODO
  VisionPathPlanner(const int width, const int height, const int steer_max_angle, const int detect_line_count, const int sustaining_time);

  // void setSustainingTime(const int sustaining_time);
  void setChangePixelThres(const int change_pixel_thres);
  void setTimeAfterDetectObs(const int time_after_detect_obstacle, const int index) throw(my_out_of_range);

  int getSustainingTime() const;
  int getChangePixelThres() const;
  int getTimeAfterDetectObs(const int index) const throw(my_out_of_range);

protected:
  virtual cv::Point detectLaneCenter(const int index) throw(my_out_of_range);

protected:
  int sustaining_time_ = 40;
  int change_pixel_thres_ = 18;
  std::unique_ptr<int[]> time_after_detect_obstacle_arr_;
  std::unique_ptr<cv::Point[]> last_lane_middle_arr_;

  std::unique_ptr<bool[]> start_flag_arr_;
  std::unique_ptr<bool[]> sustaining_arr_;
};

#endif
