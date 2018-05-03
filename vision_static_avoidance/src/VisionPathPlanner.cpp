#include "vision_static_avoidance/VisionPathPlanner.h"

using namespace std;
using namespace cv;

VisionPathPlanner::VisionPathPlanner(const int width, const int height, const int steer_max_angle, const int detect_line_count, const int sustaining_time)
  : InToOutLaneDetector(width, height, steer_max_angle, detect_line_count), sustaining_time_(sustaining_time)
{
  time_after_detect_obstacle_arr_ = unique_ptr<int[]>(new int[DETECT_LINE_COUNT_]);
  last_lane_middle_arr_ = unique_ptr<Point[]>(new Point[DETECT_LINE_COUNT_]);
  start_flag_arr_ = unique_ptr<bool[]>(new bool[DETECT_LINE_COUNT_]);
  sustaining_arr_ = unique_ptr<bool[]>(new bool[DETECT_LINE_COUNT_]);

  for(int index = 0; index < DETECT_LINE_COUNT_; index++) {
    time_after_detect_obstacle_arr_[index] = sustaining_time_;
    start_flag_arr_[index] = true;
    sustaining_arr_[index] = false;
  }
}

// void VisionPathPlanner::setSustainingTime(const int sustaining_time) { sustaining_time_ = sustaining_time; }
void VisionPathPlanner::setChangePixelThres(const int change_pixel_thres) { change_pixel_thres_ = change_pixel_thres; }
void VisionPathPlanner::setTimeAfterDetectObs(const int time_after_detect_obstacle, const int index)
  throw(my_out_of_range)
{
  if(index >= DETECT_LINE_COUNT_) {
    throw_my_out_of_range(getOutOfRangeMsg(index, DETECT_LINE_COUNT_));
  }

  time_after_detect_obstacle_arr_[index] = time_after_detect_obstacle;
}

int VisionPathPlanner::getSustainingTime() const { return sustaining_time_; }
int VisionPathPlanner::getChangePixelThres() const { return change_pixel_thres_; }
int VisionPathPlanner::getTimeAfterDetectObs(const int index) const
  throw(my_out_of_range)
{
  if(index >= DETECT_LINE_COUNT_) {
    throw_my_out_of_range(getOutOfRangeMsg(index, DETECT_LINE_COUNT_));
  }

  return time_after_detect_obstacle_arr_[index];
}

Point VisionPathPlanner::detectLaneCenter(const int index)
  throw(my_out_of_range)
{
  if(index >= DETECT_LINE_COUNT_) {
    throw_my_out_of_range(getOutOfRangeMsg(index, DETECT_LINE_COUNT_));
  }

  Point new_middle(Point((cur_right_point_arr_[index].x + cur_left_point_arr_[index].x) / 2, roi_binary_img_.rows * detect_y_offset_arr_[index] / 100));

  if(start_flag_arr_[index]) {
    last_lane_middle_arr_[index] = new_middle;
    start_flag_arr_[index] = false;
  }

  if((abs(new_middle.x - last_lane_middle_arr_[index].x) > change_pixel_thres_) && !sustaining_arr_[index]) {
    // cout << "new_middle: " << new_middle.x << endl;
    // cout << "last_lane_middle_arr_: " << last_lane_middle_arr_[index].x << endl;
    cout <<  abs(new_middle.x - last_lane_middle_arr_[index].x) << " > " << change_pixel_thres_ << endl;
    time_after_detect_obstacle_arr_[index] = 0;
    sustaining_arr_[index] = true;
  }

  if(time_after_detect_obstacle_arr_[index] >= sustaining_time_) {
    last_lane_middle_arr_[index].x = new_middle.x;
    sustaining_arr_[index] = false;
  }

  time_after_detect_obstacle_arr_[index]++;
  cout << "index: " << index << ", time: " << time_after_detect_obstacle_arr_[index] << endl;

  return last_lane_middle_arr_[index];
}
