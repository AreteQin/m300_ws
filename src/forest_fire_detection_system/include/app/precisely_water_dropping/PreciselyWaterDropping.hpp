/*******************************************************************************
 *   Copyright (C) 2023 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: PreciselyWaterDropping.hpp
 *
 *   @Author: ShunLi
 *
 *   @Email: 2015097272@qq.com
 *
 *   @Date: 03/07/2023
 *
 *   @Description: this is the water dropping application with the help from
 *   ifrared images.
 *
 *******************************************************************************/

#include <ros/ros.h>

namespace FFDS {
namespace APP {
class PreciselyWaterDropper {
 private:
  int target_x_;
  int target_y_;
  int ctrl_times_;
  int ctrl_threshold_;

 public:
  PreciselyWaterDropper(const int target_x, const int target_y,
                        const int ctrl_times, const int ctrl_threshold);
  void run();
};

class WaterDropperMoveByOffset{
 private:
  float Des_x_;
  float Des_y_;
  float Des_z_;
  float Des_ya_;
  float PTM_;
  float YTD_;

  ros::NodeHandle nh;

public:
  WaterDropperMoveByOffset(
                           const float Des_x,
                           const float Des_y,
                           const float Des_z,
                           const float Des_ya,
                           const float PTM,
                           const float YTD);
  void run2();
};

}  // namespace APP
}  // namespace FFDS
