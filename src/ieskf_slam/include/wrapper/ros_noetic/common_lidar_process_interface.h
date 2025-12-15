#pragma once
#include "ieskf_slam/type/base_type.h"
#include <sensor_msgs/PointCloud2.h>
#include "pcl_conversions/pcl_conversions.h"

namespace ROSNoetic
{
  class CommonLidarProcessInterface
  {
  private:
    /* data */
  public:
  virtual bool process(const sensor_msgs::PointCloud2 &msg,IESKFSlam::Frame& frame) = 0;//纯虚函数
  };
  
  
}