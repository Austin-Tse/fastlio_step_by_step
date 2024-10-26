#pragma once
#include "ieskf_slam/type/imu.h"
#include "ieskf_slam/type/pointcloud.h"
#include <deque>
namespace IESKFSlam
{
  struct MeasureGroup
  { 
    //雷达的时间
    double lidar_begin_time;
    double lidar_end_time;
    // 点云的数据
    PointCloud cloud;
    // imu的数据
    std::deque<IMU> imus;
  };
  
} // namespace IESKFSlam
