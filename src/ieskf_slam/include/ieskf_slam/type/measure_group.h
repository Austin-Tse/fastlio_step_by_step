//
// Created by xie on 24-10-27.
//

#pragma once
#include "ieskf_slam/type/imu.h"
#include "ieskf_slam/type/frame.h"
#include <deque>
#include "ieskf_slam/type/frame.h"
#include "ieskf_slam/type/pointcloud.h"

namespace IESKFSlam
{
    //存储了一组测量数据，包括激光雷达的起始和结束时间戳、一系列IMU测量数据以及对应的点云数据。
    struct MeasureGroup{
        double lidar_begin_time;
        double lidar_end_time;
        std::deque<IMU> imus;
        Frame frame;//结构体
        PointCloud cloud;
    };
}