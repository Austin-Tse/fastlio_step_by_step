#pragma once


#include "common_lidar_process_interface.h"

namespace velodyne_ros{
    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D;
        float intensity;
        uint16_t ring;
        float time;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
    
}

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring)(float, time, time))


namespace ROSNoetic
{
    class VelodyneProcess:public CommonLidarProcessInterface
    {
    private:
        /* data */
    public:
        bool process(const sensor_msgs::PointCloud2 &msg,IESKFSlam::Frame& frame){
            pcl::PointCloud<velodyne_ros::Point> rs_cloud;
            pcl::fromROSMsg(msg,rs_cloud);
            frame.cloud_ptr->clear();
            double end_time = msg.header.stamp.toSec();
            double start_time = end_time +rs_cloud[0].time;
            for (auto &&p:rs_cloud)
            {
                double point_time = p.time + end_time;
                IESKFSlam::Point point;
                point.x = p.x;
                point.y = p.y;
                point.z = p.z;
                point.intensity = p.intensity;
                point.ring = p.ring;
                point.offset_time = static_cast<uint32_t>((point_time - start_time)*1e9);
                frame.cloud_ptr->push_back(point);
            }
            frame.time_stamp.fromSec(start_time);
            return true;
        };
    };
}