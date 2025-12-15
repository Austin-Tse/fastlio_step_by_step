#pragma once
#include "common_lidar_process_interface.h"

namespace avia_ros
{
  struct EIGEN_ALIGN16 Point
  {
    PCL_ADD_POINT4D;
    float intensity;
    std::uint32_t offset_time;
    std::uint8_t line;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
} // namespace avia_ros


POINT_CLOUD_REGISTER_POINT_STRUCT(avia_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint32_t, offset_time, offset_time)
    (std::uint8_t, line, line)
)

namespace ROSNoetic
{
  class AVIAProcess:public CommonLidarProcessInterface
  {
    public:
    AVIAProcess(){}
    ~AVIAProcess(){}
    
    bool process(const sensor_msgs::PointCloud2 &msg,IESKFSlam::Frame &frame){
      pcl::PointCloud<avia_ros::Point> avia_cloud;
      pcl::fromROSMsg(msg,avia_cloud);
      frame.cloud_ptr->clear();
      for (auto &&point:avia_cloud)//使用&&（右值引用）可以避免每次循环时拷贝avia_cloud中的元素，直接操作元素，提升代码性能
      {
        IESKFSlam::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = point.z;
        p.intensity = point.intensity;
        p.ring = point.line;
        p.offset_time = point.offset_time;
        frame.cloud_ptr->push_back(p);//cloud是PC实例化的对象中的一个pc成员，所以使用push_back
        /* code */
      }
      frame.time_stamp.fromNsec(msg.header.stamp.toNSec());
      return true;
    }
  };
} // namespace ROSNoetic
