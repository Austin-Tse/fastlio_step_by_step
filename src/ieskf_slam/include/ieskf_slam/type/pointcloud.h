#pragma once
#include <ieskf_slam/type/point.h>
#include <ieskf_slam/type/timestamp.h>

namespace IESKFSlam
{
  // 这是在IESKFSlam下的Point类型
  using PCLPointCloud = pcl::PointCloud<Point>;//using关键字用于定义类型别名，它让程序员能够为已存在的类型赋予一个新的名称
  using PCLPointPtr = pcl::PointCloud<Point>::Ptr;
    using PCLPointCloudPtr = PCLPointCloud::Ptr;
  struct PointCloud{
    using Ptr = std::shared_ptr<PCLPointCloud>;//C++智能指针类型，用于自动管理PCLPointCloud对象的生命周期   using起别名
    TimeStamp time_stamp;
    PCLPointPtr cloud_ptr;
    PointCloud()
    {
      cloud_ptr = pcl::make_shared<PCLPointCloud>();//定义为PCLPointCloud的共享指针，用于在多个组件间共享点云数据。
    }
  };

} // namespace IESKFSlam
