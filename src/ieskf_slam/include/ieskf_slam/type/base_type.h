#pragma once
#include <ieskf_slam/type/pointcloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
// #include <voxel_grid.h>//使用ros自带的pcl

namespace IESKFSlam
{
  using VoxelFilter = pcl::VoxelGrid<Point>;


  //KD-TREE
  using KDTree = pcl::KdTreeFLANN<Point>;
  using KDTreePtr = KDTree::Ptr;
  using KDTreeConstPtr = KDTree::ConstPtr;

  //定义重力常量
  const double GRAVITY = 9.81;

  template<typename _first, typename _second, typename _third>
  struct triple{
    _first first;
    _second second;
    _third third;
  };
}