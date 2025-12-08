//
// Created by xie on 24-10-27.
//
#include "ieskf_slam/modules/map/rect_map_manager.h"
#include "pcl/common/transforms.h"
#include "ieskf_slam/math/math.h"


namespace IESKFSlam
{
    RectMapManager::RectMapManager(const std::string &config_file_path, const std::string &prefix): ModuleBase(config_file_path,prefix)
    {
        local_map_ptr = pcl::make_shared<PCLPointCloud>();
        kdtree_ptr = pcl::make_shared<KDTree>();
        readParam<float>("map_resolution",map_resolution,0.5f);
        readParam<float>("map_side_length_2",map_size_length_2,500.0f);
        std::cout<<"[RectMapManager] map_resolution: "<<map_resolution<<", map_size_length_2: "<<map_size_length_2<<std::endl;
    }

    RectMapManager::~RectMapManager()
    {
    }
    void RectMapManager::addScan(IESKFSlam::PCLPointCloudPtr curr_scan, const Eigen::Quaterniond &att_q,
                                 const Eigen::Vector3d &pos_t) {
        PCLPointCloud scan;
        pcl::transformPointCloud(*curr_scan,scan, compositeTransform(att_q,pos_t).cast<float>());

        // 如果地图为空，直接添加第一帧
        if (local_map_ptr->empty())
        {
            *local_map_ptr = scan;
        }
        else
        {
            // 添加新点：离最近点距离大于分辨率就添加
            for (auto &point:scan.points)    
            {
                std::vector<int> indices;
                std::vector<float> distance;
                kdtree_ptr->nearestKSearch(point, 5, indices, distance);
                if (distance[0] > map_resolution)
                {
                    local_map_ptr->points.push_back(point);
                }
            }
        }
        
        int before_size = local_map_ptr->points.size();
        std::cout<<"local map size before cull: "<<before_size<<std::endl;
        
        // 剔除超出范围的点（双指针分区算法）                          
        int left_index = 0, right_index = local_map_ptr->points.size()-1;
        while (left_index < right_index)
        {
            // 从右边找超出范围的点（任意坐标超出范围就删除）
            while (left_index < right_index && (abs(local_map_ptr->points[right_index].x-pos_t.x())>map_size_length_2
                   || abs(local_map_ptr->points[right_index].y - pos_t.y())>map_size_length_2
                   || abs(local_map_ptr->points[right_index].z - pos_t.z())>map_size_length_2))
            {
                right_index--;
            }
            // 从左边找在范围内的点（所有坐标都在范围内才保留）
            while (left_index < right_index && abs(local_map_ptr->points[left_index].x - pos_t.x())<=map_size_length_2
                && abs(local_map_ptr->points[left_index].y - pos_t.y())<=map_size_length_2
                && abs(local_map_ptr->points[left_index].z - pos_t.z())<=map_size_length_2)
            {
                left_index++;
            }
            std::swap(local_map_ptr->points[left_index],local_map_ptr->points[right_index]);
        }
        local_map_ptr->points.resize(right_index+1);

        int after_size = local_map_ptr->points.size();
        std::cout<<"local map size after cull: "<<after_size<<std::endl;
        kdtree_ptr->setInputCloud(local_map_ptr);//更新kdtree
        





        // VoxelFilter filter;
        // filter.setLeafSize(0.5f,0.5f,0.5f);
        // *local_map_ptr+=scan;//将变换的扫描点云添加到局部地图
        // filter.setInputCloud(local_map_ptr);//将局部地图体素滤波
        // filter.filter(*local_map_ptr);
        // kdtree_ptr->setInputCloud(local_map_ptr);//更新kdtree
    }
    void RectMapManager::reset() {
        local_map_ptr->clear();
    }

    PCLPointCloudPtr RectMapManager::getLocalMap() {
        return local_map_ptr;
    }

    KDTreeConstPtr RectMapManager::readKDtree() {
        return kdtree_ptr;
    }
}