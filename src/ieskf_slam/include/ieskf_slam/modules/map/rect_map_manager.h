//
// Created by xie on 24-10-27.
//



#pragma once
#include "ieskf_slam/modules/modules_base.h"
#include "ieskf_slam/type/pointcloud.h"
#include "ieskf_slam/type/base_type.h"
#include "pcl/common/transforms.h"
#include "ieskf_slam/math/math.h"
namespace IESKFSlam
{
    class RectMapManager:private ModuleBase
    {
    private:
        PCLPointCloudPtr local_map_ptr;
        KDTreePtr kdtree_ptr;
    public:
        RectMapManager(const std::string &config_file_path,const std::string &prefix);
        ~RectMapManager();
        void reset();
        void addScan(PCLPointCloudPtr curr_scan,const Eigen::Quaterniond &att_q,const Eigen::Vector3d &pos_t);
        PCLPointCloudPtr getLocalMap();
        KDTreeConstPtr readKDtree();
    };
}


