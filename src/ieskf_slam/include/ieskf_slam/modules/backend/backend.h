#pragma once
#include "ieskf_slam/modules/modules_base.h"
#include "ieskf_slam/type/base_type.h"
#include "ieskf_slam/type/pose.h"
#include "ieskf_slam/type/pointcloud.h"
#include "ieskf_slam/modules/pose_graph_opt/pose_graph_opt.h"
#include <pcl/registration/icp.h>
#include "scan_context/Scancontext.h"
namespace IESKFSlam
{
    class BackEnd:private ModuleBase
    {
    public:
        BackEnd(const std::string &config_file_path,const std::string & prefix );
        ~BackEnd();
    
    using Ptr = std::shared_ptr<BackEnd>;
    bool scanRegister(Eigen::Matrix4f &match_result,int from_id,int to_id,float angle);
    bool addFrame(PCLPointCloud &opt_map,PCLPointCloud &curr_cloud,const Pose &pose);
    private:
        std::vector<Pose> poses;//关键帧位姿缓存

        SC2::SCManager SC_manager;//scan context管理器
        std::vector<PCLPointCloud> clouds;//关键帧点云缓存

        Eigen::Quaterniond extrin_r;
        Eigen::Vector3d extrin_t;
        std::vector<BinaryEdge> binary_edges;
        VoxelFilter voxel_filter;
        PoseGraphOpt pgo;

    };

}