#pragma once

#include "ieskf_slam/type/pose.h"
#include <vector>

namespace IESKFSlam
{
    struct BinaryEdge
    {
        int from_vertex;
        int to_vertex;
        Pose constraint; // 变换约束
        Eigen::Matrix<double,6,6> information; // 信息矩阵
        BinaryEdge(int i_fromv = 0,int i_tov = 0,Eigen::Matrix<double,6,6> i_info = Eigen::Matrix<double,6,6>::Identity())
            :from_vertex(i_fromv),to_vertex(i_tov),information(i_info){};
    };

    class PoseGraphOpt
    {
        private:
        public:
            PoseGraphOpt(){};
            ~PoseGraphOpt(){};
            bool solve(std::vector<Pose> &poses,std::vector<BinaryEdge> &bes);
    };
    
}