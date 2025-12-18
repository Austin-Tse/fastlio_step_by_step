#include "ieskf_slam/modules/backend/backend.h"

namespace IESKFSlam
{
    BackEnd::BackEnd(const std::string &config_file_path,const std::string & prefix ):ModuleBase(config_file_path,prefix,"BackEnd")
    {
        print_table();
    }

    BackEnd::~BackEnd()
    {

    }



    bool BackEnd::addFrame(PCLPointCloud &opt_map, PCLPointCloud &curr_cloud,const Pose &pose)
    {
        //后端处理逻辑待实现
        static int cnt = 0;
        if (cnt > 100 || poses.empty() || (poses.back().position - pose.position).norm() > 1.0)
        {
            SC_manager.makeAndSaveScancontextAndKeys(curr_cloud);
            clouds.push_back(curr_cloud);
            poses.push_back(pose);
            cnt = 0;

            auto res = SC_manager.detectLoopClosureID();//检测闭环

            if (res.first != -1)
            {
                Eigen::Matrix4f trans_icp;
                Eigen::Matrix4d T_L_I,T_I_L,T_c;
                if (scanRegister(trans_icp,clouds.size() - 1,res.first,res.second))
                {
                    //闭环优化逻辑待实现
                    BinaryEdge be;
                    be.from_vertex = curr_cloud.size() - 1;
                    be.to_vertex = res.first;
                    //转换矩阵转pose
                    T_L_I.setIdentity();
                    T_L_I.block<3,3>(0,0) = extrin_r.toRotationMatrix();
                    T_L_I.block<3,1>(0,3) = extrin_t;

                    T_I_L.block<3,3>(0,0) = extrin_r.conjugate().toRotationMatrix();
                    T_I_L.block<3,1>(0,3) = extrin_r.conjugate() * (-extrin_t);

                    //计算T_c
                    T_c = T_L_I * trans_icp.cast<double>() * T_I_L;
                    be.constraint.rotation = T_c.block<3,3>(0,0);
                    be.constraint.position = T_c.block<3,1>(0,3);
                    binary_edges.push_back(be);

                    //复制所有位姿
                    std::vector<Pose> copy_poses = poses;
                    if (pgo.solve(copy_poses,binary_edges))
                    {
                        opt_map.clear();
                        for (size_t i = 0; i < clouds.size(); ++i)
                        {
                            //应用优化后的位姿
                            Eigen::Matrix4f T_I_W,T_L_W;
                            T_I_W.setIdentity();
                            T_I_W.block<3,3>(0,0) = copy_poses[i].rotation.cast<float>().toRotationMatrix();
                            T_I_W.block<3,1>(0,3) = copy_poses[i].position.cast<float>();
                            T_L_W = T_I_W * T_L_I.cast<float>();
                            PCLPointCloud global_cloud;
                            voxel_filter.setInputCloud(clouds[i].makeShared());
                            voxel_filter.filter(global_cloud);
                            pcl::transformPointCloud(global_cloud,global_cloud,T_L_W);
                            opt_map += global_cloud;
                        }
                        voxel_filter.setInputCloud(opt_map.makeShared());
                        voxel_filter.filter(opt_map);
                        for (int i = 0; i < poses.size(); i++)
                        {
                            poses[i] = copy_poses[i];
                        }
                        return true;
                        
                    }
                    

                }
                else
                {
                   binary_edges.pop_back();
                }
                
            }
            
        }



        return true;
    }



    //返回变换矩阵，输入源点云和目标点云的id，以及初始旋转角度
    bool BackEnd::scanRegister(Eigen::Matrix4f &match_result,int from_id,int to_id,float angle)
    {
        Eigen::AngleAxisf init_rotation(-1 * angle,Eigen::Vector3f::UnitZ());//初始旋转矩阵 
        // 既然 Source 已经比 Target 多转了 angle 度，为了让 Source “变回” 或者 “对齐” 到 Target 的姿态，我们需要将 Source 反向旋转 angle 度。
        Eigen::Matrix4f initial_tranform = Eigen::Matrix4f::Identity();

        initial_tranform.block<3,3>(0,0) = init_rotation.toRotationMatrix();
        pcl::IterativeClosestPoint<Point,Point> icp;
        icp.setInputSource(clouds[from_id].makeShared());
        icp.setInputTarget(clouds[to_id].makeShared());

        icp.setMaximumIterations(20);
        PCLPointCloud result;
        icp.align(result,initial_tranform);
        match_result = icp.getFinalTransformation();
        std::cout <<"icp_score: "<< icp.getFitnessScore() << std::endl;

        return icp.getFitnessScore() < 3.0;
    }
}