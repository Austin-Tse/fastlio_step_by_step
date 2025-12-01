


#pragma once


#include "ieskf_slam/modules/ieskf/ieskf.h"


#include "ieskf_slam/type/base_type.h"
#include "ieskf_slam/math/geometry.h"



namespace IESKFSlam
{
    class LIOZHModel : public IESKF::CalZHInterface
    {
        private:
            using loss_type = triple<Eigen::Vector3d,Eigen::Vector3d,double>;
            PCLPointCloudPtr current_cloud_ptr;
            KDTreeConstPtr global_map_kdtree_ptr;
            PCLPointCloudConstPtr local_map_ptr;

            const int NEAR_POINTS_NUM = 5;


        public:
        using Ptr = std::shared_ptr<LIOZHModel>;
        void prepare(KDTreeConstPtr kd_tree,PCLPointPtr current_cloud,PCLPointCloudConstPtr local_map){
            global_map_kdtree_ptr = kd_tree;
            current_cloud_ptr = current_cloud;
            local_map_ptr = local_map;
        }
            bool calculate(const IESKF::State18 &state, Eigen::MatrixXd&Z, Eigen::MatrixXd&H) override;//override是在重写基类的虚函数时使用的关键字，表示这个函数是对基类中同名虚函数的重写。

    };

    // 函数的核心任务是计算观测矩阵 H 和观测残差向量 Z
    inline bool LIOZHModel::calculate(const IESKF::State18 &state, 
                               Eigen::MatrixXd &Z, 
                               Eigen::MatrixXd &H)
    {
        std::vector<loss_type> loss_v;//存储损失函数相关数据
        loss_v.resize(current_cloud_ptr->points.size());//预分配空间以提高效率
        std::vector<bool> is_effect_point(current_cloud_ptr->points.size(), false);//标记哪些点对计算有效
        std::vector<loss_type> loss_real;//存储最终有效的损失函数数据
        int vaild_point_num = 0;//有效点的数量

        #ifdef MP_EN
            omp_set_num_threads(MP_PROC_NUM);
            #pragma omp parallel for
        #endif
        for(size_t i = 0; i < current_cloud_ptr->points.size(); ++i)//遍历当前点云中的每个点
        {
            Point point_imu = current_cloud_ptr->points[i];
            Point point_world;
            point_world = transformPoint(point_imu,state.rotation,state.position);//将点从IMU坐标系转换到世界坐标系

            std::vector<int> point_ind;
            std::vector<float> distance;

            global_map_kdtree_ptr->nearestKSearch(point_world,NEAR_POINTS_NUM,point_ind,distance);//在KD-TREE中查找最近邻点  每一个点在kdtree中查找5个最近邻点
            if(distance.size()<NEAR_POINTS_NUM || distance[NEAR_POINTS_NUM-1]>5.0f)
                continue;
            std::vector<Point> planar_points;
            for (size_t ni = 0; ni < NEAR_POINTS_NUM; ni++)//遍历最近邻点
            {
                planar_points.push_back(local_map_ptr->points[point_ind[ni]]);
            }
            Eigen::Vector4d pabcd;
            if (planarCheck(planar_points,pabcd,0.1))//函数验证最近邻点是否构成平面。如果成功,则计算平面参数
            {
                double pd = point_world.x * pabcd(0) + point_world.y * pabcd(1) + point_world.z * pabcd(2) + pabcd(3);//计算点到平面的距离
                loss_type loss;
                loss.first = Eigen::Vector3d(point_imu.x,point_imu.y,point_imu.z);//点在IMU坐标系下的坐标
                loss.second = pabcd.block<3,1>(0,0);//平面的法向量
                loss.third = pd;//点到平面的距离
                if (isnan(pd) || isnan(loss.second(0)) || isnan(loss.second(1)) || isnan(loss.second(2)))
                {
                    continue;
                }
                double s = 1-0.9*fabs(pd)/sqrt(loss.first.norm());//计算一个得分 s，用于评估点的有效性
                
                if (s >0.9)//距离越近的点权重越高
                {
                    vaild_point_num++;
                    loss_v[i] = loss;
                    is_effect_point[i] = true;
                }
                
            }
        }
            for (size_t i = 0; i < current_cloud_ptr->size(); i++)
            {
                if (is_effect_point[i])loss_real.push_back(loss_v[i]);
            }
            vaild_point_num = loss_real.size();
            H = Eigen::MatrixXd::Zero(vaild_point_num,18);
            Z.resize(vaild_point_num,1);
            for (size_t vi = 0; vi < vaild_point_num; vi++)//遍历所有有效点
            {
                Eigen::Vector3d dr = -1.0 * loss_real[vi].second.transpose() * state.rotation.toRotationMatrix() *
                                skewSymmetric(loss_real[vi].first);
                H.block<1,3>(vi,0) = dr.transpose();//观测矩阵H的相关部分-旋转部分
                H.block<1,3>(vi,3) = loss_real[vi].second.transpose();//观测矩阵H的相关部分-平移部分
                Z(vi,0) = loss_real[vi].third;//观测残差
            }
        return true;
    }
}
