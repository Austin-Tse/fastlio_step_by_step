/*
 * @Descripttion: 
 * @Author: MengKai
 * @version: 
 * @Date: 2023-06-09 00:07:58
 * @LastEditors: MengKai
 * @LastEditTime: 2023-06-14 12:19:34
 */
#include "ieskf_slam/modules/frontend/frontend.h"

namespace IESKFSlam
{
    FrontEnd::FrontEnd(const std::string &config_file_path,const std::string & prefix ):ModuleBase(config_file_path,prefix,"Front End Module")
    {
        ieskf_ptr = std::make_shared<IESKF>(config_file_path,"ieskf");
        map_ptr  = std::make_shared<RectMapManager>(config_file_path,"map");
    }

    FrontEnd::~FrontEnd()
    {
    }
    void FrontEnd::addImu(const IMU&imu){
        imu_deque.push_back(imu);
//        std::cout<<"receive imu"<<std::endl;
    }
    void FrontEnd::addPointCloud(const PointCloud&pointcloud){
        pointcloud_deque.push_back(pointcloud);
        std::cout<<"receive cloud"<<std::endl;
    }
    bool FrontEnd::track(){
        MeasureGroup mg;
        if(syncMeasureGroup(mg)){

            if(!imu_inited){
                map_ptr->reset();
                map_ptr->addScan(mg.cloud.cloud_ptr,Eigen::Quaterniond::Identity(),Eigen::Vector3d::Zero());
                initState(mg);
                return false;
            }
            std::cout<<mg.imus.size()<<" scale: "<<imu_scale<<std::endl;
            return true;
        }
        return false;
    }
    const PCLPointCloud& FrontEnd::readCurrentPointCloud(){
        return current_pointcloud;
    }
    bool FrontEnd::syncMeasureGroup(MeasureGroup&mg){
        mg.imus.clear();
        mg.cloud.cloud_ptr->clear();
        if ( pointcloud_deque.empty()||imu_deque.empty())
        {
            return false;
        }
        ///. wait for imu
        double imu_end_time = imu_deque.back().time_stamp.sec();
        double imu_start_time = imu_deque.front().time_stamp.sec();
        double cloud_start_time =pointcloud_deque.front().time_stamp.sec();
        double cloud_end_time = pointcloud_deque.front().cloud_ptr->points.back().offset_time/1e9+cloud_start_time;

        //当imu末尾数据早于这一帧的pointcloud数据的结束点云数据，无法铺满两帧之间，点云后半段是空的
        if (imu_end_time<cloud_end_time){
            std::cout<<"imu_end_time<cloud_end_time"<<std::endl;
            return false;
        }

        //当imu的起始时间早于这一帧pointcloud数据的末尾，无法铺满两帧之间，点云后前段是空的
        if (imu_start_time>cloud_end_time)
        {
            pointcloud_deque.pop_front();
            std::cout<<"imu_start_time>cloud_end_time"<<std::endl;
            return false;
        }

        if(imu_start_time<cloud_start_time)
        {
            imu_deque.pop_front();
            std::cout<<"imu_start_time<cloud_start_time"<<std::endl;
            return false;
        }


        mg.cloud = pointcloud_deque.front();//取出点云数据
        pointcloud_deque.pop_front();
        mg.lidar_begin_time = cloud_start_time;
        mg.lidar_end_time = cloud_end_time;

        while (!imu_deque.empty())
        {
            if (imu_deque.front().time_stamp.sec()<mg.lidar_end_time)//在这一帧结束之前的imu都扔进去
            {
                mg.imus.push_back(imu_deque.front());
                imu_deque.pop_front();

            }else{
                break;
            }
        }
        if(mg.imus.size()<=5){//如果这一帧点云结束之前的imu少于5个，那么直接结束

            return false;
        }
        return true;
    }
    void FrontEnd::initState(MeasureGroup&mg){
        static int imu_count = 0;
        static Eigen::Vector3d mean_acc{0,0,0};
        auto &ieskf= *ieskf_ptr;
        if (imu_inited)
        {
            return ;
        }

        for (size_t i = 0; i < mg.imus.size(); i++)
        {
            imu_count++;
            auto x = ieskf.getX();
            mean_acc +=mg.imus[i].acceleration;//累加加速度
            x.bg += mg.imus[i].gyroscope;//累加偏置
            ieskf.setX(x);

        }
        if (imu_count >= 5)
        {
            auto x = ieskf.getX();
            mean_acc /=double(imu_count);//求得平均值

            x.bg /=double(imu_count);//求得平均偏置
            imu_scale  = GRAVITY/mean_acc.norm();
            std::cout<<"imu_scale"<<imu_scale<<std::endl;
            // 重力的符号为负 就和fastlio公式一致
            x.gravity = - mean_acc / mean_acc.norm() * GRAVITY;

            ieskf.setX(x);
            imu_inited = true;
        }
        return ;
    }
} // namespace IESKFSlam