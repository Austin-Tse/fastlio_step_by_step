//
// Created by xie on 24-10-31.
//
#include "ieskf_slam/modules/frontback_Propagate/frontback_propagate.h"
namespace IESKFSlam
{
    FrontbackPropagate::FrontbackPropagate() {}
    FrontbackPropagate::~FrontbackPropagate() {}
    void FrontbackPropagate::propagate(IESKFSlam::MeasureGroup &mg, IESKF::Ptr ieskf_ptr) {
        //将每一帧的点云按照采集的时间从大到小排序
        std::sort(mg.cloud.cloud_ptr->points.begin(),mg.cloud.cloud_ptr->points.end(),[](Point x,Point y)->bool{return x.offset_time<y.offset_time;});
        mg.imus.push_front(last_imu);

        double dt = 0;
        IMU in;
        IESKF::State18 imu_state;
        for (auto it_imu= mg.imus.begin();it_imu<(mg.imus.end()-1) ;it_imu ++) {
            auto &&head = *(it_imu);
            auto &&tail = *(it_imu+1);

            auto angvel_avr = 0.5*(head.gyroscope+tail.gyroscope);
            auto acc_avr = 0.5 * (head.acceleration+tail.acceleration) * imu_scale;//TODO
            double dt = tail.time_stamp.sec() - head.time_stamp.sec();//IMU的时间
            in.acceleration = acc_avr;
            in.gyroscope = angvel_avr;
            ieskf_ptr->predict(in,dt);
        }
        dt = mg.lidar_end_time-mg.imus.back().time_stamp.sec();//最后一个 IMU 数据点到激光雷达结束时间的时间差 dt，并进行一次预测
        ieskf_ptr->predict(in,dt);
        //记录最后一个imu数据
        last_imu = mg.imus.back();
        last_imu.time_stamp.fromSec(mg.lidar_end_time);
    }
}