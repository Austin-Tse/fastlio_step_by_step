//
// Created by xie on 24-10-31.
//

#pragma once
#include "ieskf_slam/modules/ieskf/ieskf.h"
#include "ieskf_slam/type/measure_group.h"

namespace IESKFSlam
{
    class FrontbackPropagate
    {
    private:
        struct IMUPose6d
        {
            double time;
            Eigen::Vector3d acc;//加速度
            Eigen::Vector3d angvel;//角速度
            Eigen::Vector3d vel;//速度
            Eigen::Vector3d pos;//位置
            Eigen::Quaterniond rot;//旋转

            IMUPose6d(double time_ = 0.0,Eigen::Vector3d acc_ = Eigen::Vector3d::Zero(),
                      Eigen::Vector3d angvel_ = Eigen::Vector3d::Zero(),
                      Eigen::Vector3d vel_ = Eigen::Vector3d::Zero(),
                      Eigen::Vector3d pos_ = Eigen::Vector3d::Zero(),
                      Eigen::Quaterniond rot_ = Eigen::Quaterniond::Identity())
                    : time(time_),acc(acc_),angvel(angvel_),vel(vel_),pos(pos_),rot(rot_){}


        };
        Eigen::Vector3d acc_s_last;
        Eigen::Vector3d angvel_last;
    public:
    // v5
        double last_lidar_end_time_;
        IMU last_imu_;

        double imu_scale;
        IMU last_imu;
        FrontbackPropagate();
        ~FrontbackPropagate();
        void propagate(MeasureGroup&mg,IESKF::Ptr ieskf_ptr);
    };
}
