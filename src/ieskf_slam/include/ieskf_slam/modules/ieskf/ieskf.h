//
// Created by xie on 24-10-27.
//

#pragma once
#include "ieskf_slam/modules/modules_base.h"
#include "ieskf_slam/type/imu.h"
#include <Eigen/Dense>

namespace IESKFSlam
{
    class IESKF:private ModuleBase
    {
    public:
        struct State18
        {
            Eigen::Quaterniond  rotation;
            Eigen::Vector3d position;
            Eigen::Vector3d velocity;
            Eigen::Vector3d bg;
            Eigen::Vector3d ba;
            Eigen::Vector3d gravity;
            State18(){
                rotation = Eigen::Quaterniond::Identity();
                position = Eigen::Vector3d::Zero();
                velocity = Eigen::Vector3d ::Zero();
                bg = Eigen::Vector3d ::Zero();
                ba = Eigen::Vector3d ::Zero();
                gravity = Eigen::Vector3d ::Zero();
            }
        };
    private:
        State18 X;
    public:
        IESKF(const std::string & config_path,const std::string& prefix);
        ~IESKF();
        void predict(const IMU&imu,double dt);
        bool update();
        const State18&getX();
        void setX(const State18&x_in);
    };
}