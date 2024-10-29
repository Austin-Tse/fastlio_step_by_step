//
// Created by xie on 24-10-27.
//
#include "ieskf_slam/modules/ieskf/ieskf.h"
namespace IESKFSlam
{
    IESKF::IESKF(const std::string &config_path, const std::string &prefix): ModuleBase(config_path,prefix,"IESKF"){

    }

    IESKF::~IESKF()
    {

    }
    void IESKF::predict(const IESKFSlam::IMU &imu, double dt) {

    }

    bool IESKF::update() {

    }

    const IESKF::State18 &IESKF::getX() {
        return X;
    }

    void IESKF::setX(const IESKFSlam::IESKF::State18 &x_in) {
        X = x_in;
    }
}