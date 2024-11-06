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
    public:
        double imu_scale;
        IMU last_imu;
        FrontbackPropagate();
        ~FrontbackPropagate();
        void propagate(MeasureGroup&mg,IESKF::Ptr ieskf_ptr);
    };
}
