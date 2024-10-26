#pragma once
#include "ieskf_slam/modules/module_base.h"
#include "ieskf_slam/type/pointcloud.h"

namespace IESKFSlam
{
  class RectMapManager:private ModuleBase
  { 
    private:
    PCLPointCloudPtr local_map_ptr;
    PCLPointCloudPtr localmap;
    PCLPointCloudPtr los;
    PCLPointCloudPtr s;

  };
}