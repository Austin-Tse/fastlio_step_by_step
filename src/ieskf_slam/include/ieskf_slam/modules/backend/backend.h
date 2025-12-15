#pragma once
#include "ieskf_slam/modules/modules_base.h"
#include "ieskf_slam/type/base_type.h"

namespace IESKFSlam
{
    class BackEnd:private ModuleBase
    {
    public:
        BackEnd(const std::string &config_file_path,const std::string & prefix );
        ~BackEnd();
    };
}