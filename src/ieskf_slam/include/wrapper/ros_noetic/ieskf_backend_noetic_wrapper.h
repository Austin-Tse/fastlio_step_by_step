#pragma once
#include "ros/ros.h"
#include "ieskf_slam/modules/backend/CloudWithPose.h"
#include "ieskf_slam/modules/backend/backend.h"


namespace ROSNoetic
{
class IESKFBackEndWrapper
{
private:
    ros::Publisher global_opt_map_pub;
    ros::Subscriber cloud_with_pose_sub;
    IESKFSlam::PCLPointCloud in_cloud,out_cloud;
    IESKFSlam::BackEnd::Ptr backend_ptr;

    void cloudWithPoseMsgCallBack(const ieskf_slam::CloudWithPosePtr &msg);
    void publishMsg();
public:
    IESKFBackEndWrapper(ros::NodeHandle& nh);
};
}