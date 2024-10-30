#pragma once
#include "ieskf_slam/modules/frontend/frontend.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "wrapper/ros_noetic/avia_process.h"

namespace ROSNoetic
{
  enum LIDAR_TYPE
  {
    AVIA = 0,
  };
  class IESKFFrontEndWrapper
  {
  private:
    IESKFSlam::FrontEnd::Ptr front_end_ptr;
    ros::Subscriber cloud_subscriber;
    ros::Subscriber imu_subscriber;
//    ros::Subscriber odometry_subscriber;
    ros::Publisher curr_cloud_pub;

    ros::Publisher path_pub;

    std::shared_ptr<CommonLidarProcessInterface> lidar_process_ptr;

    IESKFSlam::PCLPointCloud curr_cloud;
    Eigen::Quaterniond curr_q;
    Eigen::Vector3d curr_r;
    void lidarCloudMsgCallBack(const sensor_msgs::PointCloud2ConstPtr& msg);
    void imuMsgCallBack(const sensor_msgs::ImuPtr& msg);
//    void odometryMsgCallBack(const nav_msgs::OdometryPtr& msg);
    void run();
    void publishMsg();
  public:
    IESKFFrontEndWrapper(ros::NodeHandle& nh);
    ~IESKFFrontEndWrapper();
  };

  
}