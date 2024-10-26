#include "ieskf_slam/modules/frontend/frontend.h"
#include "ieskf_slam/modules/frontend/rect_map_manager.h"

namespace IESKFSlam
{
  FrontEnd::FrontEnd(const std::string &config_file_path,const std::string & prefix):ModuleBase(config_file_path,"Front End Module")//实例化FronEnd之前初始化父类ModulBase ，传入两个参数
  {
    ieskf_ptr = std::make_shared<IESKF>(config_file_path,"ieskf");
    map_ptr = std::make_shared<RectMapManager>(config_file_path,"map");
  }
  FrontEnd::~FrontEnd()
  {

  }


  void FrontEnd::addImu(const IMU& imu)//添加imu数据
  {
    imu_deque.push_back(imu);
  }


  void FrontEnd::addPointCloud(const PointCloud& pointcloud)
  {
    pointcloud_deque.push_back(pointcloud);
    std::cout<<"receive cloud"<<std::endl;
  }


  bool FrontEnd::track()
  {
    MeasureGroup mg;
    if(syncMeasureGroup(mg))
    {
      if(!imu_inited)
      {
        map_ptr->reset();
      }
    }
  } // namespace IESKF_Slam

  //检查pointcloud_deque imu_deque 是否符合同步的要求
  bool FrontEnd::syncMeasureGroup(MeasureGroup& mg)
  {
    mg.imus.clear();
    mg.cloud.cloud_ptr->clear();

    if(pointcloud_deque.empty()||imu_deque.empty())
    {
      return false;
    }

    double imu_end_time = imu_deque.back().time_stamp.sec();
    double imu_start_time = imu_deque.front().time_stamp.sec();
    double cloud_start_time = pointcloud_deque.front().time_stamp.sec();//数组头就是指针
    double cloud_end_time = pointcloud_deque.front().cloud_ptr->back().offset_time/1e9+cloud_start_time;
    

    if(imu_end_time<cloud_end_time)//imu不能覆盖cloud
    {
      return false;
    }
    if (cloud_end_time<imu_start_time)//点云的时间都老于imu的时间，证明点云数据已经过时
    {
      pointcloud_deque.pop_front();
      return false;
    }

    mg.cloud = pointcloud_deque.front();
    pointcloud_deque.pop_front();
    mg.lidar_begin_time = cloud_start_time;//该点云的时间段
    mg.lidar_end_time = cloud_end_time;

    //TODO--添加代码验证imu早于雷达的数据
    while (!imu_deque.empty() && imu_deque.front().time_stamp.sec() < mg.lidar_begin_time) {
    imu_deque.pop_front();
    std::cout<<"imu数据早于最早一帧的点云"<<std::endl;
    }
    while (!imu_deque.empty())
    {
      if (imu_deque.front().time_stamp.sec()<mg.lidar_end_time) //保留所有在这一帧点云结束之前的数据
      {
        //放入mg.deque
        mg.imus.push_back(imu_deque.front());
        imu_deque.pop_front();
        std::cout<<"imu数据晚于最早一帧的点云"<<std::endl;
      }else{
        break;
      }
    }

    if(mg.imus.size()<5)
    {
      return false;
      std::cout<<"mg.imus.size()<5"<<std::endl;
    }
    return true;

  }
}