#include "wrapper/ros_noetic/ieskf_frontend_noetic_wrapper.h"
#include "ieskf_slam/globaldefine.h"


namespace ROSNoetic
{
  IESKFFrontEndWrapper::IESKFFrontEndWrapper(ros::NodeHandle &nh)
  {
    std::string config_file_name,lidar_topic,imu_topic;
    nh.param<std::string>("wrapper/config_file_name",config_file_name,"");
    nh.param<std::string>("wrapper/lidar_topic",lidar_topic,"");
    nh.param<std::string>("wrapper/imu_topic",imu_topic,"");
    std::cout<<"lidar_topic: "<<lidar_topic<<std::endl;
    std::cout<<"imu_topic: "<<imu_topic<<std::endl;
    front_end_ptr = std::make_shared<IESKFSlam::FrontEnd>(CONFIG_DIR+config_file_name,"front_end");//构造共享指针对象，并使用指定的配置文件进行初始化

    //发布者和订阅者
    cloud_subscriber = nh.subscribe(lidar_topic,100,&IESKFFrontEndWrapper::lidarCloudMsgCallBack,this);
    imu_subscriber = nh.subscribe(imu_topic,100,&IESKFFrontEndWrapper::lidarCloudMsgCallBack,this);
    odometry_subscriber = nh.subscribe("/odometry",100,&IESKFFrontEndWrapper::odometryMsgCallBack,this);
    //读取雷达类型
    int lidar_type = 0;
    nh.param<int>("wrapper/lidar_type",lidar_type,AVIA);
    if(lidar_type == AVIA)
    {
      lidar_process_ptr = std::make_shared<AVIAProcess>();
    }
    else{
      std::cout <<"unsupport lidar type"<<std::endl;
      exit(100);//如果报错退出，那么可以捕获100
    }
    current_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("curr_cloud",100);
    run();
  }

    IESKFFrontEndWrapper::~IESKFFrontEndWrapper()
  {

  }
    void IESKFFrontEndWrapper::lidarCloudMsgCallBack(const sensor_msgs::PointCloud2ConstPtr&msg)
    {
      IESKFSlam::PointCloud cloud;
      lidar_process_ptr->process(*msg,cloud);
      front_end_ptr->addPointCloud(cloud);
    }

    void IESKFFrontEndWrapper::imuMsgCallBack(const sensor_msgs::ImuPtr &msg)
    {
      IESKFSlam::IMU imu;
      imu.time_stamp.fromNsec(msg->header.stamp.toNSec());
      imu.acceleration = {msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z};
      imu.gyroscale = {msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z};
      front_end_ptr->addImu(imu);
    }

    void IESKFFrontEndWrapper::odometryMsgCallBack(const nav_msgs::OdometryPtr& msg)
    {
      IESKFSlam::Pose pose;
      pose.time_stamp.fromNsec(msg->header.stamp.toNSec());
      pose.position = {msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z};
      pose.rotation.w() = msg->pose.pose.orientation.w;
      pose.rotation.x() = msg->pose.pose.orientation.x;
      pose.rotation.y() = msg->pose.pose.orientation.y;
      pose.rotation.z() = msg->pose.pose.orientation.z;
      front_end_ptr->addPose(pose);
    }

    void IESKFFrontEndWrapper::run(){
      ros::Rate rate(500);
      while (ros::ok())
      {
        /* code */
        rate.sleep();
        ros::spinOnce();
        if(front_end_ptr->track())
        {
          publishMsg();
        }
      }
      
    }

    void IESKFFrontEndWrapper::publishMsg()
    {
      auto cloud = front_end_ptr->readCurrentPointCloud();
      sensor_msgs::PointCloud2 msg;
      pcl::toROSMsg(cloud,msg);
      msg.header.frame_id = "map";
      current_cloud_pub.publish(msg);//发布话题信息
    }
    // void IESKFF lidarCloudMsgCallBack(const sensor_msgs::PointCloud2ConstPtr& msg)
    // {

    // }
    // void imuMsgCallBack(const sensor_msgs::ImuPtr& msg)
    // void odometryMsgCallBack(const nav_msgs::OdometryPtr& msg)
    // void run()
    // void publishMsg()


} // namespace ROSNoetic
