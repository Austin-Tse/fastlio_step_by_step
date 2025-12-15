/*
 * @Descripttion: 
 * @Author: MengKai
 * @version: 
 * @Date: 2023-06-09 00:07:58
 * @LastEditors: MengKai
 * @LastEditTime: 2023-06-14 12:19:34
 */
#include "ieskf_slam/modules/frontend/frontend.h"


namespace IESKFSlam
{
    FrontEnd::FrontEnd(const std::string &config_file_path,const std::string & prefix ):ModuleBase(config_file_path,prefix,"Front End Module")
    {
        float leaf_size;
        readParam("filter_leaf_size",leaf_size,0.5f);
        voxel_filter.setLeafSize(leaf_size,leaf_size,leaf_size);



        //读取imu参数
        //旋转
        std::vector<double> extrin_v;
        readParam("extrin_r",extrin_v,std::vector<double>());
        extrin_r_q.setIdentity();
        if (extrin_v.size() ==9)
        {
            Eigen::Matrix3d extrin_r_m;
            extrin_r_m<<extrin_v[0],extrin_v[1],extrin_v[2],
                         extrin_v[3],extrin_v[4],extrin_v[5],
                         extrin_v[6],extrin_v[7],extrin_v[8];
            extrin_r_q = Eigen::Quaterniond(extrin_r_m);
        }else if (extrin_v.size() == 4)
        {
            extrin_r_q.x() = extrin_v[0];
            extrin_r_q.y() = extrin_v[1];
            extrin_r_q.z() = extrin_v[2];
            extrin_r_q.w() = extrin_v[3];
        }

        //平移
        std::vector<double> extrin_t;
        readParam("extrin_t",extrin_t,std::vector<double>());
        extrin_t_v.setZero();
        if (extrin_t.size() ==3)
        {
            extrin_t_v<<extrin_t[0],extrin_t[1],extrin_t[2];
        }


        ieskf_ptr = std::make_shared<IESKF>(config_file_path,"ieskf");
        map_ptr  = std::make_shared<RectMapManager>(config_file_path,"map");
        fbpropagate_ptr = std::make_shared<FrontbackPropagate>();
        lio_zh_model_ptr = std::make_shared<LIOZHModel>();
        ieskf_ptr->calc_zh_ptr = lio_zh_model_ptr;
        filter_point_cloud_ptr = pcl::make_shared<PCLPointCloud>();
        lio_zh_model_ptr->prepare(
            map_ptr->readKDtree(),
            filter_point_cloud_ptr,
            map_ptr->getLocalMap()
        );
        readParam("enable_record",enable_record,true);
        readParam("record_file_name",record_file_name,std::string("frontend_record.txt"));
        if (enable_record)
        {
            record_file.open(RESULT_DIR+record_file_name,std::ios::out);
            if (!record_file.is_open())
            {
                std::cerr<<"Failed to open record file: "<<record_file_name<<std::endl;
            }else{
                std::cout<<"Record file opened: "<<record_file_name<<std::endl;
            }
        }
        print_table();
        
    }

    FrontEnd::~FrontEnd()
    {
        record_file.close();
    }
    //传入imu数据
    void FrontEnd::addImu(const IMU&imu){
        imu_deque.push_back(imu);
       std::cout<<"receive imu"<<std::endl;
    }

    //传入点云数据
    void FrontEnd::addPointCloud(const Frame&frame){
        frame_deque.push_back(frame);
        std::cout<<"receive cloud"<<std::endl;
    }
    bool FrontEnd::track(){
        MeasureGroup mg;//雷达开始时间，雷达结束时间，imu deque，pointcloud
        if(syncMeasureGroup(mg)){

            if(!imu_inited){
                map_ptr->reset();
                map_ptr->addScan(mg.frame.cloud_ptr,Eigen::Quaterniond::Identity(),Eigen::Vector3d::Zero());
                initState(mg);
                return false;
            }
            static int frame_count = 0;
            frame_count++;
            
            fbpropagate_ptr->propagate(mg,ieskf_ptr);
            
            auto x_before_update = ieskf_ptr->getX();
            std::cout << "\n[FRAME #" << frame_count << "] Before update - pos: [" 
                      << x_before_update.position.transpose() << "], vel_norm: " 
                      << x_before_update.velocity.norm() << std::endl;
            
            voxel_filter.setInputCloud(mg.frame.cloud_ptr);
            voxel_filter.filter(*filter_point_cloud_ptr);//滤波后的点云存储在filter
            std::cout << "Point cloud: raw=" << mg.frame.cloud_ptr->size() 
                      << ", filtered=" << filter_point_cloud_ptr->size() << std::endl;
            
            ieskf_ptr->update();
            auto x = ieskf_ptr->getX();
            
            std::cout << "After update - pos: [" << x.position.transpose() 
                      << "], vel_norm: " << x.velocity.norm() << std::endl;
            
            // 检查位置变化是否异常
            double pos_change = (x.position - x_before_update.position).norm();
            if (pos_change > 10.0) {
                std::cout << "[WARN] Large position jump: " << pos_change << " meters!" << std::endl;
            }
            
            // 将IMU位姿转换到LiDAR位姿后再入图
            // Eigen::Quaterniond lidar_q = x.rotation * extrin_r_q; // R_l = R_i * R_extr
            // Eigen::Vector3d lidar_p = x.rotation * extrin_t_v + x.position; // p_l = R_i * t_extr + p_i

            Eigen::Quaterniond rota = Eigen::Quaterniond::Identity();
            Eigen::Vector3d trans = Eigen::Vector3d::Zero();
            if (enable_record)
            {
                // 设置15位小数
                record_file<<std::setprecision(15)<<mg.lidar_end_time<<" "
                           <<x.position.x()<<" "<<x.position.y()<<" "<<x.position.z()<<" "
                           <<x.rotation.x()<<" "<<x.rotation.y()<<" "<<x.rotation.z()<<" "<<x.rotation.w()<<std::endl;
            }
            map_ptr->addScan(filter_point_cloud_ptr,x.rotation,x.position);

            
            return true;
        }
        return false;
    }
    const PCLPointCloud& FrontEnd::readCurrentPointCloud(){
        return *filter_point_cloud_ptr;
    }


    bool FrontEnd::syncMeasureGroup(MeasureGroup&mg){
        //同步一帧点云和对应的imu数据
        mg.imus.clear();
        mg.frame.cloud_ptr->clear();
        if ( frame_deque.empty()||imu_deque.empty())
        {
            return false;
        }
        ///. wait for imu
        double imu_end_time = imu_deque.back().time_stamp.sec();
        double imu_start_time = imu_deque.front().time_stamp.sec();
        double cloud_start_time =frame_deque.front().time_stamp.sec();
        double cloud_end_time = frame_deque.front().cloud_ptr->points.back().offset_time/1e9+cloud_start_time;

        //当imu末尾数据早于这一帧的pointcloud数据的结束点云数据，无法铺满两帧之间，点云后半段是空的
        if (imu_end_time<cloud_end_time){
            return false;
        }

        //当imu的起始时间早于这一帧pointcloud数据的末尾，无法铺满两帧之间，点云后前段是空的
        if (imu_start_time>cloud_end_time)
        {
            frame_deque.pop_front();
            std::cout<<"imu_start_time>cloud_end_time"<<std::endl;
            return false;
        }


        mg.frame = frame_deque.front();//取出点云数据
        frame_deque.pop_front();
        //设置点云的起始和结束时间
        mg.lidar_begin_time = cloud_start_time;
        mg.lidar_end_time = cloud_end_time;

        while (!imu_deque.empty())
        {
            if (imu_deque.front().time_stamp.sec()<mg.lidar_end_time)//在这一帧结束之前的imu都扔进去
            {
                mg.imus.push_back(imu_deque.front());
                imu_deque.pop_front();

            }else{
                break;
            }
        }
        if(mg.imus.size()<=5){//如果这一帧点云结束之前的imu少于5个，那么直接结束

            return false;
        }
        return true;
    }
    // 初始化，利用多帧imu数据计算初始加速度均值和陀螺仪偏置
    void FrontEnd::initState(MeasureGroup&mg){
        static int imu_count = 0;
        static Eigen::Vector3d mean_acc{0,0,0};
        auto &ieskf= *ieskf_ptr;
        if (imu_inited)
        {
            return ;
        }

        std::cout << "[INIT] Collecting IMU data for initialization... current count: " << imu_count << std::endl;
        
        // 遍历这一帧点云中的所有imu数据，累加加速度和陀螺仪偏置
        for (size_t i = 0; i < mg.imus.size(); i++)
        {
            imu_count++;
            auto x = ieskf.getX();
            mean_acc +=mg.imus[i].acceleration;//累加加速度
            x.bg += mg.imus[i].gyroscope;//累加偏置
            ieskf.setX(x);

        }
        if (imu_count >= 5)
        {
            auto x = ieskf.getX();
            mean_acc /=double(imu_count);//求得平均加速度

            x.bg /=double(imu_count);//求得平均偏置
            imu_scale  = GRAVITY/mean_acc.norm();//计算imu尺度因子

            
            std::cout << "========== IMU INITIALIZATION COMPLETE =========="<< std::endl;
            std::cout << "imu_scale: " << imu_scale << std::endl;
            std::cout << "mean_acc: [" << mean_acc.transpose() << "], norm: " << mean_acc.norm() << std::endl;
            std::cout << "bg (gyro bias): [" << x.bg.transpose() << "]" << std::endl;
            
            // 重力的符号为负 就和fastlio公式一致  重力对齐，初始化方向为imu初始静止的方向
            x.gravity = - mean_acc / mean_acc.norm() * GRAVITY;
            std::cout << "gravity: [" << x.gravity.transpose() << "]" << std::endl;
            
            // 检查初始化的合理性
            if ((imu_scale < 0.5 || imu_scale > 2.0) && (imu_scale < 9.0 || imu_scale > 11.0)) {
                std::cout << "[WARN] IMU scale abnormal! Expected ~1.0, got " << imu_scale << std::endl;
            }
            if (x.bg.norm() > 0.5) {
                std::cout << "[WARN] Gyro bias large: " << x.bg.norm() << " rad/s" << std::endl;
            }
            std::cout << "================================================="<< std::endl;
            
            ieskf.setX(x);
            imu_inited = true;
            fbpropagate_ptr->imu_scale = imu_scale;//传递imu尺度因子
            fbpropagate_ptr->last_imu = mg.imus.back();//当前帧的最后一个imu作为下一帧的第一个imu
            fbpropagate_ptr->last_lidar_end_time_ = mg.lidar_end_time;//当前帧的点云结束时间作为下一帧的点云开始时间
        }
        return ;
    }


    IESKF::State18 FrontEnd::readState() {
        return ieskf_ptr->getX();
    }


} // namespace IESKFSlam