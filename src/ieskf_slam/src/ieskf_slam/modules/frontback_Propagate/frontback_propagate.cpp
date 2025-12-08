//
// Created by xie on 24-10-31.
//
#include "ieskf_slam/modules/frontback_Propagate/frontback_propagate.h"
namespace IESKFSlam
{
    FrontbackPropagate::FrontbackPropagate() {}
    FrontbackPropagate::~FrontbackPropagate() {}
    void FrontbackPropagate::propagate(IESKFSlam::MeasureGroup &mg, IESKF::Ptr ieskf_ptr) {
        //将每一帧的点云中的点按照采集的时间从大到小排序
        std::sort(mg.cloud.cloud_ptr->points.begin(),mg.cloud.cloud_ptr->points.end(),[](Point x,Point y)->bool{return x.offset_time<y.offset_time;});
        
        std::cout << "[PROPAGATE] IMU count: " << mg.imus.size() 
                  << ", time span: [" << mg.lidar_begin_time << ", " << mg.lidar_end_time 
                  << "], duration: " << (mg.lidar_end_time - mg.lidar_begin_time) << "s" << std::endl;
        

                  
        std::vector<IMUPose6d> IMUPose;
        //存储了这一帧的imu数据  包括时间，加速度，角速度，速度，位置，旋转
        auto v_imu = mg.imus;
        v_imu.push_front(last_imu);
        const double & imu_beg_time = v_imu.front().time_stamp.sec();
        const double & imu_end_time = v_imu.back().time_stamp.sec();
        std::cout << "[PROPAGATE] IMU time span: [" << imu_beg_time << ", " << v_imu.back().time_stamp.sec() << "]" << std::endl;
        //雷达开始时间
        const double & pcl_beg_time = mg.lidar_begin_time;
        const double & pcl_end_time = mg.lidar_end_time;
        //这一帧对应的点云数据
        auto &pcl_out = *mg.cloud.cloud_ptr;
        //获取当前状态
        auto imu_state = ieskf_ptr->getX();
        IMUPose.clear();
        IMUPose.emplace_back(0.0,acc_s_last,angvel_last,imu_state.velocity,imu_state.position,imu_state.rotation);

        Eigen::Vector3d angle_avr,acc_avr,acc_imu,vel_imu,pos_imu;
        Eigen::Matrix3d R_imu;

        // mg.imus.push_front(last_imu);
        double dt = 0;
        IMU in;
        // IESKF::State18 imu_state;

        //遍历imu数据进行预测
        for (auto it_imu= v_imu.begin();it_imu<(v_imu.end()-1);it_imu ++) {
            auto &&head = *(it_imu);//这是旧的imu数据
            auto &&tail = *(it_imu+1);//这是新的imu数据
            // std::cout << "[PROPAGATE] IMU time: " << head.time_stamp.sec() << " -> " << tail.time_stamp.sec() << std::endl;
            // std::cout << "[PROPAGATE] Lidar_begin_time: " << mg.lidar_begin_time << std::endl;
            // std::cout << "[PROPAGATE] Lidar_end_time: " << mg.lidar_end_time << std::endl;
            // std::cout << "[PROPAGATE] Last Lidar end time: " << last_lidar_end_time_ << std::endl;
            if (tail.time_stamp.sec() < last_lidar_end_time_)
            {
                continue;
            }
            auto angvel_avr = 0.5*(head.gyroscope+tail.gyroscope);
            auto acc_avr = 0.5 * (head.acceleration+tail.acceleration) * imu_scale;//TODO
            double dt = tail.time_stamp.sec() - head.time_stamp.sec();//相邻两个IMU的时间
            
            // 处理跨越情况：head < last_end < tail
            if (head.time_stamp.sec() < last_lidar_end_time_)
            {
                continue;
                // dt = tail.time_stamp.sec() - last_lidar_end_time_; TODO
            }
            else
            {
                dt = tail.time_stamp.sec() - head.time_stamp.sec();
            }

            in.acceleration = acc_avr;//赋值平均加速度
            in.gyroscope = angvel_avr;//赋值平均角速度
            ieskf_ptr->predict(in,dt);//进行预测

            imu_state = ieskf_ptr->getX();
            angvel_last = angvel_avr - imu_state.bg;//预测之后的去偏角速度
            acc_s_last  = imu_state.rotation * (acc_avr - imu_state.ba);//预测之后的去偏加速度

            for (size_t i = 0; i < 3; i++)//重力有三个方向的分量
            {
                acc_s_last[i] +=imu_state.gravity[i];//加上重力
            }
            // 后一个imu数据距离雷达开始时间的偏移
            double &&offs_t = tail.time_stamp.sec() - pcl_beg_time;
            // 数据越新，放在越前面  
            IMUPose.emplace_back(offs_t,acc_s_last,angvel_last,imu_state.velocity,imu_state.position,imu_state.rotation);
        }

        dt = pcl_end_time - imu_end_time;
        ieskf_ptr->predict(in,dt);
        imu_state = ieskf_ptr->getX();
        //后部是新数据
        last_imu = mg.imus.back();
        last_lidar_end_time_ = pcl_end_time;


        if (pcl_out.points.begin() == pcl_out.points.end())
        {
            return;
        }
        // 刚刚从大到小排序过，现在从后向前遍历点云中的点
        // 新----》旧
        auto it_pcl = pcl_out.points.end()-1;
        for (auto it_kp = IMUPose.end() -1;it_kp !=IMUPose.begin();it_kp--)
        {
            auto tail = it_kp;//新的imu位姿数据
            auto head = it_kp -1;//旧的imu位姿数据
            R_imu = head->rot.toRotationMatrix();
            vel_imu = head->vel;//开始时候的速度
            pos_imu = head->pos;//开始时候的位置
            acc_imu = tail->acc;//平均加速度
            angle_avr = tail->angvel;//平均角速度

            for (; it_pcl->offset_time / 1e9 > head->time; it_pcl --)//寻找的是所有采集时间晚于当前 IMU 数据块起始时间 head->time 的点。
            {
                dt = it_pcl->offset_time / 1e9 - head->time;//点相对于当前imu数据块起始时间的偏移
                Eigen::Matrix3d R_i(R_imu * so3Exp(angle_avr * dt));//旋转矩阵
                Eigen::Vector3d P_i(it_pcl->x,it_pcl->y,it_pcl->z);//点的坐标
                Eigen::Vector3d T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt-imu_state.position);//平移矩阵
                Eigen::Vector3d P_compensate = imu_state.rotation.conjugate() * (R_i * P_i + T_ei);//补偿后的点坐标
                it_pcl->x = P_compensate(0);
                it_pcl->y = P_compensate(1);
                it_pcl->z = P_compensate(2);
                if (it_pcl == pcl_out.points.begin())
                {
                    break;
                }

            }
            
        }
        return;
    }
}