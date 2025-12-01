#pragma once

#include "Eigen/Dense"
#include <vector>


namespace IESKFSlam
{
    //潜在问题 TODO
// "Gotcha"警告: 这里有一个隐藏的假设——代码假设拟合的平面不会通过原点附近。如果所有输入点都接近原点,固定 d = 1 的做法可能导致数值不稳定或错误的结果。更健壮的做法是先将点云中心化,或使用 SVD 方法直接求解法向量。
    //检查一组3D点是否位于同一平面上,并计算该平面的方程参数。
    //函数通过最小二乘法拟合一个平面方程
    // 函数通过最小二乘法拟合一个平面方程ax+by+cz+d=0,然后验证所有点是否都在这个平面附近(在阈值范围内)  -----d控制平面与原点的距离
    template<typename pointTypeT>
    bool planarCheck(const std::vector<pointTypeT>& points,Eigen::Vector4d& pabcd,float threshold)
    {
        Eigen::Vector3d normal_vector;//法向量
        Eigen::MatrixXd A;//存储点
        Eigen::VectorXd B;//
        int point_num = points.size();
        A.resize(point_num,3);
        B.resize(point_num);
        B.setOnes();
        B = B * -1.0;
        for (size_t i = 0; i < point_num; i++)
        {
            A(i,0) = points[i].x;
            A(i,1) = points[i].y;
            A(i,2) = points[i].z;
        }

        normal_vector = A.colPivHouseholderQr().solve(B);

        for (size_t j = 0; j < point_num; j++)
        {
            // 验证点到平面的距离是否在阈值范围内
            if(fabs(normal_vector(0)*points[j].x + normal_vector(1)*points[j].y + normal_vector(2)*points[j].z + 1.0) > threshold)
            {
                return false;//如果任何点的距离超过阈值,说明点集不够"平面",函数返回 false。
            }   
        }
        double normal = normal_vector.norm();
        normal_vector.normalize();
        pabcd(0) = normal_vector(0);
        pabcd(1) = normal_vector(1);
        pabcd(2) = normal_vector(2);
        pabcd(3) = 1/normal;
        return true;
    }
    //使用四元数和平移向量对3D点进行变换。
    template<typename PointType,typename T>
    PointType transformPoint(PointType point,const Eigen::Quaternion<T>& q,const Eigen::Matrix<T,3,1>& t)
    {
        Eigen::Matrix<T,3,1> ep(point.x,point.y,point.z);
        ep = q * ep + t;
        point.x = ep.x();
        point.y = ep.y();
        point.z = ep.z();
        return point;
    }
} // namespace IESKFslam
