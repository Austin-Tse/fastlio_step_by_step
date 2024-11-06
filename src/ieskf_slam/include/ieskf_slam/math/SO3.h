#pragma once
#include "Eigen/Dense"


namespace IESKFSlam
{
    //计算向量的反对成矩阵
    inline Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d &so3)
    {
        Eigen::Matrix3d so3_skew_sym;
        so3_skew_sym.setZero();
        so3_skew_sym(0,1) = -1 * so3(2);
        so3_skew_sym(0,2) = so3(1);
        so3_skew_sym(1,0) = so3(2);
        so3_skew_sym(1,2) = -1* so3(0);
        so3_skew_sym(2,0) = -1 * so3(1);
        so3_skew_sym(2,1) = so3(0);

        return so3_skew_sym;

    }

    inline Eigen::Matrix3d so3Exp(const Eigen::Vector3d &so3){
        Eigen::Matrix3d SO3;
        double so3_norm = so3.norm();
        if(so3_norm<=0.0000001)
        {
            SO3.setIdentity();
            return SO3;
        }
        //计算反对称矩阵
        Eigen::Matrix3d so3_skew_sym = skewSymmetric(so3);//将三维向量转换为反对称矩阵
        SO3 = Eigen::Matrix3d ::Identity()+(so3_skew_sym/so3_norm)*sin(so3_norm)+(so3_skew_sym*so3_skew_sym)/(so3_norm*so3_norm)*(1-cos(so3_norm));
        return SO3;
    }

    inline Eigen::Matrix3d A_T(const Eigen::Vector3d &v)//TODO
    {
        Eigen::Matrix3d res;
        double squaredNorm = v[0] * v[0]+v[1]*v[1]+v[2]*v[2];
        double norm = std::sqrt(squaredNorm);
        if(norm<1e-11){
            return Eigen::Matrix3d::Identity();
        }else{
            res = Eigen::Matrix3d ::Identity()+(1-std::cos(norm))/squaredNorm* skewSymmetric(v)+(1-std::sin(norm)/norm)/squaredNorm*skewSymmetric(v)*skewSymmetric(v);

        }
        return  res;
    }
}