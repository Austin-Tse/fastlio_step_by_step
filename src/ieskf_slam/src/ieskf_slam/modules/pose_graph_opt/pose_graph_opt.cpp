#include "ieskf_slam/modules/pose_graph_opt/pose_graph_opt.h"
#include <ceres/ceres.h>
#include <ieskf_slam/modules/pose_graph_opt/pose_graph_3d_error_term.h>

namespace IESKFSlam
{
    bool PoseGraphOpt::solve(std::vector<Pose> &poses, std::vector<BinaryEdge> &bes)
    {
        if (poses.size() <= 10)
        {
            return false;
        }

        // 构建优化问题
        ceres::Problem *problem = new ceres::Problem();
        // 核函数, 这里就不用核函数了
        ceres::LossFunction *loss_function = nullptr;
        // 四元数局部参数化，指定四元数的求导方式
        ceres::LocalParameterization *quaternion_local_parameterization = new ceres::EigenQuaternionParameterization();

        // 1. 添加帧间约束
        for (int i = 0; i < poses.size() - 1; i++)
        {
            Pose constraint;
            // 计算帧间相对位姿，From k+1帧 to k帧，T_c = T_{k}^{-1}T_{k+1}
            constraint.rotation = poses[i].rotation.conjugate() * poses[i + 1].rotation;
            constraint.position = poses[i].rotation.conjugate() * (poses[i + 1].position - poses[i].position);

            // 创建残差块
            ceres::CostFunction *cost_function = PoseGraph3dErrorTerm::Create(constraint);
            // 添加残差块
            problem->AddResidualBlock(cost_function, loss_function, 
                                      poses[i].position.data(),
                                      poses[i].rotation.coeffs().data(), 
                                      poses[i + 1].position.data(),
                                      poses[i + 1].rotation.coeffs().data());
            
            // 对于四元数，要设定使用四元数局部参数化的求导方式
            problem->SetParameterization(poses[i].rotation.coeffs().data(), quaternion_local_parameterization);
            problem->SetParameterization(poses[i + 1].rotation.coeffs().data(), quaternion_local_parameterization);
        }

        // 对于第一个顶点，我们希望它固定
        problem->SetParameterBlockConstant(poses.front().rotation.coeffs().data());
        problem->SetParameterBlockConstant(poses.front().position.data());

        // 2. 处理回环约束
        for (auto &&be : bes)
        {
            // 创建残差块
            ceres::CostFunction *cost_function = PoseGraph3dErrorTerm::Create(be.constraint);
            // 添加残差块
            problem->AddResidualBlock(cost_function, loss_function, 
                                      poses[be.to_vertex].position.data(),
                                      poses[be.to_vertex].rotation.coeffs().data(), 
                                      poses[be.from_vertex].position.data(),
                                      poses[be.from_vertex].rotation.coeffs().data());
            
            // 对于四元数，要设定使用四元数局部参数化的求导方式
            problem->SetParameterization(poses[be.to_vertex].rotation.coeffs().data(), quaternion_local_parameterization);
            problem->SetParameterization(poses[be.from_vertex].rotation.coeffs().data(), quaternion_local_parameterization);
        }

        // 配置求解参数
        ceres::Solver::Options options;
        // 最大迭代次数
        options.max_num_iterations = 200;
        // 线性求解方式
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        ceres::Solver::Summary summary;
        // 求解
        ceres::Solve(options, problem, &summary);
        // 输出求解结果
        std::cout << summary.BriefReport() << std::endl;
        
        delete problem;
        delete quaternion_local_parameterization;
        
        return summary.termination_type == ceres::CONVERGENCE;
    }
}