// SPDX-License-Identifier: MIT
// @file ceres_factor.h
// @brief ceres factor definitions —— SE(3) 相对位姿 Ceres 因子
// @author Ou Guangjun
// @created 2026-03-04
// @maintainer ouguangjun98@gmail.com
//
// ============================================================
// 【模块概览】graph::RelativePoseFactor
// 作用：Ceres AutoDiff 相对位姿因子，用于 PoseGraph 中里程计 / 回环两类相对位姿约束。
// 数学：给定测量 T_ab_meas = (t_meas, q_meas)，当前估计 T_ab_est = (q_a^-1(t_b-t_a), q_a^-1 * q_b)
//       残差 r ∈ ℝ^6 定义为：
//         r_t = t_ab_est - t_meas                                (平移误差)
//         r_r = 2 * vec(q_meas * q_ab_est^-1)                    (小角度旋转误差，小角度下 ≈ Δθ)
//       最后统一左乘 sqrt(Ω) 归一化 → 最终 r 送入 0.5*||r||^2 目标函数。
// 参数块顺序（与 factor_graph.cc::addRelativeEdge 保持一致）：
//   AutoDiffCostFunction<Factor, 6, 3, 4, 3, 4>
//                              ↑  ↑  ↑  ↑  ↑
//                              残差维 t_a q_a t_b q_b
// ============================================================

#ifndef LEGKILO_CERES_FACTOR_H
#define LEGKILO_CERES_FACTOR_H

#include <array>
#include <cmath>

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "core/slam/backend/factor_graph/graph_utils.h"

namespace legkilo {
namespace graph {

// ============================================================
// 【类】RelativePoseFactor
// 作用：SE(3) 相对位姿测量的 Ceres AutoDiff 因子。
// 关键成员：
//   - trans_ / rot_       ：测量 T_ab_meas = (t_meas, q_meas)
//   - sqrt_information_   ：6×6 平方根信息矩阵（=diag(1/σ_t, 1/σ_r)）
// 使用方式：由 FactorGraph::addRelativeEdge 通过 Create() 创建 CostFunction，
//           再 AddResidualBlock(cost, loss, t_a, q_a, t_b, q_b)。
// 【注意】q 参数块布局采用 Eigen::Quaternion 内存序 (x,y,z,w)，
//         必须搭配 EigenQuaternionManifold 才能正确沿流形更新。
// ============================================================
class RelativePoseFactor {
   public:
   //对齐内存
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief 构造：拆解 Isometry3d 得到 (t_meas, q_meas)，并缓存 sqrt(Ω)。
     */
    RelativePoseFactor(const Eigen::Isometry3d& pose, const Eigen::Matrix<double, 6, 6>& sqrt_information)
        : trans_(pose.translation()), rot_(Eigen::Quaterniond(pose.rotation())), sqrt_information_(sqrt_information) {}

    /**
     * @brief Ceres AutoDiff 残差函数（模板类型 T = double 或 ceres::Jet）
     *
     * 【算法要点】
     *  - 步骤 1: 从 raw 指针 Map 得到 p_a, q_a, p_b, q_b
     *  - 步骤 2: 计算相对位姿估计：q_ab_est = q_a^-1 * q_b；p_ab_est = q_a^-1 * (p_b - p_a)
     *  - 步骤 3: 平移残差 = p_ab_est - t_meas；旋转残差 = 2 * vec(q_meas * q_ab_est^-1)
     *  - 步骤 4: 左乘 sqrt(Ω) 归一化（详见 factor_graph.cc::MakeSqrtInformation）
     */
    template <typename T>
    bool operator()(const T* const p_a_ptr, const T* const q_a_ptr, const T* const p_b_ptr, const T* const q_b_ptr,
                    T* residuals_ptr) const {
                        // 节点 a 当前估计位姿
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_a(p_a_ptr);
        Eigen::Map<const Eigen::Quaternion<T>> q_a(q_a_ptr);

                        // 节点 b 当前估计位姿
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_b(p_b_ptr);
        Eigen::Map<const Eigen::Quaternion<T>> q_b(q_b_ptr);

        // Relative transform estimate a->b


        // 乘 (R_a^T) 是为了把它转换成节点 a 坐标系表达的相对位移。
        // 对单位四元数：conjugate() == inverse()，速度更快，无浮点开销
        const Eigen::Quaternion<T> q_a_inverse = q_a.conjugate();
        const Eigen::Quaternion<T> q_ab_estimated = q_a_inverse * q_b;              // = R_a^T * R_b
        const Eigen::Matrix<T, 3, 1> p_ab_estimated = q_a_inverse * (p_b - p_a);    // = R_a^T * (t_b - t_a)

        // Orientation error
        // delta_q = q_meas * q_ab_est^-1；若估计与测量一致，则 delta_q ≈ 单位四元数，vec≈0
        const Eigen::Quaternion<T> delta_q = rot_.template cast<T>() * q_ab_estimated.conjugate();

        Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residuals_ptr);
        // 1. 平移误差 = 估计值 - 测量值
        residuals.template block<3, 1>(0, 0) = p_ab_estimated - trans_.template cast<T>();
        // 2. 旋转误差 = 2 * delta_q 的虚部 (向量部分)
        //    数学动机：单位四元数 (w, x, y, z) 在小角度下 (x, y, z) ≈ Δθ/2，
        //    因此 2*vec(delta_q) 是李代数 so(3) 上的小角度近似。
        residuals.template block<3, 1>(3, 0) = T(2.0) * delta_q.vec();

        // 3. 左乘 sqrt(Ω) 做归一化（等价于 Σ^{-1/2}）
        residuals = sqrt_information_.template cast<T>() * residuals;
        return true;
    }

    /**
     * @brief 工厂：包装成 ceres AutoDiffCostFunction。
     *   模板参数 <6, 3, 4, 3, 4> ↔ 残差 6 维，参数块 (t_a=3, q_a=4, t_b=3, q_b=4)。
     */
    static ceres::CostFunction* Create(const Eigen::Isometry3d& t_ab_measured,//传感器测量到的节点 A 到节点 B 的相对位姿。
                                       const Eigen::Matrix<double, 6, 6>& sqrt_information) {//测量的权重（平方根信息矩阵）
        return new ceres::AutoDiffCostFunction<RelativePoseFactor, 6, 3, 4, 3, 4>(
            new RelativePoseFactor(t_ab_measured, sqrt_information));
    }

   private:
    const Eigen::Vector3d trans_;                     // t_ab_meas
    const Eigen::Quaterniond rot_;                    // q_ab_meas（单位四元数）
    const Eigen::Matrix<double, 6, 6> sqrt_information_;  // sqrt(Ω) 权重矩阵
};

}  // namespace graph
}  // namespace legkilo
#endif  // LEGKILO_CERES_FACTOR_H
