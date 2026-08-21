// SPDX-License-Identifier: MIT
// @file ceres_factor.h
// @brief ceres factor definitions
// @author Ou Guangjun
// @created 2026-03-04
// @maintainer ouguangjun98@gmail.com

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

class RelativePoseFactor {
   public:
   //对齐内存
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    RelativePoseFactor(const Eigen::Isometry3d& pose, const Eigen::Matrix<double, 6, 6>& sqrt_information)
        : trans_(pose.translation()), rot_(Eigen::Quaterniond(pose.rotation())), sqrt_information_(sqrt_information) {}

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
        const Eigen::Quaternion<T> q_a_inverse = q_a.conjugate();
        const Eigen::Quaternion<T> q_ab_estimated = q_a_inverse * q_b;
        const Eigen::Matrix<T, 3, 1> p_ab_estimated = q_a_inverse * (p_b - p_a);

        // Orientation error
        const Eigen::Quaternion<T> delta_q = rot_.template cast<T>() * q_ab_estimated.conjugate();

        Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residuals_ptr);
        // 1. 平移误差 = 估计值 - 测量值
        residuals.template block<3, 1>(0, 0) = p_ab_estimated - trans_.template cast<T>();
        // 2. 旋转误差 = 2 * delta_q 的虚部 (向量部分)
        residuals.template block<3, 1>(3, 0) = T(2.0) * delta_q.vec();

        residuals = sqrt_information_.template cast<T>() * residuals;
        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Isometry3d& t_ab_measured,//传感器测量到的节点 A 到节点 B 的相对位姿。
                                       const Eigen::Matrix<double, 6, 6>& sqrt_information) {//测量的权重（平方根信息矩阵）
        return new ceres::AutoDiffCostFunction<RelativePoseFactor, 6, 3, 4, 3, 4>(
            new RelativePoseFactor(t_ab_measured, sqrt_information));
    }

   private:
    const Eigen::Vector3d trans_;
    const Eigen::Quaterniond rot_;
    const Eigen::Matrix<double, 6, 6> sqrt_information_;
};

}  // namespace graph
}  // namespace legkilo
#endif  // LEGKILO_CERES_FACTOR_H
