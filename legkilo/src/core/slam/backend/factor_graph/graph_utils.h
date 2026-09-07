// SPDX-License-Identifier: MIT
// @file graph_utils.h
// @brief Graph utility functions and structures —— 因子图公共数据结构 & 四元数流形工具
// @author Ou Guangjun
// @created 2026-03-04
// @maintainer ouguangjun98@gmail.com
//
// ============================================================
// 【模块概览】graph 命名空间
// 作用：定义因子图使用的基础数据结构（位姿参数块、回环边、边键、四元数流形工具）。
// 组件：
//   - PoseParam            ：Ceres 直接读写的 (t[3], q[4]) 原始参数块
//   - QuaternionUpdateRule ：为四元数参数挂 EigenQuaternionManifold/Parameterization
//   - LoopEdge             ：回环边描述 (i, j, T_ij_meas)
//   - EdgeType             ：以无序 (i,j) 对作为哈希 key
// 【注意】四元数内存序采用 Eigen 约定 (x, y, z, w)，与 Ceres 的 EigenQuaternion* 保持一致。
// ============================================================

#ifndef LEGKILO_GRAPH_UTILS_H
#define LEGKILO_GRAPH_UTILS_H

#include "common/math_utils.hpp"

#include <ceres/ceres.h>
#if (CERES_VERSION_MAJOR >= 2)
#include <ceres/manifold.h>
#else
#include <ceres/local_parameterization.h>
#endif
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>

namespace legkilo {
namespace graph {

// ============================================================
// 【结构体】PoseParam
// 作用：SE(3) 位姿的 C 数组表示，作为 Ceres 参数块直接读写。
// 关键成员：
//   - t[3]：平移 (x, y, z)
//   - q[4]：四元数 xyzw（Eigen 内存序，与 EigenQuaternionManifold 匹配）
// 【注意】初始 q = (0,0,0,1) 即单位四元数，避免未初始化时优化跑飞。
// ============================================================
struct PoseParam {
    double t[3]{0.0, 0.0, 0.0};
    double q[4]{0.0, 0.0, 0.0, 1.0};  // xyzw  —— Eigen 内存序，Ceres AutoDiff 直接 Map

    PoseParam() = default;

    // 从 Isometry3d 构造：会自动拆解旋转/平移
    PoseParam(const Eigen::Isometry3d& iso) { this->set(iso.translation(), Eigen::Quaterniond(iso.rotation())); }

    // 平移 → Eigen::Vector3d
    Eigen::Vector3d tEigen() const { return {t[0], t[1], t[2]}; }

    // 四元数 → Eigen::Quaterniond（构造顺序是 (w, x, y, z)，因此这里把 q[3] 放在最前）
    Eigen::Quaterniond qEigen() const { return Eigen::Quaterniond(q[3], q[0], q[1], q[2]); }  // wxyz

    // 从 (t, q) 一次性覆盖内部 C 数组
    void set(const Eigen::Vector3d& tt, const Eigen::Quaterniond& qq) {
        t[0] = tt.x();
        t[1] = tt.y();
        t[2] = tt.z();
        q[0] = qq.x();
        q[1] = qq.y();
        q[2] = qq.z();
        q[3] = qq.w();
    }

    // 组装为 Eigen::Isometry3d（先平移后旋转 == translate() 后 rotate()）
    Eigen::Isometry3d toIsometry() const {
        Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
        iso.translate(this->tEigen());
        iso.rotate(this->qEigen());
        return iso;
    }
};

using EdgeType = UnorderedIntPairKey;  // 无序对键：确保 (i,j) 与 (j,i) 视为同一条边
using NodeType = int64_t;

// ============================================================
// 【结构体】LoopEdge
// 作用：一条回环边的最小描述 (from_id, to_id, T_ij)。
//       在 FactorGraph 中通过 unordered_map<EdgeType, LoopEdge> 存储，天然去重。
// ============================================================
struct LoopEdge {
    NodeType i, j;
    Eigen::Isometry3d meas;  // relative pose measurement from i to j  —— T_ij

    LoopEdge(NodeType ii, NodeType jj, const Eigen::Isometry3d& m) : i(ii), j(jj), meas(m) {}
};

// ============================================================
// 【类】QuaternionUpdateRule
// 作用：把 Ceres 的四元数流形挂到指定参数块，使优化沿 SO(3) 流形正确演进。
//       兼容 Ceres 1.x（SetParameterization）与 Ceres 2.x（SetManifold）。
// 使用方式：在 AddParameterBlock(q, 4) 之后调用 Attach(problem, q) 即可。
// ============================================================
class QuaternionUpdateRule {
   public:
    static void Attach(ceres::Problem& problem, double* q_xyzw) {
#if (CERES_VERSION_MAJOR >= 2)
        // Ceres 2.x：Manifold 是流形接口，同名 Eigen 版本假定 xyzw 内存序
        problem.SetManifold(q_xyzw, new ceres::EigenQuaternionManifold());
#else
        // Ceres 1.x：老 API，行为等价
        problem.SetParameterization(q_xyzw, new ceres::EigenQuaternionParameterization());
#endif
    }
};

}  // namespace graph
}  // namespace legkilo
#endif  // LEGKILO_GRAPH_UTILS_H
