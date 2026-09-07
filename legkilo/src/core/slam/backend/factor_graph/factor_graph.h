// SPDX-License-Identifier: MIT
// @file factor_graph.h
// @brief Factor graph representation and manipulation —— 基于 Ceres 的 6-DOF PoseGraph
// @author Ou Guangjun
// @created 2026-03-04
// @maintainer ouguangjun98@gmail.com
//
// ============================================================
// 【模块概览】legkilo::FactorGraph
// 作用：以 Submap 为节点、里程计/回环相对位姿为边，构建 6-DOF 位姿图并用 Ceres 求解。
// 优化变量：每个节点由 (t[3], q[4]) 参数化，四元数通过 EigenQuaternionManifold 更新。
// 边类型：
//   1) 相邻/隔一 Submap 的里程计边（addOdometryEdgesForNewestNode，硬约束但 sigma 较小）
//   2) 回环边（addLoopClosureEdge，sigma 更大 + CauchyLoss 鲁棒核，抗错误回环）
// 求解：SPARSE_NORMAL_CHOLESKY，单线程；每次 addNode 后由外部 backend 触发 optimize。
// 结果保存：saveG2o 写 g2o 文件用于离线检查。
// ============================================================

#ifndef LEGKILO_FACTOR_GRAPH_H
#define LEGKILO_FACTOR_GRAPH_H

#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "common/math_utils.hpp"
#include "core/slam/backend/factor_graph/ceres_factor.h"
#include "core/slam/backend/factor_graph/graph_utils.h"

namespace legkilo {

// ============================================================
// 【类】FactorGraph
// 作用：封装 ceres::Problem，实现基于 6-DOF 相对位姿的 PoseGraph 优化。
// 关键成员：
//   - poses_        ：节点 ID → 优化变量 PoseParam（Ceres 直接读写这块内存）
//   - initial_poses_：节点 ID → 添加时的初始位姿（用于构造里程计边、g2o 参考）
//   - loop_edges_   ：已加入的回环边（去重用；key 为无序 (i,j) 对）
//   - edges_        ：所有边的快照（供 saveG2o 序列化）
//   - problem_      ：Ceres 求解器 Problem
// 使用方式：Backend 每完成一个 Submap → addNode(id, T_wsubmap)（内部自动挂里程计边）→
//           optimize() → getPose(id, T_opt) 逐个回写。
//           回环检测通过 → addLoopClosureEdge(i, j, T_ij_meas) → optimize()。
// ============================================================
class FactorGraph {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using NodeType = int64_t;

    /**
     * @brief 构造函数：读取 yaml 中所有优化相关参数，创建空的 ceres::Problem。
     * @param yaml_file 后端配置文件路径（与 Backend 共用一份 yaml）
     */
    FactorGraph(const std::string& yaml_file);

    ~FactorGraph();

    /**
     * @brief 向因子图新增一个节点（对应一个已完成的 Submap）。
     * @param id           节点 ID（子地图 ID）
     * @param initial_pose 节点初始位姿 T_wsubmap
     *
     * 【算法要点】
     *  - 步骤 1: 若 id 已存在则直接返回（避免重复参数块）
     *  - 步骤 2: 把 initial_pose 拷贝入 poses_/initial_poses_ 并注册到 Ceres 参数块
     *  - 步骤 3: 挂 EigenQuaternionManifold 到 q 参数（保证优化在流形上进行）
     *  - 步骤 4: 若这是第一个节点，把 t、q 都设为 constant（gauge 固定）
     *  - 步骤 5: 调 addOdometryEdgesForNewestNode() 挂两条里程计边（与前一/前前节点）
     */
    void addNode(NodeType id, const Eigen::Isometry3d& initial_pose);

    /**
     * @brief 新增一条回环边（i → j 的相对位姿测量）。
     * @param meas T_ij，回环 verify 返回的两子地图之间的相对位姿
     *
     * 【要点】
     *  - 使用比里程计更大的 sigma（loopclosure_*_multiplier_ 放大）
     *  - 加上 CauchyLoss(loopclosure_loss_scale_) 抑制错误回环的影响
     *  - loop_edges_ 内部去重，同一 (i,j) 只会加入一次
     */
    void addLoopClosureEdge(NodeType i, NodeType j, const Eigen::Isometry3d& meas);

    /**
     * @brief 求解：调用 ceres::Solve；结束后写出 g2o 快照。
     */
    void optimize();

    /**
     * @brief 读取指定节点当前的优化后位姿。
     * @return 找不到 id 时返回 false。
     */
    bool getPose(NodeType node_id, Eigen::Isometry3d& pose) const;

    /**
     * @brief 把当前所有节点/边序列化为 g2o 文本文件（VERTEX_SE3:QUAT + EDGE_SE3:QUAT）。
     */
    bool saveG2o(const std::string& file_path) const;

   private:

   // 边快照结构，只用于 saveG2o；实际参与优化的是 ceres 内部的 ResidualBlock。
   struct StoreEdge {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        NodeType i;                                  // from 节点 ID
        NodeType j;                                  // to   节点 ID
        Eigen::Isometry3d meas;                      // T_ij 相对位姿测量
        Eigen::Matrix<double, 6, 6> sqrt_information;// 平方根信息矩阵（=对角 1/sigma）
   };
    /**
     * @brief 内部通用：添加一条相对位姿约束到 ceres::Problem。
     * @param trans_sigma   平移测量标准差 [m]
     * @param rot_sigma_deg 旋转测量标准差 [deg]
     * @param loss_function 可选鲁棒核；nullptr 表示纯 L2
     *
     * 【注意】meas 会被封装进 graph::RelativePoseFactor，参数块顺序 (t_i,q_i,t_j,q_j)。
     */
    void addRelativeEdge(NodeType i, NodeType j, const Eigen::Isometry3d& meas, double trans_sigma,
                         double rot_sigma_deg, ceres::LossFunction* loss_function = nullptr);

    /**
     * @brief 为最新加入的节点自动挂里程计边。
     *  - 若已有 ≥2 个节点：挂一条 prev → newest 的边（近距 sigma）
     *  - 若已有 ≥3 个节点：再挂一条 prev2 → newest 的边（sigma 放大 next_multiplier，作为"跳一跃"约束）
     */
    void addOdometryEdgesForNewestNode();

    std::map<NodeType, graph::PoseParam> poses_;  // optimized poses  —— Ceres 直接读写的参数块

    std::map<NodeType, Eigen::Isometry3d> initial_poses_;  // fixed  —— 添加时的初始位姿快照，永不变

    // 回环边去重容器；key 使用无序 (i,j) 对以避免 (a,b) 与 (b,a) 双写
    std::unordered_map<graph::EdgeType, graph::LoopEdge, graph::EdgeType::Hasher> loop_edges_;
    std::vector<NodeType> node_ids_;             // 按添加顺序保存 ID，便于查找 prev/prev2
    std::unique_ptr<ceres::Problem> problem_;    // Ceres 优化问题

    // ===== 优化参数（yaml 可覆盖，构造时读入）=====
    size_t max_iterations_ = 100;                            // Ceres 最大迭代次数
    double odom_near_trans_sigma_ = 0.1;//相邻子地图之间的平移测量，典型误差大约在 10 cm 这个量级。
    double odom_near_rot_sigma_deg_ = 5.0;//相邻子地图之间的旋转测量，典型误差大约在 5° 这个量级。
    double odom_next_trans_sigma_multiplier_ = 2.0;          // "跳一跃"里程计边平移 sigma 放大倍数
    double odom_next_rot_sigma_multiplier_ = 2.0;            // "跳一跃"里程计边旋转 sigma 放大倍数
    double loopclosure_trans_sigma_multiplier_ = 10.0;       // 回环边平移 sigma 放大倍数（不确定性更大）
    double loopclosure_rot_sigma_multiplier_ = 4.0;          // 回环边旋转 sigma 放大倍数
    double loopclosure_loss_scale_ = 3.0;                    // CauchyLoss 尺度（越小越"温柔"抑制异常残差）
    ceres::LinearSolverType linear_solver_type_ = ceres::SPARSE_NORMAL_CHOLESKY;  // 稀疏 Cholesky，适合 PoseGraph

    std::string g2o_output_path_;                            // saveG2o 输出路径（构造时确定）
    std::vector<StoreEdge> edges_;                           // 边快照，仅供 saveG2o 使用

};

}  // namespace legkilo
#endif  // LEGKILO_FACTOR_GRAPH_H
