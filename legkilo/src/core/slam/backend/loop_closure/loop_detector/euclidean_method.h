// SPDX-License-Identifier: MIT
// @file euclidean_method.h
// @brief Simple loop candidates by Euclidean distance and id separation
//        ——基于欧氏距离 + ID 间隔的简单回环候选检测器
// @author Ou
//
// ============================================================
// 【模块概览】legkilo::EuclideanLoopDetector
// 作用：从优化后的子地图位姿列表中挑选出"距离足够近 + ID 相距足够远"的对，作为回环候选。
// 直觉：车辆走了一圈又回到出发点，两个子地图在空间上很近但插入时间（≈ID）相隔很远。
// 关键成员：
//   - search_radius_    ：距离阈值 [m]（3D 欧氏）
//   - min_id_separation_：ID 间隔阈值（避免误把相邻子地图当作回环）
// 使用方式：由 LoopClosure 持有；每次 Backend 更新位姿后调用 detectLatest() 取候选。
// ============================================================

#ifndef LEGKILO_LOOP_CLOSURE_LOOP_DETECTOR_EUCLIDEAN_METHOD_H
#define LEGKILO_LOOP_CLOSURE_LOOP_DETECTOR_EUCLIDEAN_METHOD_H

#include <algorithm>
#include <cstdint>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

namespace legkilo {

using NodeType = int64_t;

// ============================================================
// 【类】EuclideanLoopDetector
// 见文件顶部模块概览。
// ============================================================
class EuclideanLoopDetector {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EuclideanLoopDetector() = default;

    /**
     * @brief 设置搜索半径（下限 0.1 m，避免设 0 导致无候选）。
     */
    void setSearchRadius(double r);

    /**
     * @brief 设置 ID 间隔阈值（下限 1，避免与自身或相邻子地图匹配）。
     */
    void setMinIdSeparation(int s);

    /**
     * @brief 从优化后位姿列表中检测回环候选。
     * @param poses (id, T_world_submap) 列表
     * @return 若 (id_i, id_j) 满足 |p_i - p_j| < r 且 id_j - id_i >= min_id_separation_，
     *         则加入结果集；返回值中总有 id_i < id_j（避免重复对）。
     *
     * 【算法要点】
     *  - 步骤 1: 提取所有子地图平移向量
     *  - 步骤 2: 构建 3D KD-Tree（nanoflann）
     *  - 步骤 3: 对每个子地图做半径查询，过滤 ID 间隔不足的邻居
     */
    std::vector<std::pair<NodeType, NodeType>> detectLatest(
        const std::vector<std::pair<NodeType, Eigen::Isometry3d>>& poses) const;

   private:
    double search_radius_ = 5.0;   // 距离阈值 [m]
    int min_id_separation_ = 5;    // ID 间隔阈值
};

}  // namespace legkilo

#endif  // LEGKILO_LOOP_CLOSURE_LOOP_DETECTOR_EUCLIDEAN_METHOD_H
