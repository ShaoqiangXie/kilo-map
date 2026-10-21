// SPDX-License-Identifier: MIT
// @file submap.h
// @brief Submap representation —— 子地图（关键帧点云聚合体）
// @author Ou Guangjun
// @created 2026-01-28
// @maintainer ouguangjun98@gmail.com
#ifndef LEGKILO_SUBMAP_H
#define LEGKILO_SUBMAP_H

#include <memory>
#include <vector>

#include "common/pcl_types.h"
namespace legkilo {

// ============================================================
// 【类】Submap
// 作用：将连续若干关键帧的点云在同一"子地图坐标系"下拼接，形成一个稠密局部地图。
//       子地图是因子图的一个节点，也是回环模块 verify 时用的 target/source。
// 关键成员：
//   - id_               ：全局唯一自增 ID；同时作为因子图节点 ID
//   - origin_ / origin_opti_ ：子地图原点位姿（前端初值 / 后端优化后的值）
//   - cloud_sum_        ：子地图坐标系下累积的点云（内存中；可 release 后按需 loadPCD）
//   - frame_poses_      ：每一关键帧的"前端绝对位姿"（世界系，仅用于记录/可视化）
//   - related_frame_poses_ ：每一关键帧相对 origin_ 的位姿（子地图坐标系；用于点云再变换或估首尾距离）
// 使用方式：
//   ① Backend 构造 Submap 并不断 addFrame；
//   ② 达到条件后 setFinished(true, 0.1) → 降采样 + 存 PCD；
//   ③ 释放内存后如需再次访问点云（例如回环 verify），getCloud() 会自动 loadPCD。
// 【注意】updateOriginOpti 只是更新 origin_opti_，不会改动 cloud_sum_ 的点，
//         因此对外发布点云时需外部再 applyTransform(origin_opti_)。
// ============================================================
class Submap {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // 因子图节点 ID 类型（与 Backend::NodeType 保持一致）
    using NodeType = int64_t;

    /**
     * @brief 构造函数：分配全局唯一 ID，初始化空点云。
     *
     * 【要点】此时还没有 origin_，需在第一次 addFrame 时用该帧位姿初始化 origin_/origin_opti_。
     */
    Submap() : id_(generateGlobalId()), finished_(false), cloud_sum_(new PointCloudType()) {}

    /**
     * @brief 向子地图追加一帧关键帧点云。
     * @param frame 机体系下的关键帧点云
     * @param pose  该关键帧在世界系下的位姿 T_wb
     *
     * 【算法要点】
     *  - 步骤 1: 首帧时把 pose 记为 origin_/origin_opti_（子地图坐标系）；
     *  - 步骤 2: 计算 T_sub_body = origin_^-1 * pose，把点云投影到子地图坐标系并追加到 cloud_sum_；
     *  - 步骤 3: 分别把 pose 与 T_sub_body 存入 frame_poses_ / related_frame_poses_。
     */
    void addFrame(const CloudPtr& frame, const Eigen::Isometry3d& pose);

    /**
     * @brief 收尾子地图：可选做体素降采样，随后写盘为 PCD。
     * @param downsample true 表示做体素滤波（默认）
     * @param resolution 体素分辨率（米，默认 0.1）
     *
     * 【幂等】重复调用只生效一次；后续 addFrame 会被拒绝。
     */
    void setFinished(bool downsample = true, double resolution = 0.1);

    bool isFinished() const { return finished_; }

    NodeType getId() const { return id_; }

    size_t getNumFrames() const { return frame_poses_.size(); }

    // 子地图原点（前端首帧的世界位姿，一经设置不变，用于计算相对帧变换）
    Eigen::Isometry3d getOrigin() const { return origin_; }

    // 子地图原点（后端因子图优化后的世界位姿，会随 updateOriginOpti 更新）
    Eigen::Isometry3d getOriginOpti() const { return origin_opti_; }

    // 该子地图对应 PCD 的完整保存路径（供 recorder 记录、回环 verify 时 reload）
    std::string getPCDPath() const {
        if (save_path_folder.empty()) return "";
        if (save_path_folder.back() == '/') return save_path_folder + "submap_" + std::to_string(id_) + ".pcd";
        return save_path_folder + "/submap_" + std::to_string(id_) + ".pcd";
    }

    // 所有关键帧的"前端绝对位姿"（世界系）
    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> getFramePoses() const {
        return frame_poses_;
    }

    // 所有关键帧相对 origin_ 的位姿（子地图坐标系）
    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> getRelatedFramePoses() const {
        return related_frame_poses_;
    }

    /**
     * @brief 子地图首/尾关键帧在子地图坐标系下的欧氏距离。
     *        用作 isSubmapFinished 中的"走远了"判据。
     */
    double getBeginEndFrameDistance() const;

    /**
     * @brief 后端因子图优化后回写：更新子地图原点位姿（不改变 cloud_sum_ 的点值）。
     */
    void updateOriginOpti(const Eigen::Isometry3d& new_origin) { origin_opti_ = new_origin; }

    /**
     * @brief 获取子地图点云；若已被 releaseCloud() 释放，则自动从 PCD 磁盘加载。
     */
    CloudPtr getCloud() { return cloud_sum_ ? cloud_sum_ : this->loadPCD(); }

    /**
     * @brief 释放内存点云（cloud_sum_.reset()）；不删磁盘 PCD。
     */
    void releaseCloud();

    /**
     * @brief 全局设置：所有 Submap 的 PCD 保存目录（后端启动时统一调用一次）。
     */
    static void setSavePathFolder(const std::string& dir) { save_path_folder = dir; }

   private:
    // 将当前 cloud_sum_ 写为二进制压缩 PCD；返回是否成功
    bool savePCD();

    // 从磁盘加载子地图 PCD 到 cloud_sum_
    CloudPtr loadPCD();

    // 生成全局自增 ID（进程内单调递增，不复用）
    static NodeType generateGlobalId();

   private:
    NodeType id_;  // 子地图全局唯一 ID，也是因子图节点 ID

    bool finished_;  // Whether the submap is finished  —— true 表示已 setFinished，禁止再 addFrame

    CloudPtr cloud_sum_;  // Sum of point clouds       —— 子地图坐标系下累积的点云（可能被 release）

    Eigen::Isometry3d origin_;  // Initial origin pose  —— 首帧的前端位姿（不再变化）

    Eigen::Isometry3d origin_opti_;  // Optimized origin pose  —— 优化后的原点位姿

    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>
        related_frame_poses_;  // Relative poses of frames  —— 每帧相对 origin_ 的位姿

    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>
        frame_poses_;  // frontend absolute poses of frames  —— 每帧的世界系位姿

    // 所有 Submap 共享的 PCD 保存目录（inline static 允许在头文件内定义并共享）
    inline static std::string save_path_folder = "tmp";
};
using SubmapPtr = std::shared_ptr<Submap>;

}  // namespace legkilo

#endif  // LEGKILO_SUBMAP_H
