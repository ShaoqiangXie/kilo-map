// SPDX-License-Identifier: MIT
// @file backend.h
// @brief Backend processing —— SLAM 后端主控模块（子地图管理 + 因子图优化 + 回环）
// @author Ou Guangjun
// @created 2026-01-28
// @maintainer ouguangjun98@gmail.com
//
// ============================================================
// 【模块概览】legkilo::Backend
// 作用：接收前端里程计输出（关键帧点云 + 位姿），组装成 Submap，
//       构建因子图（Ceres），并异步驱动回环检测，最终得到全局一致的轨迹与地图。
// 处理管线（单个后端工作线程 workerLoop）：
//   前端帧入队 addFrame → 关键帧判定 → 加入当前 Submap → Submap 收尾（downsample + savePCD）
//   → 因子图 addNode + addOdometryEdges → optimize → 通知 LoopClosure
//   → 空闲/停止时 finalize → 回环 fetchAndApplyLoopClosures → 再 optimize
// 关键依赖：Submap、FactorGraph、LoopClosure、ViewerSlamInterface
// ============================================================

#ifndef LEGKILO_BACKEND_H
#define LEGKILO_BACKEND_H

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "common/pcl_types.h"
#include "core/slam/backend/loop_closure/loop_closure.h"
#include "core/slam/backend/submap.h"

namespace legkilo {
class ViewerSlamInterface;
class FactorGraph;
}  // namespace legkilo

namespace legkilo {
// 因子图 / 子地图共享的节点 ID 类型（每个 Submap 一个 ID）
using NodeType = int64_t;
// (子地图 ID, 该子地图优化后的全局位姿) 列表，用于批量同步优化结果
using IDPoses = std::vector<std::pair<NodeType, Eigen::Isometry3d>>;
using IDPosesPtr = std::shared_ptr<IDPoses>;

// ============================================================
// 【类】Backend
// 作用：SLAM 后端总控器，串联"关键帧筛选 → 子地图组装 → 因子图优化 → 回环修正"整条流水线。
// 关键成员：
//   - factor_graph_    : Ceres 因子图，负责所有子地图位姿的联合优化
//   - loop_closure_    : 回环检测器（可选，通过 yaml 开关）
//   - current_submap_  : 正在构建的子地图；满帧数/满距离/空闲超时时收尾
//   - finished_submaps_: 已经完成、准备参与全局优化的子地图集合
//   - worker_thread_   : 后端异步工作线程（workerLoop），前端只往队列里 push
// 使用方式：主流程构造 Backend → start() 启动工作线程 → 每帧 addFrame() →
//           结束时 stop()（析构函数也会兜底调用）。
// ============================================================
class Backend {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // 前端每帧送来的最小数据包（放入内部队列等待后端消费）
    struct FramePacket {
        CloudPtr cloud;                                              // 机体坐标系下的当前帧点云
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();      // 前端估计的世界系位姿
        double timestamp = 0.0;                                      // 帧时间戳（秒）
        LidarMatchTypesPtr match_types = nullptr;                    // 前端 ICP 匹配类型（可选，供可视化用）
    };

    // 后端行为参数（由 yaml 读入，见 backend.cc 构造函数）
    struct Config {
        double kf_trans_threshold = 0.3;          // 关键帧筛选的平移阈值 [m]，位移超过此值即认为是关键帧
        double kf_degree_threshold = 5.0;         // 关键帧筛选的旋转阈值 [deg]
        size_t kf_max_num_submap = 50;            // 单个子地图最多容纳的关键帧数
        double kf_max_dist_submap = 5.0;          // 子地图首尾关键帧最大距离 [m]，超过则收尾
        double submap_idle_finish_timeout = 1.0;  // 空闲多少秒后强制收尾当前子地图 [sec]
    };

    /**
     * @brief 构造后端：读取 yaml 参数、创建因子图、按需创建回环检测器。
     * @param yaml_file 后端配置文件路径（含关键帧阈值、子地图上限、回环开关等）
     *
     * 【要点】
     *  - 会创建结果保存目录并清空旧内容；SLAMResultRecorder 也在这里初始化。
     *  - loop_closure_ 是否创建，取决于 yaml 的 loop_closure_enable。
     */
    explicit Backend(const std::string& yaml_file);

    /**
     * @brief 析构函数：调用 stop() 保证工作线程和回环线程正常退出。
     */
    ~Backend();

    /**
     * @brief 主线程向后端投递一帧数据（无阻塞，仅入队）。
     * @param frame       机体坐标系下的点云
     * @param pose        前端给出的世界系位姿 T_wb
     * @param timestamp   帧时间戳（秒）
     * @param match_types 每个点的匹配类型（可选，供可视化模块使用）
     *
     * 【注意】此函数是前端与后端的唯一入口，线程安全，通过 cv_ 唤醒后端线程。
     */
    void addFrame(const CloudPtr& frame, const Eigen::Isometry3d& pose, double timestamp,
                  LidarMatchTypesPtr match_types = nullptr);

    /**
     * @brief 启动后端工作线程与回环线程。
     */
    void start();

    /**
     * @brief 请求停止，通知条件变量并 join 所有子线程。
     */
    void stop();

    /**
     * @brief 注入可视化模块指针；后端在处理关键帧、完成子地图、加边等时会向其推送数据。
     */
    void setViewerInterface(ViewerSlamInterface* viewer_interface);

   private:
    /**
     * @brief 后端工作线程主体。
     *
     * 【算法要点】
     *  - 步骤 1: 用 cv_.wait_for() 等待 addFrame 唤醒，或触发 idle 超时。
     *  - 步骤 2: 依次处理本轮取出的所有 FramePacket；对每一帧：
     *              a) 若尚无 current_submap_，则新建一个；
     *              b) 若是关键帧则 addFrame 到当前子地图；
     *              c) 若子地图达到帧数/距离上限则 finalizeCurrentSubmap。
     *  - 步骤 3: 若发生 idle 超时或正在停止，则强制收尾当前子地图。
     *  - 步骤 4: 拉取回环并应用（fetchAndApplyLoopClosures），刷新可视化。
     */
    void workerLoop();

    /**
     * @brief 判断当前帧位姿相对上一个关键帧是否满足关键帧条件。
     * @return true 表示应作为关键帧，同时更新 last_kf_pose_。
     *
     * 【判定规则】平移 > kf_trans_threshold 或 旋转角度 > kf_degree_threshold 时成立；
     *              首帧无条件视为关键帧。
     */
    bool isKeyFrame(const Eigen::Isometry3d& current_pose);

    /**
     * @brief 判断给定子地图是否达到收尾条件（帧数达上限 或 首尾距离超阈值）。
     */
    bool isSubmapFinished(const SubmapPtr& submap) const;

    /**
     * @brief 收尾当前子地图：
     *   1) setFinished 触发降采样并保存 PCD；
     *   2) 加入 finished_submaps_；
     *   3) 向因子图 addNode 并 optimize；
     *   4) 把优化后的位姿回写各子地图，并通知可视化 / SLAMResultRecorder / LoopClosure。
     */
    void finalizeCurrentSubmap();

    /**
     * @brief 从 loop_closure_ 拉取已验证通过的回环，加入因子图并再次优化。
     *
     * 【注意】只有回环模块被开启时才被调用（在 workerLoop 中判空）。
     */
    void fetchAndApplyLoopClosures();

    /**
     * @brief 把所有 finished_submaps_ 的当前状态（PCD 路径 / 原始位姿 / 优化后位姿）
     *        同步给 SLAMResultRecorder，并 flush 到磁盘。
     */
    void updateRecordedSubmaps();

   private:
    Config config_;  // 后端配置项（关键帧/子地图阈值、空闲超时等）

    // ===== 子地图管理 =====
    std::mutex mutex_finished_submaps_;                  // 保护 finished_submaps_（后端线程与回环反馈路径都会访问）
    std::map<NodeType, SubmapPtr> finished_submaps_;     // 按 ID 排序保存全部已完成子地图
    SubmapPtr current_submap_;                           // 正在构建的子地图（仅工作线程访问）

    // ===== 后端工作线程与生产者-消费者队列 =====
    std::mutex mutex_queue_;                             // 保护 frame_queue_（addFrame 与 workerLoop 之间）
    std::condition_variable cv_;                         // 用于 addFrame 唤醒 workerLoop（也参与 idle 超时）
    std::deque<FramePacket> frame_queue_;                // 前端投递的原始帧队列
    std::thread worker_thread_;                          // 后端主线程
    std::atomic<bool> stopping_{false};                  // 停止标志（stop() 置位，workerLoop 检测退出）

    // ===== 可视化接口 =====
    ViewerSlamInterface* viewer_interface_ = nullptr;    // 由外部注入，不拥有生命周期

    // ===== 因子图 =====
    std::unique_ptr<FactorGraph> factor_graph_;          // 每个已完成 Submap 都是图上一个节点

    // ===== 回环检测 =====
    // 定义指针，此时还是空指针
    // yaml 中 loop_closure_enable=false 时保持为空，后端不会跑回环。
    std::unique_ptr<LoopClosure> loop_closure_;

    // ===== 关键帧筛选运行时变量 =====
    bool is_first_frame_ = true;             // 首帧标志，用于无条件把第一帧当作关键帧
    Eigen::Isometry3d last_kf_pose_;         // 上一个关键帧位姿，用于计算相对位移/旋转
};

}  // namespace legkilo

#endif  // LEGKILO_BACKEND_H
