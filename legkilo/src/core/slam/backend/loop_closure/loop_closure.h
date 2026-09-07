// ============================================================
// 【模块概览】legkilo::LoopClosure
// 作用：SLAM 后端的回环检测线程（独立线程）。
//       - 输入：Backend 在每次 finalize 后通过 insert() 投递所有子地图的最新优化位姿；
//       - 检测：使用 EuclideanLoopDetector（欧氏距离 + ID 间隔）挑选候选对；
//       - 验证：优先 small_gicp（GICP）；若开启 KISS-Matcher 且 ID 相距足够远，用其做 coarse 初值再跑一次 GICP；
//       - 输出：通过 fetchVerified() 返回已通过验证、尚未被消费的回环 (i, j, T_ij)。
// 【要点】
//   - 只有"距离近 且 ID 相隔远"（走了一圈又回到原地）才可能是真正的回环，
//     纯依赖距离会误判（例如短距离小回旋）。
//   - Candidate::attempts 用于限制单对候选的最大验证次数，避免无效反复计算。
// ============================================================
#ifndef LEGKILO_LOOP_CLOSURE_H
#define LEGKILO_LOOP_CLOSURE_H

#include <atomic>
#include <condition_variable>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "common/math_utils.hpp"
#include "common/pcl_types.h"

namespace legkilo {
class EuclideanLoopDetector;
}

namespace legkilo {

// 与 Backend / FactorGraph 一致的节点 ID 类型（子地图 ID）
using NodeType = int64_t;
// (子地图 ID, 优化后世界位姿) 列表；Backend 触发回环时以此为输入
using IDPoses = std::vector<std::pair<NodeType, Eigen::Isometry3d>>;
using IDPosesPtr = std::shared_ptr<IDPoses>;

// 通过验证的一条回环约束的对外输出（供 fetchVerified 返回）
struct LoopClosureOutput {
    NodeType i;
    NodeType j;
    Eigen::Isometry3d meas;  // relative pose from i to j  —— 送入 FactorGraph::addLoopClosureEdge
};

// ============================================================
// 【类】LoopClosure
// 作用：回环检测线程；输入优化后的子地图位姿，输出经几何验证的回环约束。
// 关键成员：
//   - detector_          ：欧氏候选检测器（近距离 + ID 间隔大）
//   - history_candidates_：历史候选表 (i,j)→Candidate；用 attempts 限制重复验证
//   - trigger_queue_     ：Backend 通过 insert() 投递的触发队列（最新一个即可）
//   - worker_            ：验证线程
// 使用方式：Backend 构造时 new LoopClosure → start()
//           → 每次 finalize 后 insert(optimized_poses)
//           → 一段时间后 fetchVerified() 取回环并 addLoopClosureEdge。
// ============================================================
class LoopClosure {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief 从 yaml 读取所有回环参数并构造 detector；此时线程尚未启动。
     */
    explicit LoopClosure(const std::string& config_file);

    /**
     * @brief 析构：兜底调用 stop()。
     */
    ~LoopClosure();

    /**
     * @brief 启动验证线程 workerLoop。
     */
    void start();

    /**
     * @brief 请求停止：置 stopping_ → notify_one → join worker。
     */
    void stop();

    // Input: optimized submap poses (global). Typically pass all finished submaps in order.
    /**
     * @brief Backend 每次 finalize 后调用：投递最新优化后的子地图位姿列表。
     *        队列内旧的会被覆盖（workerLoop 只取 back() 处理）。
     */
    void insert(IDPosesPtr optimized_poses);

    // Output: fetch verified loop closures (not yet fetched).
    /**
     * @brief Backend 定期调用：取出所有 valid && !fetched 的候选，
     *        取完后将其标记为 fetched，避免重复入图。
     */
    std::vector<LoopClosureOutput> fetchVerified();

   private:
    // 单个回环候选（一对子地图 i,j 的完整验证状态）
    struct Candidate {
        NodeType i;                     // 较小 ID
        NodeType j;                     // 较大 ID
        int attempts = 0;               // 已验证次数（受 verify_max_attempts_ 限制）
        bool valid = false;             // 是否已通过几何验证
        bool fetched = false;           // 是否已被 Backend 消费（fetchVerified）
        double fitness_score = 0.0;     // small_gicp 的 fitness（预留，未主动使用）
        size_t kiss_inliers = 0;        // KISS-Matcher 的最终内点数（若启用）
        Eigen::Isometry3d meas = Eigen::Isometry3d::Identity();  // i -> j 相对位姿
    };

    /**
     * @brief 验证线程主体：
     *   [1] cv_ 等待触发（或 stopping）
     *   [2] 从触发队列取最新一份 optimized_poses（老的丢弃）
     *   [3] 用 detector_ 检测候选，未见过的写入 history_candidates_
     *   [4] 对未 valid 且 attempts 未超限的候选运行 verify()
     */
    void workerLoop();

    /**
     * @brief 对给定候选 (i,j) 做几何验证。
     * @param T_ij_out 输入：初值（来自优化后位姿差）；输出：验证得到的精细相对位姿
     * @return true 表示通过（converged 且 source_inlier_ratio 达标）
     *
     * 【算法要点】
     *  - 从磁盘加载两子地图 PCD，估计法线/协方差
     *  - 先跑 small_gicp（GICP）得 result1
     *  - 若 loop_kiss_matcher_enabled_ 且 id_gap>阈值：再跑 KISS-Matcher；若通过则用其结果初值化再跑一次 GICP 得 result2
     *  - 在 result1 与 result2 中挑更好的作为最终结果
     */
    bool verify(NodeType i, NodeType j, Eigen::Isometry3d& T_ij_out);

   private:
    // ===== 配置参数（由 yaml 读入）=====
    double search_radius_ = 5.0;                    // 欧氏候选：距离阈值 [m]
    int min_id_separation_ = 5;                     // 欧氏候选：ID 间隔阈值（防止相邻子地图匹配自己）
    double icp_max_correspondence_dist_ = 1.0;      // GICP：拒绝对应点的距离阈值 [m]
    double icp_inlier_dist_threshold_ = 0.5;        // GICP：内点距离阈值 [m]
    int icp_max_iterations_ = 40;                   // GICP：最大迭代
    int icp_num_threads_ = 2;                       // GICP：TBB 线程数
    double icp_source_inlier_ratio_threshold_ = 0.5;// verify 通过的最低内点比
    bool loop_kiss_matcher_enabled_ = false;        // 是否启用 KISS-Matcher（作为 GICP 的 coarse 初值）
    size_t kiss_inliers_threshold_ = 40;            // KISS-Matcher 通过所需最少内点
    bool kiss_use_quatro_ = false;                  // KISS-Matcher 内部是否启用 Quatro（更强初始化）
    int kiss_refine_min_id_gap_ = 100;              // 只有 |i-j| 超过该值才使用 KISS-Matcher
    int verify_max_attempts_ = 2;                   // 每对候选最多验证次数

    // PCD folder
    std::string submap_pcd_folder_;                 // 子地图 PCD 存放目录（与 Submap::setSavePathFolder 一致）

    // ===== 历史候选表 =====
    std::mutex mutex_cand_;                         // 保护 history_candidates_（worker 与 fetchVerified 共享）
    std::unordered_map<UnorderedIntPairKey, Candidate, UnorderedIntPairKey::Hasher, UnorderedIntPairKey::Equal>
        history_candidates_;                        // key 无序 → (i,j)==(j,i) 视为同一候选

    // detector
    std::unique_ptr<EuclideanLoopDetector> detector_;  // 前向声明中定义，见 euclidean_method.h

    // ===== 线程与生产者-消费者队列 =====
    std::mutex mutex_queue_;                        // 保护 trigger_queue_
    std::condition_variable cv_;                    // insert 唤醒 workerLoop
    std::deque<IDPosesPtr> trigger_queue_;          // 触发队列；每次只取 back()
    std::thread worker_;                            // 验证线程
    std::atomic<bool> stopping_{false};             // 停止标志
};

}  // namespace legkilo
#endif  // LEGKILO_LOOP_CLOSURE_H
