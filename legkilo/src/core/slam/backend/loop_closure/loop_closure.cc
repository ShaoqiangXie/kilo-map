#include "core/slam/backend/loop_closure/loop_closure.h"

#include <cstdlib>

#include <glog/logging.h>
#include <pcl/io/pcd_io.h>

#include "common/timer_utils.hpp"
#include "common/yaml_helper.hpp"
#include "core/slam/backend/loop_closure/loop_detector/euclidean_method.h"
#include "kiss_matcher/KISSMatcher.hpp"
#include "small_gicp/ann/kdtree_tbb.hpp"
#include "small_gicp/factors/gicp_factor.hpp"
#include "small_gicp/points/point_cloud.hpp"
#include "small_gicp/registration/reduction_tbb.hpp"
#include "small_gicp/registration/registration.hpp"
#include "small_gicp/util/normal_estimation_tbb.hpp"

namespace legkilo {

// ---------------------------------------------------------------
// 构造：读取回环相关 yaml 参数、创建 detector、确定 PCD 目录（与 Submap 保持一致）
// ---------------------------------------------------------------
LoopClosure::LoopClosure(const std::string& config_file) {
    stopping_.store(false, std::memory_order_release);

    YamlHelper yaml(config_file);

    // [步骤 1] 欧氏候选检测参数
    search_radius_ = yaml.get<double>("loop_search_radius", 5.0);
    min_id_separation_ = yaml.get<int>("loop_min_id_separation", 5);
    // [步骤 2] GICP 几何验证参数
    icp_max_correspondence_dist_ = yaml.get<double>("loop_icp_max_corr_dist", 1.0);
    icp_inlier_dist_threshold_ = yaml.get<double>("loop_icp_inlier_dist_threshold", 0.5);
    icp_max_iterations_ = yaml.get<int>("loop_icp_max_iterations", 20);
    icp_source_inlier_ratio_threshold_ = yaml.get<double>("loop_icp_fitness_threshold", 0.7);
    icp_num_threads_ = yaml.get<int>("loop_icp_num_threads", 2);
    verify_max_attempts_ = yaml.get<int>("loop_verify_max_attempts", 2);
    // [步骤 3] KISS-Matcher 相关（可选：给远距离回环提供更强的 coarse 初值）
    loop_kiss_matcher_enabled_ = yaml.get<bool>("loop_kiss_matcher_enabled", false);
    kiss_inliers_threshold_ = yaml.get<size_t>("loop_kiss_inliers_threshold", 40);
    kiss_refine_min_id_gap_ = yaml.get<int>("loop_kiss_refine_min_id_gap", 50);
    kiss_use_quatro_ = yaml.get<bool>("loop_kiss_use_quatro", false);

    // [步骤 4] 创建候选检测器并同步参数
    detector_ = std::make_unique<EuclideanLoopDetector>();
    detector_->setSearchRadius(search_radius_);
    detector_->setMinIdSeparation(min_id_separation_);

    // [步骤 5] PCD 目录：与 Backend 中 Submap::setSavePathFolder 一致，避免读到旧路径
    const std::string res_folder = yaml.get<std::string>("temp_result_save_folder", "temp");
    submap_pcd_folder_ = std::string(ROOT_DIR) + "result/" + res_folder + "/";
}

LoopClosure::~LoopClosure() { this->stop(); }

void LoopClosure::start() { worker_ = std::thread(&LoopClosure::workerLoop, this); }

// 停止：置标志 → 唤醒 wait → join
void LoopClosure::stop() {
    stopping_.store(true, std::memory_order_release);
    cv_.notify_one();
    if (worker_.joinable()) worker_.join();
}

// ---------------------------------------------------------------
// insert：Backend 每次 finalize 后调用，投递最新优化位姿。
//   注意 workerLoop 只使用队列最后一个元素（丢弃过期快照）。
// ---------------------------------------------------------------
void LoopClosure::insert(IDPosesPtr optimized_poses) {
    if (!optimized_poses || optimized_poses->empty()) return;

    {
        std::lock_guard<std::mutex> lk(mutex_queue_);
        trigger_queue_.push_back(optimized_poses);
    }

    cv_.notify_one();
}

// ---------------------------------------------------------------
// fetchVerified：把"验证通过且尚未消费"的回环一次性取走，并置 fetched=true
//   这样 Backend 在下一次调用之前不会重复拿到相同的回环。
// ---------------------------------------------------------------
std::vector<LoopClosureOutput> LoopClosure::fetchVerified() {
    std::vector<LoopClosureOutput> outs;
    std::lock_guard<std::mutex> lk(mutex_cand_);
    for (auto& kv : history_candidates_) {
        auto& c = kv.second;
        if (c.valid && !c.fetched) {
            outs.push_back({c.i, c.j, c.meas});
            c.fetched = true;
        }
    }
    return outs;
}

// ---------------------------------------------------------------
// workerLoop：回环线程主循环
//   [1] 等待被 insert 唤醒；取最新一份 optimized_poses（老的丢弃）
//   [2] 用 detector_ 得到本轮候选对（i,j）
//   [3] 未见过的候选写入 history_candidates_，同时用两位姿之差作为初值
//   [4] 从 history_candidates_ 中挑出 attempts 未超限、尚未 valid 的进行验证
//   [5] 依次跑 verify()（耗时，故不持锁）
//   [6] 把 attempts / valid / meas 写回 history_candidates_
// 【要点】cur_input->at(id) 假设 IDPoses 是按 id 有序连续排列，与 Backend 生成方式一致；
//         若断言不成立会 OOB，需在此层保证 IDPoses 完整。
// ---------------------------------------------------------------
void LoopClosure::workerLoop() {
    while (true) {
        IDPosesPtr cur_input;

        // [步骤 1] 从触发队列获取最新一次的位姿快照
        {
            std::unique_lock<std::mutex> lk(mutex_queue_);
            cv_.wait(lk, [&] { return stopping_.load(std::memory_order_acquire) || !trigger_queue_.empty(); });
            if (!trigger_queue_.empty()) {
                if (trigger_queue_.size() > 1) {
                    // 若堆积说明验证跟不上后端节奏（GICP 太慢或子地图产出太快）
                    LOG(WARNING) << "LoopClosure worker is falling behind! Queue size: " << trigger_queue_.size();
                }
                cur_input = trigger_queue_.back();  // 只取最新
                trigger_queue_.clear();             // 丢弃过期快照
            }
        }

        if (stopping_.load(std::memory_order_acquire)) break;
        if (!cur_input) continue;

        // [步骤 2] 生成候选：距离近且 ID 相隔远（详见 euclidean_method.cc）
        const auto detect_result = detector_->detectLatest(*cur_input);
        std::vector<Candidate> verified_candidates;

        // [步骤 3~4] 更新候选表 + 挑出待验证的
        {
            std::lock_guard<std::mutex> lk(mutex_cand_);
            for (const auto& ij : detect_result) {
                UnorderedIntPairKey key(ij.first, ij.second);
                if (history_candidates_.find(key) == history_candidates_.end()) {
                    Candidate c;
                    c.i = ij.first;
                    c.j = ij.second;
                    // 用当前优化后位姿之差作为几何验证的初值 T_ij_init = T_i^-1 * T_j
                    c.meas = cur_input->at(ij.first).second.inverse() *
                             cur_input->at(ij.second).second;  // initial guess from optimized poses
                    history_candidates_.insert({key, c});
                }
            }

            // 选出：还没通过 && 尝试次数未超限 —— 拷一份到 verified_candidates 避免持锁跑 verify
            for (const auto& kv : history_candidates_) {
                const auto& c = kv.second;
                if (c.valid || c.attempts >= verify_max_attempts_) continue;
                verified_candidates.push_back(c);
            }
        }

        if (verified_candidates.empty()) continue;

        LOG(INFO) << "Verifying candidate number: " << verified_candidates.size();

        // [步骤 5] 逐个跑 verify（GICP + 可选 KISS-Matcher），不持锁
        for (auto& c : verified_candidates) {
            Timer::measure("LoopClosure::verify", [&]() { c.valid = this->verify(c.i, c.j, c.meas); });
        }

        // [步骤 6] 把结果写回 history_candidates_（重新持锁）
        {
            std::lock_guard<std::mutex> lk(mutex_cand_);
            for (const auto& c : verified_candidates) {
                UnorderedIntPairKey key(c.i, c.j);
                if (history_candidates_.find(key) != history_candidates_.end()) {
                    history_candidates_[key].attempts += 1;
                    history_candidates_[key].valid = c.valid;
                    history_candidates_[key].meas = c.meas;   // 覆写为验证后的精细相对位姿
                }
            }
        }
    }
}

// ---------------------------------------------------------------
// verify：对候选 (i,j) 做几何验证
// 【算法要点】
//   [1] 从磁盘加载 submap_i.pcd（target） 与 submap_j.pcd（source）
//   [2] 构造 small_gicp::PointCloud + TBB KdTree + 协方差
//   [3] run_small_gicp(initial_guess) → result1
//   [4] 若开启 KISS-Matcher 且 ID 相距足够远 → 用其估计一个 T_kiss，再跑一次 GICP → result2
//   [5] 在 result1/result2 中挑更优者（converged + error 更小 + inlier 比更高）
//   [6] 通过条件：converged && source_inlier_ratio >= icp_source_inlier_ratio_threshold_
// 【注意】selected_result.T_target_source 即为 T_ij，
//         因为 target=i, source=j，从 source 到 target 就是 j 在 i 系下的位姿。
// ---------------------------------------------------------------
bool LoopClosure::verify(NodeType i, NodeType j, Eigen::Isometry3d& T_ij_out) {
    // Load submap clouds
    // todo: lru cache
    // [步骤 1] 从磁盘加载 i、j 的子地图 PCD
    const std::string fi = submap_pcd_folder_ + "submap_" + std::to_string(i) + ".pcd";
    const std::string fj = submap_pcd_folder_ + "submap_" + std::to_string(j) + ".pcd";

    CloudPtr cloud_i = pcl_utils::makeCloud<PointType>();
    CloudPtr cloud_j = pcl_utils::makeCloud<PointType>();
    if (pcl::io::loadPCDFile(fi, *cloud_i) != 0) return false;
    if (pcl::io::loadPCDFile(fj, *cloud_j) != 0) return false;
    if (cloud_i->empty() || cloud_j->empty()) return false;

    // [步骤 2] 转成 small_gicp::PointCloud（内部会带协方差和法线容器）
    auto target_vec = pcl_utils::PclCloudToVecCloud(cloud_i);
    auto source_vec = pcl_utils::PclCloudToVecCloud(cloud_j);
    const Eigen::Isometry3d initial_guess = T_ij_out;   // 初值来自 workerLoop 传入

    auto target = std::make_shared<small_gicp::PointCloud>(*target_vec);
    auto source = std::make_shared<small_gicp::PointCloud>(*source_vec);

    // KdTree（TBB 并行构建）
    auto target_tree =
        std::make_shared<small_gicp::KdTree<small_gicp::PointCloud>>(target, small_gicp::KdTreeBuilderTBB());
    auto source_tree =
        std::make_shared<small_gicp::KdTree<small_gicp::PointCloud>>(source, small_gicp::KdTreeBuilderTBB());

    // 每个点计算局部协方差（k=10 近邻），供 GICP 使用
    small_gicp::estimate_covariances_tbb(*target, *target_tree, 10);
    small_gicp::estimate_covariances_tbb(*source, *source_tree, 10);

    // [步骤 3] GICP 配准函数（lambda，闭包引用参数）
    auto run_small_gicp = [&](const Eigen::Isometry3d& init_T) {
        small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionTBB> reg;
        // 对应点距离平方阈值（reject）
        reg.rejector.max_dist_sq = icp_max_correspondence_dist_ * icp_max_correspondence_dist_;
        // 内点距离平方阈值（用于 inlier 比统计）
        reg.inlier_max_dist_sq = icp_inlier_dist_threshold_ * icp_inlier_dist_threshold_;
        reg.optimizer.max_iterations = icp_max_iterations_;
        // align 返回 T_target_source，即 source → target 的变换
        return reg.align(*target, *source, *target_tree, init_T);
    };

    // 用优化后位姿差作为初值跑一次 GICP
    auto result1 = run_small_gicp(initial_guess);
    auto selected_result = result1;   // 缺省选 result1
    bool used_result2 = false;

    // [步骤 4] 若启用 KISS-Matcher 且回环 ID 间隔足够大（远距回环更需要 coarse 初值）
    const auto id_gap = static_cast<int64_t>(std::llabs(static_cast<long long>(i - j)));
    bool kiss_checked = false;
    bool kiss_passed = false;
    size_t kiss_final_inliers = 0;
    small_gicp::RegistrationResult result2; 



    if (loop_kiss_matcher_enabled_ && id_gap > kiss_refine_min_id_gap_) {
        kiss_checked = true;

        // KISS-Matcher 配置：0.3 m 体素采样，可选启用 Quatro（更鲁棒的初始化）
        kiss_matcher::KISSMatcherConfig kiss_config;
        kiss_config.use_voxel_sampling_ = true;
        kiss_config.voxel_size_ = 0.3; 
        kiss_config.use_quatro_ = kiss_use_quatro_;
        kiss_matcher::KISSMatcher matcher(kiss_config);
        // estimate(source, target) 返回 source → target 的变换（与 GICP 语义一致）
        const auto kiss_solution = matcher.estimate(*source_vec, *target_vec);
        kiss_final_inliers = matcher.getNumFinalInliers();
        kiss_passed = kiss_solution.valid && kiss_final_inliers >= kiss_inliers_threshold_;

        if (kiss_passed) {
            // 用 KISS-Matcher 输出组装 T_kiss，再跑一次 GICP 精化
            Eigen::Isometry3d T_kiss = Eigen::Isometry3d::Identity();
            T_kiss.linear() = kiss_solution.rotation;
            T_kiss.translation() = kiss_solution.translation;

            result2 = run_small_gicp(T_kiss);
            //结果2收敛并且结果1没有收敛，或者结果2的误差小于结果1并且结果2的源内点比例大于结果1
            // [步骤 5] result2 更优的判定：
            //   a) result2 收敛 且
            //   b) 要么 result1 未收敛，要么 (result2.error 更小 && result2.inlier_ratio 更高)
            const bool result2_is_better =
                result2.converged &&
                (!result1.converged ||
                 (result2.error < result1.error && result2.source_inlier_ratio > result1.source_inlier_ratio));

            if (result2_is_better) {
                selected_result = result2;
                used_result2 = true;
            }
        }
    }

    // [步骤 6] 最终输出：T_ij = T_target_source（target=i, source=j）
    T_ij_out = selected_result.T_target_source;
    // 通过标准：收敛 + 源云内点比例达标
    const bool verified =
        selected_result.converged && selected_result.source_inlier_ratio >= icp_source_inlier_ratio_threshold_;

    LOG(INFO) << "LoopClosure verify [" << i << ", " << j << "]"
              << " id_gap=" << id_gap << " result1_converged=" << result1.converged
              << " result1_error=" << result1.error << " result1_num_inliers=" << result1.num_inliers
              << " result1_source_inlier_ratio=" << result1.source_inlier_ratio << " kiss_checked=" << kiss_checked
              << " kiss_passed=" << kiss_passed << " kiss_final_inliers=" << kiss_final_inliers
              << " result2_converged=" << result2.converged << " result2_error=" << result2.error
              << " result2_num_inliers=" << result2.num_inliers
              << " result2_source_inlier_ratio=" << result2.source_inlier_ratio
              << " selected=" << (used_result2 ? "result2" : "result1")
              << " min_source_inlier_ratio=" << icp_source_inlier_ratio_threshold_ << " verified=" << verified;

    return verified;
}

}  // namespace legkilo
