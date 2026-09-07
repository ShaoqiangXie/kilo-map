#include "core/slam/backend/backend.h"

#include <glog/logging.h>
#include <boost/filesystem.hpp>

#include "common/file_io.hpp"
#include "common/timer_utils.hpp"
#include "common/voxel_grid.hpp"
#include "common/yaml_helper.hpp"
#include "core/slam/backend/factor_graph/factor_graph.h"
#include "core/slam/tool/slam_result_recorder.h"
#include "viewer/viewer_slam_interface.h"

namespace legkilo {

// ---------------------------------------------------------------
// 构造函数：读取 yaml → 建立结果目录 → 创建因子图 → 按需创建回环模块
// ---------------------------------------------------------------
Backend::Backend(const std::string& yaml_file) {
    // [步骤 1] 初始化状态：清停止标志、置首帧标志
    stopping_.store(false, std::memory_order_release);
    is_first_frame_ = true;

    // [步骤 2] 从 yaml 中读取后端行为参数（详见 Config 结构体注释）
    YamlHelper yaml_helper(yaml_file);
    config_.kf_trans_threshold = yaml_helper.get<double>("kf_trans_threshold", 0.3);
    config_.kf_degree_threshold = yaml_helper.get<double>("kf_degree_threshold", 5.0);
    config_.kf_max_num_submap = yaml_helper.get<size_t>("kf_max_num_submap", 50);
    config_.kf_max_dist_submap = yaml_helper.get<double>("kf_max_dist_submap", 5.0);
    // idle 超时下限 0.1 s，避免过短导致子地图频繁被切断
    config_.submap_idle_finish_timeout = std::max(yaml_helper.get<double>("submap_idle_finish_timeout", 1.0), 0.1);

    // [步骤 3] 准备本次运行的结果保存目录：确保存在 → 清空 → 让 Submap 和 Recorder 都知道
    std::string temp_result_save_folder =
        std::string(ROOT_DIR) + "result/" + yaml_helper.get<std::string>("temp_result_save_folder", "temp") + "/";
    file_io::ensureDirectory(temp_result_save_folder);
    file_io::clearDirectoryContents(temp_result_save_folder);
    Submap::setSavePathFolder(temp_result_save_folder);      // 每个子地图 PCD 都写到该目录
    SLAMResultRecorder::initialize(temp_result_save_folder, yaml_file);  // 结果记录器（TUM 轨迹 / g2o 等）

    // [步骤 4] 创建因子图（自带 Ceres::Problem，节点/边参数在 yaml 中一并读入）
    factor_graph_ = std::make_unique<FactorGraph>(yaml_file);

    // [步骤 5] 若开启回环，构造 LoopClosure 对象；否则 loop_closure_ 保持为空指针，后端会自动跳过回环相关逻辑
    bool loop_closure_enabled = yaml_helper.get<bool>("loop_closure_enable", true);
    if (loop_closure_enabled) loop_closure_ = std::make_unique<LoopClosure>(yaml_file);//创建 LoopClosure 对象，loop_closure_为非空指针
}

// ---------------------------------------------------------------
// 析构：兜底调用 stop() 保证工作线程能正常退出（避免野线程）
// ---------------------------------------------------------------
Backend::~Backend() {
    this->stop();
    LOG(INFO) << "Backend is destroyed";
}

    // 将当前帧加入后端：
    // 1. cloud_body：机体坐标系下的点云
    // 2. 当前 IMU 位姿：旋转和平移
    // 3. 当前帧时间戳
    // 4. 前端匹配类型或匹配结果

// ---------------------------------------------------------------
// addFrame：前端 → 后端的唯一入口，线程安全，非阻塞
//   将 (点云, 位姿, 时间戳, 匹配类型) 打包进 frame_queue_，并唤醒 workerLoop
// ---------------------------------------------------------------
void Backend::addFrame(const CloudPtr& frame, const Eigen::Isometry3d& pose, double timestamp,
                       LidarMatchTypesPtr match_types) {
    // [步骤 1] 输入合法性检查：空点云直接丢弃，避免污染子地图
    if (!frame || frame->empty()) return;

    // [步骤 2] 上锁入队 + 唤醒消费者线程（cv_.wait_for 会被打断，随后取出 local_queue 处理）
    std::lock_guard<std::mutex> lock(mutex_queue_);
    frame_queue_.push_back({frame, pose, timestamp, match_types});
    cv_.notify_one();
}

// ---------------------------------------------------------------
// start：先启动回环线程，再启动后端主线程；两者通过 IDPosesPtr 通信
// ---------------------------------------------------------------
void Backend::start() {
    if (loop_closure_) loop_closure_->start();
    worker_thread_ = std::thread(&Backend::workerLoop, this);
}

// ---------------------------------------------------------------
// stop：置停止标志 → 唤醒 workerLoop → join 后端线程 → 停止回环线程
// ---------------------------------------------------------------
void Backend::stop() {
    stopping_.store(true, std::memory_order_release);  // 通知 workerLoop 停止
    cv_.notify_one();                                  // 从条件变量中唤醒 workerLoop
    if (worker_thread_.joinable()) worker_thread_.join();
    if (loop_closure_) loop_closure_->stop();
}

// 注入可视化接口指针；后端不拥有其生命周期，仅通过它推送数据（关键帧、子地图、边）
void Backend::setViewerInterface(ViewerSlamInterface* viewer_interface) { viewer_interface_ = viewer_interface; }

// ---------------------------------------------------------------
// workerLoop：后端主循环（唯一在后台线程执行的函数）
//   1. 等待队列有帧 or 停止 or idle 超时
//   2. 逐帧筛选关键帧、更新当前子地图
//   3. 满足条件时收尾子地图 → addNode + optimize
//   4. 拉取回环 → 加边 + optimize
//   5. 刷新可视化，检查退出
// ---------------------------------------------------------------
void Backend::workerLoop() {
    // 将配置中的“子地图空闲完成超时”从秒转换为毫秒。
    // 在指定时间内没有收到新帧时，当前子地图会被结束。
    const auto idle_timeout = std::chrono::milliseconds(static_cast<int>(config_.submap_idle_finish_timeout * 1000));//1000毫秒
    while (true) {
        std::deque<FramePacket> local_queue;//// 临时队列：用于一次性取出共享队列里的待处理帧。
        bool timed_out = false;//        // 标记本次等待是否因超时而结束。
        {
            std::unique_lock<std::mutex> lock(mutex_queue_);


            // 等待以下任一条件成立：
            // 1. 收到后端停止信号；
            // 2. 共享帧队列中出现待处理帧；
            // 3. 等待时间超过 idle_timeout。
            //
            // wait_for() 返回 false 表示等待超时，因此这里取反后赋给 timed_out。

            // bool wait_for(lock, timeout, pred);  pred为true，那么返回true   否则等待idle_timeout 后返回false
            //等待唤醒

            timed_out = !cv_.wait_for(
                lock, idle_timeout, [&] { return stopping_.load(std::memory_order_acquire) || !frame_queue_.empty(); });
            local_queue.swap(frame_queue_);
        }
        // 依次处理本次取出的所有帧。
        for (const auto& frame : local_queue) {
            const auto& current_cloud = frame.cloud;
            const auto& current_pose = frame.pose;
            const auto& current_match_types = frame.match_types;
            // 如果当前没有正在构建的子地图，说明这是新子地图的第一帧。
            if (!current_submap_) current_submap_ = std::make_shared<Submap>();  // first frame of a new submap
            // 将当前子地图发送给可视化模块。
            // 这里会传入子地图 ID、当前位姿、点云和匹配类型。
            if (viewer_interface_) {
                viewer_interface_->insertCurrentKeyframe(current_submap_->getId(), current_pose,
                                                         pcl_utils::PclCloudToVecCloud(current_cloud),
                                                         current_match_types);


            // std::cout <<"---------------------------------------------------------------------------------"<< std::endl;
            // std::cout <<"current_submap_->getId() " <<current_submap_->getId()<< std::endl;
            // std::cout <<"---------------------------------------------------------------------------------"<< std::endl;



            }
            // 判断当前帧是否满足关键帧条件。
            const bool is_keyframe = this->isKeyFrame(current_pose);
            // 记录前端帧信息，包括时间戳、子地图 ID、位姿以及是否为关键帧。
            SLAMResultRecorder::recordFrontendFrame(frame.timestamp, current_submap_->getId(), current_pose,
                                                    is_keyframe);

            if (!is_keyframe) continue;// // 非关键帧只做记录和显示，不加入子地图。

            current_submap_->addFrame(current_cloud, current_pose);//// 将关键帧点云及其位姿加入当前子地图。
            // 如果当前子地图满足结束条件，则完成并提交当前子地图。
            if (this->isSubmapFinished(current_submap_)) { this->finalizeCurrentSubmap(); }
        }
        // 长时间没有收到新帧，或者系统正在停止时，
        // 结束当前尚未完成的子地图。
        if (timed_out || stopping_.load(std::memory_order_acquire)) { this->finalizeCurrentSubmap(); }
         // 获取并应用回环检测模块产生的位姿修正结果。
        if (loop_closure_) this->fetchAndApplyLoopClosures();
         // 通知可视化模块刷新地图、轨迹等绘制内容。
        if (viewer_interface_) viewer_interface_->refreshDrawables();
        // 再次检查停止信号。
        if (stopping_.load(std::memory_order_acquire)) {
            SLAMResultRecorder::flush();
            break;
        }
    }
}

// ---------------------------------------------------------------
// isKeyFrame：根据位姿增量决定是否作为关键帧
//   规则：|Δt| > 阈值(m) 或 |Δθ| > 阈值(deg)；首帧无条件为关键帧
// ---------------------------------------------------------------
bool Backend::isKeyFrame(const Eigen::Isometry3d& current_pose) {
    if (is_first_frame_) {
        is_first_frame_ = false;
        last_kf_pose_ = current_pose;
        return true;
    }//如果是第一帧，就设置为关键帧
    // [步骤 1] 计算相对位姿 ΔT = T_kf^-1 * T_curr（即当前位姿在上一关键帧坐标系下的表达）
    Eigen::Isometry3d relative_pose = last_kf_pose_.inverse() * current_pose;//计算当前帧相对于上一个关键帧的位姿变化
    // [步骤 2] 平移变化 = |Δt|（欧氏范数）
    double trans_diff = relative_pose.translation().norm();//计算平移变化的欧几里得距离    translation()--平移   norm()--欧几里得范数
    // [步骤 3] 旋转变化 = AxisAngle(ΔR).angle()，弧度转角度
    double rot_diff = Eigen::AngleAxisd(relative_pose.rotation()).angle() * (180.0 / M_PI);  // in degrees     //计算旋转变化的角度差，转换为角度单位

    // [步骤 4] 任一维度超阈值 → 作为关键帧，并更新 last_kf_pose_
    if (trans_diff > config_.kf_trans_threshold || rot_diff > config_.kf_degree_threshold) {   //超过0.3米或5度就认为是关键帧
        last_kf_pose_ = current_pose;
        return true;
    }

    return false;
}

// ---------------------------------------------------------------
// isSubmapFinished：子地图收尾判定
//   条件 A：关键帧数达到上限（kf_max_num_submap）
//   条件 B：首尾关键帧欧氏距离超阈值（kf_max_dist_submap）
//   注意：workerLoop 在 idle/停止时也会强制 finalize，这里只处理"帧多/走远"两种主动收尾
// ---------------------------------------------------------------
bool Backend::isSubmapFinished(const SubmapPtr& submap) const {
    bool is_finished = false;

    // 条件 A：关键帧数量超过上限
    is_finished |= submap->getNumFrames() >= config_.kf_max_num_submap;

    // 条件 B：子地图首尾关键帧距离超过阈值（避免子地图空间跨度过大导致点云投影误差累积）
    is_finished |= submap->getBeginEndFrameDistance() >= config_.kf_max_dist_submap;

    return is_finished;
}

// ---------------------------------------------------------------
// finalizeCurrentSubmap：把 current_submap_ 从"构建中"过渡到"已完成、已进入因子图"
// 主要步骤：
//   [1] 空/无帧保护
//   [2] setFinished：内部完成 0.1 m 体素降采样 + 保存 PCD（供回环 verify 时按需 reload）
//   [3] 加入 finished_submaps_
//   [4] 推送一份"轻量点云"给 viewer 用作可视化子地图
//   [5] 向因子图 addNode（同时会自动 addOdometryEdgesForNewestNode，见 factor_graph.cc）
//   [6] optimize → 把优化后的所有子地图位姿回写、通知 recorder / viewer / loop_closure
//   [7] releaseCloud 释放内存（需要时可 loadPCD 恢复）
// ---------------------------------------------------------------
void Backend::finalizeCurrentSubmap() {
    if (!current_submap_) return;
    if (current_submap_->getNumFrames() == 0) {
        LOG(WARNING) << "Skip empty submap " << current_submap_->getId() << " during finalize";
        current_submap_.reset();
        return;
    }

    // [步骤 2] 收尾子地图：内部做体素降采样并保存 PCD（第二个参数 0.1 m = 体素分辨率）
    current_submap_->setFinished(true, 0.1);//TODO

    // [步骤 3] 加入已完成子地图集合（加锁保护，回环回调路径也会访问）
    {
        std::unique_lock<std::mutex> lock(mutex_finished_submaps_);
        finished_submaps_.insert({current_submap_->getId(), current_submap_});
    }

    // [步骤 4] 推送一份粗降采样后的子地图给可视化模块（0.1 m 体素，节省带宽）
    if (viewer_interface_) {
        static constexpr float kViewerSubmapVoxelResolution = 0.1f;  // 显示用的粗采样分辨率
        VoxelGrid viewer_voxel_filter(kViewerSubmapVoxelResolution, SamplingMode::MedianRepresentative);
        CloudPtr viewer_cloud = pcl_utils::makeCloud<PointType>();
        viewer_voxel_filter.filter(current_submap_->getCloud(), viewer_cloud);

        // 送入可视化：origin_opti（当前世界系位姿）、各关键帧位姿、点云
        viewer_interface_->insertFinishedSubmap(current_submap_->getId(),
                                                {current_submap_->getOriginOpti(), current_submap_->getFramePoses(),
                                                 pcl_utils::PclCloudToVecCloud(viewer_cloud)});
    }

    // [步骤 5] 因子图：以子地图 ID 为节点 ID，初始位姿为 origin_opti
    //         同时会自动添加与前一个节点、前前节点的里程计边（见 addOdometryEdgesForNewestNode）
    factor_graph_->addNode(current_submap_->getId(), current_submap_->getOriginOpti());

    // [步骤 6a] 求解器：使用 SPARSE_NORMAL_CHOLESKY（详见 factor_graph.cc::optimize）
    Timer::measure("FactorGraph Optimize: ", [&]() { factor_graph_->optimize(); });

    IDPosesPtr optimized_poses = std::make_shared<IDPoses>();
    // 获取所有子地图优化后的位姿
    {
        std::unique_lock<std::mutex> lock(mutex_finished_submaps_);
        for (auto& [id, submap] : finished_submaps_) {//遍历所有已完成优化的子地图
            Eigen::Isometry3d optimized_pose;
            factor_graph_->getPose(id, optimized_pose);//获取优化后的位姿
            submap->updateOriginOpti(optimized_pose);//更新子地图的优化位姿
            optimized_poses->emplace_back(id, optimized_pose);//将子地图 ID 和优化后的位姿添加到 optimized_poses 中
        }
    }
    // [步骤 6b] 同步给 recorder（写盘），并让 viewer 刷新已完成子地图的位姿
    //将所有已经完成的子地图信息同步到 SLAM 结果记录器，并把最新的后端状态写入文件。
    this->updateRecordedSubmaps();
    //刷新可视化模块中已完成子地图的位姿信息，以便在可视化界面中显示优化后的位姿。
    if (viewer_interface_) { viewer_interface_->updateFinishedSubmapPose(optimized_poses); }

    // [步骤 6c] 触发回环模块：把优化后的所有子地图位姿投递给回环线程，
    //          回环线程会在其中挑选距离近 / ID 相隔远的候选做几何验证（见 loop_closure.cc）
    if (loop_closure_) loop_closure_->insert(optimized_poses);

    // [步骤 7] 释放当前子地图占用的内存点云；回环 verify 时会再从磁盘 loadPCD
    current_submap_->releaseCloud();  // release cloud to save memory
    current_submap_.reset();
}

// ---------------------------------------------------------------
// fetchAndApplyLoopClosures：把回环模块已验证通过的回环约束加入因子图并重新优化
//   算法要点：
//     - fetchVerified 返回自上次以来"新通过校验且未被消费"的回环，
//       每条回环用 (i, j, T_ij) 表达，i、j 是两个子地图节点 ID；
//     - addLoopClosureEdge 以较宽松的 sigma + CauchyLoss 加入图（详见 factor_graph.cc），
//       从而对错误回环具备一定鲁棒性；
//     - 加完后立即触发一次全图优化，然后回写并广播位姿。
// ---------------------------------------------------------------
void Backend::fetchAndApplyLoopClosures() {
    auto loops = loop_closure_->fetchVerified();//获取已验证的回环
    if (loops.empty()) return;

    // [步骤 1] 把所有回环写入因子图 + 记录 + 可视化
    for (const auto& lc : loops) {
        factor_graph_->addLoopClosureEdge(lc.i, lc.j, lc.meas);//将回环约束加入系统
        SLAMResultRecorder::recordLoopEdge(lc.i, lc.j, lc.meas);//将回环约束记录到 SLAM 结果记录器中
        if (viewer_interface_) viewer_interface_->insertEdge(lc.i, lc.j);//将回环约束可视化
    }

    // [步骤 2] 因子图重新求解（有回环后一般能大幅拉平累计漂移）
    Timer::measure("FactorGraph Optimize: ", [&]() { factor_graph_->optimize(); });

    // [步骤 3] 把优化结果回写子地图 + 组装 IDPoses 广播
    IDPosesPtr optimized_poses = std::make_shared<IDPoses>();

    {
        std::unique_lock<std::mutex> lock(mutex_finished_submaps_);
        for (auto& [id, submap] : finished_submaps_) {
            Eigen::Isometry3d optimized_pose;
            factor_graph_->getPose(id, optimized_pose);
            submap->updateOriginOpti(optimized_pose);
            optimized_poses->emplace_back(id, optimized_pose);
        }
    }

    // [步骤 4] 写盘 + 通知 viewer 刷新位姿
    this->updateRecordedSubmaps();

    if (viewer_interface_) { viewer_interface_->updateFinishedSubmapPose(optimized_poses); }
}
//将所有已经完成的子地图信息同步到 SLAM 结果记录器，并把最新的后端状态写入文件。
// ---------------------------------------------------------------
// updateRecordedSubmaps：把所有已完成子地图（ID / PCD 路径 / 原始位姿 / 优化后位姿）
//                       同步写入 SLAM 结果记录器；随后强制 flush 后端状态到磁盘
// ---------------------------------------------------------------
void Backend::updateRecordedSubmaps() {
    Timer::measure("SLAM Result Recorder: ", [&]() {
        std::unique_lock<std::mutex> lock(mutex_finished_submaps_);
        for (const auto& [id, submap] : finished_submaps_) {
            if (!submap) continue;
            SLAMResultRecorder::updateSubmap(id, submap->getPCDPath(), submap->getOrigin(), submap->getOriginOpti());
        }
        SLAMResultRecorder::flushBackendState();
    });
}

}  // namespace legkilo
