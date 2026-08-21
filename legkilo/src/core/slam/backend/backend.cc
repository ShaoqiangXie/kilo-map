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

Backend::Backend(const std::string& yaml_file) {
    stopping_.store(false, std::memory_order_release);
    is_first_frame_ = true;

    YamlHelper yaml_helper(yaml_file);
    config_.kf_trans_threshold = yaml_helper.get<double>("kf_trans_threshold", 0.3);
    config_.kf_degree_threshold = yaml_helper.get<double>("kf_degree_threshold", 5.0);
    config_.kf_max_num_submap = yaml_helper.get<size_t>("kf_max_num_submap", 50);
    config_.kf_max_dist_submap = yaml_helper.get<double>("kf_max_dist_submap", 5.0);
    config_.submap_idle_finish_timeout = std::max(yaml_helper.get<double>("submap_idle_finish_timeout", 1.0), 0.1);

    std::string temp_result_save_folder =
        std::string(ROOT_DIR) + "result/" + yaml_helper.get<std::string>("temp_result_save_folder", "temp") + "/";
    file_io::ensureDirectory(temp_result_save_folder);
    file_io::clearDirectoryContents(temp_result_save_folder);
    Submap::setSavePathFolder(temp_result_save_folder);
    SLAMResultRecorder::initialize(temp_result_save_folder, yaml_file);

    factor_graph_ = std::make_unique<FactorGraph>(yaml_file);

    bool loop_closure_enabled = yaml_helper.get<bool>("loop_closure_enable", true);
    if (loop_closure_enabled) loop_closure_ = std::make_unique<LoopClosure>(yaml_file);//创建 LoopClosure 对象，loop_closure_为非空指针
}

Backend::~Backend() {
    this->stop();
    LOG(INFO) << "Backend is destroyed";
}

    // 将当前帧加入后端：
    // 1. cloud_body：机体坐标系下的点云
    // 2. 当前 IMU 位姿：旋转和平移
    // 3. 当前帧时间戳
    // 4. 前端匹配类型或匹配结果

void Backend::addFrame(const CloudPtr& frame, const Eigen::Isometry3d& pose, double timestamp,
                       LidarMatchTypesPtr match_types) {
    if (!frame || frame->empty()) return;

    std::lock_guard<std::mutex> lock(mutex_queue_);
    frame_queue_.push_back({frame, pose, timestamp, match_types});
    cv_.notify_one();
}

void Backend::start() {
    if (loop_closure_) loop_closure_->start();
    worker_thread_ = std::thread(&Backend::workerLoop, this);
}

void Backend::stop() {
    stopping_.store(true, std::memory_order_release);
    cv_.notify_one();
    if (worker_thread_.joinable()) worker_thread_.join();
    if (loop_closure_) loop_closure_->stop();
}

void Backend::setViewerInterface(ViewerSlamInterface* viewer_interface) { viewer_interface_ = viewer_interface; }

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

bool Backend::isKeyFrame(const Eigen::Isometry3d& current_pose) {
    if (is_first_frame_) {
        is_first_frame_ = false;
        last_kf_pose_ = current_pose;
        return true;
    }//如果是第一帧，就设置为关键帧
    Eigen::Isometry3d relative_pose = last_kf_pose_.inverse() * current_pose;//计算当前帧相对于上一个关键帧的位姿变化
    double trans_diff = relative_pose.translation().norm();//计算平移变化的欧几里得距离    translation()--平移   norm()--欧几里得范数
    double rot_diff = Eigen::AngleAxisd(relative_pose.rotation()).angle() * (180.0 / M_PI);  // in degrees     //计算旋转变化的角度差，转换为角度单位

    if (trans_diff > config_.kf_trans_threshold || rot_diff > config_.kf_degree_threshold) {   //超过0.3米或5度就认为是关键帧
        last_kf_pose_ = current_pose;
        return true;
    }

    return false;
}

bool Backend::isSubmapFinished(const SubmapPtr& submap) const {
    bool is_finished = false;

    is_finished |= submap->getNumFrames() >= config_.kf_max_num_submap;

    is_finished |= submap->getBeginEndFrameDistance() >= config_.kf_max_dist_submap;

    return is_finished;
}

void Backend::finalizeCurrentSubmap() {
    if (!current_submap_) return;
    if (current_submap_->getNumFrames() == 0) {
        LOG(WARNING) << "Skip empty submap " << current_submap_->getId() << " during finalize";
        current_submap_.reset();
        return;
    }

    current_submap_->setFinished(true, 0.1);//TODO

    {
        std::unique_lock<std::mutex> lock(mutex_finished_submaps_);
        finished_submaps_.insert({current_submap_->getId(), current_submap_});
    }

    if (viewer_interface_) {
        static constexpr float kViewerSubmapVoxelResolution = 0.1f;
        VoxelGrid viewer_voxel_filter(kViewerSubmapVoxelResolution, SamplingMode::MedianRepresentative);
        CloudPtr viewer_cloud = pcl_utils::makeCloud<PointType>();
        viewer_voxel_filter.filter(current_submap_->getCloud(), viewer_cloud);

        viewer_interface_->insertFinishedSubmap(current_submap_->getId(),
                                                {current_submap_->getOriginOpti(), current_submap_->getFramePoses(),
                                                 pcl_utils::PclCloudToVecCloud(viewer_cloud)});
    }

    factor_graph_->addNode(current_submap_->getId(), current_submap_->getOriginOpti());

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
    //将所有已经完成的子地图信息同步到 SLAM 结果记录器，并把最新的后端状态写入文件。
    this->updateRecordedSubmaps();
    //刷新可视化模块中已完成子地图的位姿信息，以便在可视化界面中显示优化后的位姿。
    if (viewer_interface_) { viewer_interface_->updateFinishedSubmapPose(optimized_poses); }

    if (loop_closure_) loop_closure_->insert(optimized_poses);

    current_submap_->releaseCloud();  // release cloud to save memory
    current_submap_.reset();
}

void Backend::fetchAndApplyLoopClosures() {
    auto loops = loop_closure_->fetchVerified();//获取已验证的回环
    if (loops.empty()) return;

    for (const auto& lc : loops) {
        factor_graph_->addLoopClosureEdge(lc.i, lc.j, lc.meas);//将回环约束加入系统
        SLAMResultRecorder::recordLoopEdge(lc.i, lc.j, lc.meas);//将回环约束记录到 SLAM 结果记录器中
        if (viewer_interface_) viewer_interface_->insertEdge(lc.i, lc.j);//将回环约束可视化
    }

    Timer::measure("FactorGraph Optimize: ", [&]() { factor_graph_->optimize(); });

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

    this->updateRecordedSubmaps();

    if (viewer_interface_) { viewer_interface_->updateFinishedSubmapPose(optimized_poses); }
}
//将所有已经完成的子地图信息同步到 SLAM 结果记录器，并把最新的后端状态写入文件。
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
