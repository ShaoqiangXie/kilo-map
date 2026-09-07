#include "core/slam/backend/submap.h"

#include <exception>

#include <glog/logging.h>
#include <pcl/io/pcd_io.h>

#include "common/voxel_grid.hpp"

namespace legkilo {

// ---------------------------------------------------------------
// addFrame：把一帧关键帧点云"折叠"到子地图坐标系
//   [1] 已 finished / 空帧 → 直接丢弃
//   [2] 首帧：把 pose 作为子地图原点（origin_ 与 origin_opti_ 初值一致）
//   [3] 计算 T_sub_body = origin_^-1 * pose，把点云投影到子地图坐标系并累加
//   [4] 记录本帧的 (world 位姿, 子地图相对位姿)
// ---------------------------------------------------------------
void Submap::addFrame(const CloudPtr& frame, const Eigen::Isometry3d& pose) {
    if (finished_) return;                       // [1] 子地图已收尾，禁止再改
    if (!frame || frame->empty()) return;

    if (this->getNumFrames() == 0) {
        // [2] 第一帧的世界位姿即为子地图的"原点"，两者初始时同步
        origin_ = pose;
        origin_opti_ = pose;
    }

    // [3] 把机体系点云投影到子地图坐标系（origin_ 系）：p_sub = T_sub_body * p_body
    Eigen::Isometry3d relative_pose = origin_.inverse() * pose;
    PointCloudType transformed_frame;
    pcl::transformPointCloud(*frame, transformed_frame, relative_pose.matrix().cast<float>());  // pcl 只接受 float
    *cloud_sum_ += transformed_frame;                                                            // 直接叠加到累积点云

    // [4] 分别保存"子地图内相对位姿"与"前端世界位姿"，两者互相独立、均按插入顺序排列
    related_frame_poses_.emplace_back(relative_pose);
    frame_poses_.emplace_back(pose);
}

// ---------------------------------------------------------------
// setFinished：收尾子地图 —— 体素降采样 + 写盘 PCD
//   降采样目的：a) 内存和存储开销显著下降；
//               b) 后续回环 verify 时 GICP/KISS 匹配更稳、速度更快
// ---------------------------------------------------------------
void Submap::setFinished(bool downsample, double resolution) {
    if (finished_) return;   // 幂等：只有第一次调用生效
    finished_ = true;

    if (downsample && cloud_sum_) {
        // 采用"中值代表点"采样，比简单质心更抗离群点
        VoxelGrid voxel_filter(resolution, SamplingMode::MedianRepresentative);
        CloudPtr cloud_filtered = pcl_utils::makeCloud<PointType>();//创建一个空的点云对象，用于存储滤波后的点云数据。
        voxel_filter.filter(cloud_sum_, cloud_filtered);//对 cloud_sum_ 进行体素滤波，并将结果存储在 cloud_filtered 中。
        cloud_sum_ = cloud_filtered;//将滤波后的点云赋值给 cloud_sum_，以便后续使用。
    }

    // 写盘：文件命名 submap_<id>.pcd；供 Recorder 记录 & 回环 verify 时按需 reload
    this->savePCD();
}

// 释放内存中的点云（不删磁盘 PCD）；下一次 getCloud() 会触发 loadPCD
void Submap::releaseCloud() { cloud_sum_.reset(); }

// ---------------------------------------------------------------
// savePCD：以二进制压缩格式写盘。异常会被吞掉并记录 warning，不抛给上层
// ---------------------------------------------------------------
bool Submap::savePCD() {
    if (!cloud_sum_ || cloud_sum_->empty() || save_path_folder.empty()) return false;
    const std::string file_path = save_path_folder + "/submap_" + std::to_string(id_) + ".pcd";
    try {
        // savePCDFileBinaryCompressed 成功时返回 0
        return pcl::io::savePCDFileBinaryCompressed(file_path, *cloud_sum_) == 0;
    } catch (const std::exception& e) {
        LOG(WARNING) << "Failed to save submap PCD: " << file_path << ", error: " << e.what();
        return false;
    }
}

// ---------------------------------------------------------------
// loadPCD：懒加载 —— 只有 cloud_sum_ 已被释放时才真正读磁盘
// ---------------------------------------------------------------
CloudPtr Submap::loadPCD() {
    if (save_path_folder.empty()) return nullptr;
    if (!cloud_sum_) {
        const std::string file_path = save_path_folder + "/submap_" + std::to_string(id_) + ".pcd";
        cloud_sum_ = pcl_utils::makeCloud<PointType>();
        // 加载失败时把 cloud_sum_ 复位为 nullptr，避免上层拿到"看似有效实则为空"的指针
        if (pcl::io::loadPCDFile(file_path, *cloud_sum_) != 0) cloud_sum_.reset();
    }
    return cloud_sum_;
}

// 全局自增 ID：每个 Submap 分配一个进程内唯一的 int64_t（不复用旧号）
Submap::NodeType Submap::generateGlobalId() {
    static NodeType global_id = 0;
    return global_id++;
}

// ---------------------------------------------------------------
// getBeginEndFrameDistance：首帧和尾帧在子地图坐标系下的欧氏距离
//   物理意义：子地图跨越的空间尺度；被 Backend::isSubmapFinished 用作切分依据。
// ---------------------------------------------------------------
double Submap::getBeginEndFrameDistance() const {
    if (related_frame_poses_.size() < 2) return 0.0;   // 帧数不足 2 时无意义
    const Eigen::Vector3d begin_pos = related_frame_poses_.front().translation();
    const Eigen::Vector3d end_pos = related_frame_poses_.back().translation();
    return (end_pos - begin_pos).norm();
}

}  // namespace legkilo
