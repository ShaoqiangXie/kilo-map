// SPDX-License-Identifier: MIT
// @file slam_result_recorder.h
// @brief Runtime recorder for SLAM result export.
// @author Ou Guangjun
// @created 2026-04-03
// @maintainer ouguangjun98@gmail.com

#ifndef LEGKILO_SLAM_RESULT_RECORDER_H
#define LEGKILO_SLAM_RESULT_RECORDER_H

#include <fstream>
#include <iomanip>
#include <map>
#include <string>
#include <vector>

#include <glog/logging.h>
#include <boost/filesystem.hpp>

#include "common/file_io.hpp"
#include "common/math_utils.hpp"

namespace legkilo {

class SLAMResultRecorder {
   public:
    struct FrameRecord {
        uint64_t frame_id = 0;
        double timestamp = 0.0;
        int64_t submap_id = -1;
        bool is_keyframe = false;
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    };

    struct SubmapRecord {
        int64_t submap_id = -1;
        std::string pcd_path;
        Eigen::Isometry3d frontend_origin = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d backend_origin = Eigen::Isometry3d::Identity();
    };

    struct LoopEdgeRecord {
        int64_t i = -1;
        int64_t j = -1;
        Eigen::Isometry3d meas = Eigen::Isometry3d::Identity();
    };

   public:
    static void initialize(const std::string& work_dir, const std::string& config_file) {
        namespace fs = boost::filesystem;

        work_dir_ = work_dir;
        file_io::ensureDirectory(work_dir_);

        frames_.clear();
        submaps_.clear();
        loop_edges_.clear();
        next_frame_id_ = 0;

        writeEmptyFiles();
        resetTumFiles();

        if (!config_file.empty() && fs::exists(config_file)) {
            fs::copy_file(config_file, configSnapshotPath(), fs::copy_option::overwrite_if_exists);
        }

        LOG(INFO) << "TUM trajectories: " << frontendTumPath() << " , " << backendTumPath();
    }

    static void recordFrontendFrame(double timestamp, int64_t submap_id, const Eigen::Isometry3d& pose,
                                    bool is_keyframe) {
        FrameRecord frame;
        frame.frame_id = next_frame_id_++;
        frame.timestamp = timestamp;
        frame.submap_id = submap_id;
        frame.is_keyframe = is_keyframe;
        frame.pose = pose;
        frames_.push_back(frame);
        appendFrontendTum(frame);
    }

    static void updateSubmap(int64_t submap_id, const std::string& pcd_path, const Eigen::Isometry3d& frontend_origin,
                             const Eigen::Isometry3d& backend_origin) {
        auto& submap = submaps_[submap_id];
        submap.submap_id = submap_id;//// 子地图编号
        submap.pcd_path = pcd_path;//
        submap.frontend_origin = frontend_origin;// 前端计算的原始位姿
        submap.backend_origin = backend_origin;// // 后端因子图优化后的位姿
    }

    static void recordLoopEdge(int64_t i, int64_t j, const Eigen::Isometry3d& meas) {
        const std::pair<int64_t, int64_t> key = std::minmax(i, j);
        auto& edge = loop_edges_[key];
        edge.i = i;
        edge.j = j;
        edge.meas = meas;
    }

    static void flush() {
        if (work_dir_.empty()) return;
        writeFrontendFrames();
        writeSubmaps();
        writeLoopEdges();
        if (frontend_tum_ofs_.is_open()) frontend_tum_ofs_.close();
        writeFrontendTum();
        writeBackendTum();
    }

    // Keep the lightweight backend files visible to external visualization nodes while SLAM is running.
    static void flushBackendState() {
        if (work_dir_.empty()) return;
        writeSubmaps();
        writeLoopEdges();
        writeBackendTum();
    }

   private:
    static void writeEmptyFiles() {
        {
            std::ofstream ofs(frontendFramesPath(), std::ios::out | std::ios::trunc);
            ofs << "frame_id,timestamp,submap_id,is_keyframe,tx,ty,tz,qx,qy,qz,qw\n";
        }

        {
            std::ofstream ofs(submapsPath(), std::ios::out | std::ios::trunc);
            ofs << "submap_id,pcd_path,frontend_tx,frontend_ty,frontend_tz,frontend_qx,frontend_qy,frontend_qz,"
                   "frontend_qw,backend_tx,backend_ty,backend_tz,backend_qx,backend_qy,backend_qz,backend_qw\n";
        }

        {
            std::ofstream ofs(loopEdgesPath(), std::ios::out | std::ios::trunc);
            ofs << "edge_id,i,j,tx,ty,tz,qx,qy,qz,qw\n";
        }
    }

    static void writeFrontendFrames() {
        std::ofstream ofs(frontendFramesPath(), std::ios::out | std::ios::trunc);
        if (!ofs.is_open()) return;

        ofs << "frame_id,timestamp,submap_id,is_keyframe,tx,ty,tz,qx,qy,qz,qw\n";
        for (const auto& frame : frames_) {
            const Eigen::Quaterniond q(frame.pose.rotation());
            ofs << frame.frame_id << "," << std::fixed << std::setprecision(9) << frame.timestamp << ","
                << frame.submap_id << "," << (frame.is_keyframe ? 1 : 0) << "," << frame.pose.translation().x() << ","
                << frame.pose.translation().y() << "," << frame.pose.translation().z() << "," << q.x() << "," << q.y()
                << "," << q.z() << "," << q.w() << "\n";
        }
    }

    static void writeSubmaps() {
        std::ofstream ofs(submapsPath(), std::ios::out | std::ios::trunc);
        if (!ofs.is_open()) return;

        ofs << "submap_id,pcd_path,frontend_tx,frontend_ty,frontend_tz,frontend_qx,frontend_qy,frontend_qz,"
               "frontend_qw,backend_tx,backend_ty,backend_tz,backend_qx,backend_qy,backend_qz,backend_qw\n";

        for (const auto& [id, submap] : submaps_) {
            ofs << id << "," << submap.pcd_path;
            writePose(ofs, submap.frontend_origin);
            writePose(ofs, submap.backend_origin);
            ofs << "\n";
        }
    }

    static void writeLoopEdges() {
        std::ofstream ofs(loopEdgesPath(), std::ios::out | std::ios::trunc);
        if (!ofs.is_open()) return;

        ofs << "edge_id,i,j,tx,ty,tz,qx,qy,qz,qw\n";

        size_t edge_id = 0;
        for (const auto& [key, edge] : loop_edges_) {
            ofs << edge_id++ << "," << edge.i << "," << edge.j;
            writePose(ofs, edge.meas);
            ofs << "\n";
        }
    }

    // TUM: timestamp tx ty tz qx qy qz qw. Pose is IMU/body in the world frame.
    static void writeTumLine(std::ofstream& ofs, double timestamp, const Eigen::Isometry3d& pose) {
        const Eigen::Quaterniond q(pose.rotation());
        ofs << std::fixed << std::setprecision(9) << timestamp << " " << pose.translation().x() << " "
            << pose.translation().y() << " " << pose.translation().z() << " " << q.x() << " " << q.y() << " " << q.z()
            << " " << q.w() << "\n";
    }

    static void resetTumFiles() {
        if (frontend_tum_ofs_.is_open()) frontend_tum_ofs_.close();
        {
            std::ofstream ofs(frontendTumPath(), std::ios::out | std::ios::trunc);
        }
        {
            std::ofstream ofs(backendTumPath(), std::ios::out | std::ios::trunc);
        }
        frontend_tum_ofs_.open(frontendTumPath(), std::ios::out | std::ios::app);
    }

    static void appendFrontendTum(const FrameRecord& frame) {
        if (!frontend_tum_ofs_.is_open()) return;
        writeTumLine(frontend_tum_ofs_, frame.timestamp, frame.pose);
        frontend_tum_ofs_.flush();
    }

    static void writeFrontendTum() {
        std::ofstream ofs(frontendTumPath(), std::ios::out | std::ios::trunc);
        if (!ofs.is_open()) return;
        for (const auto& frame : frames_) { writeTumLine(ofs, frame.timestamp, frame.pose); }
    }

    // Backend pose uses the same submap correction as SLAMResultSaver::buildBackendTrajectory().
    static void writeBackendTum() {
        std::ofstream ofs(backendTumPath(), std::ios::out | std::ios::trunc);
        if (!ofs.is_open()) return;
        for (const auto& frame : frames_) {
            const auto it = submaps_.find(frame.submap_id);
            if (it == submaps_.end()) continue;
            const auto& submap = it->second;
            const Eigen::Isometry3d relative_pose = submap.frontend_origin.inverse() * frame.pose;
            const Eigen::Isometry3d pose_backend = submap.backend_origin * relative_pose;
            writeTumLine(ofs, frame.timestamp, pose_backend);
        }
    }

    static void writePose(std::ofstream& ofs, const Eigen::Isometry3d& pose) {
        const Eigen::Quaterniond q(pose.rotation());
        ofs << "," << std::fixed << std::setprecision(9) << pose.translation().x() << "," << pose.translation().y()
            << "," << pose.translation().z() << "," << q.x() << "," << q.y() << "," << q.z() << "," << q.w();
    }

    static std::string configSnapshotPath() { return joinPath(work_dir_, "config.yaml"); }

    static std::string frontendFramesPath() { return joinPath(work_dir_, "frontend_frames.csv"); }

    static std::string submapsPath() { return joinPath(work_dir_, "submaps.csv"); }

    static std::string loopEdgesPath() { return joinPath(work_dir_, "loop_edges.csv"); }

    static std::string frontendTumPath() { return joinPath(work_dir_, "frontend_imu.tum"); }

    static std::string backendTumPath() { return joinPath(work_dir_, "backend_imu.tum"); }

    static std::string joinPath(const std::string& dir, const std::string& name) {
        if (dir.empty()) return name;
        if (dir.back() == '/') return dir + name;
        return dir + "/" + name;
    }

    inline static std::string work_dir_;
    inline static std::ofstream frontend_tum_ofs_;
    inline static uint64_t next_frame_id_ = 0;
    inline static std::vector<FrameRecord> frames_;
    inline static std::map<int64_t, SubmapRecord> submaps_;
    inline static std::map<std::pair<int64_t, int64_t>, LoopEdgeRecord> loop_edges_;
};

}  // namespace legkilo

#endif  // LEGKILO_SLAM_RESULT_RECORDER_H
