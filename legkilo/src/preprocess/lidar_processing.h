// SPDX-License-Identifier: MIT
// @file lidar_processing.h
// @brief LiDAR preprocessing: filtering, conversion and noise.
// @author Ou Guangjun
// @created 2024-12-17
// @maintainer ouguangjun98@gmail.com
//
// ============================================================
// 【模块概览】LiDAR 预处理 (LidarProcessing)
// 作用：把各型号 LiDAR 的 ROS 消息统一转成 SLAM 前端需要的
//       `common::LidarScan`：
//         * 一帧点云 (CloudPtr)，字段 = (x, y, z, intensity, curvature)
//           这里的 `curvature` 被复用来存"相对本帧起始时刻的偏移时间 (s)"，
//           前端去畸变 (undistortion) 就靠它插值 IMU 位姿。
//         * 本帧起始/结束时间 (double 秒)
// 支持型号：
//   - Velodyne (VLP-16/HDL-32/…)  ─ 字段 time  (float, 秒/或与 time_scale 联合决定单位)
//   - Ouster   (OS0/OS1/OS2)      ─ 字段 t     (uint32, 通常纳秒)
//   - Hesai    (Pandar 系列)      ─ 字段 timestamp (double, 秒 UTC)
//   - KITTI    (回放数据集)        ─ 无逐点时间戳，只有 ring；这里额外做
//                                    KISS-ICP 风格的 0.205° 竖直校正
//   - Livox    (Mid-360 等)       ─ 自定义 CustomMsg，字段 offset_time (ns) + tag
// 关键设计：
//   - `time_scale_` 由 yaml 配置，用于把不同厂商的时间字段统一到"秒"。
//     例：Velodyne time 已是秒 → 1.0；Ouster t 是纳秒 → 1e-9；Livox 是纳秒 → 1e-9。
//   - `filter_num_` 是隔点采样倍率，缓解点云过密（每 filter_num_ 取 1 点）。
//   - `min_range_ / max_range_` 直接丢弃太近/太远的点（例如自身机身/远处噪声）。
// 数据流：
//   PointCloud2 / LivoxCustomMsg ──> processing() ──> LidarScan
// ============================================================
#ifndef LEG_KILO_LIDAR_PROCESSING_H
#define LEG_KILO_LIDAR_PROCESSING_H

#include "common/pcl_types.h"
#include "common/sensor_types.hpp"
#include "interface/common/ros_compat.h"

#include <glog/logging.h>
#include <pcl_conversions/pcl_conversions.h>

// ------------------------------------------------------------
// [Velodyne] 逐点结构：与 velodyne_pointcloud 驱动发布的 PointCloud2 字段一一对应
//   - x/y/z      : 反射点坐标 (m)
//   - intensity  : 反射强度
//   - time       : 相对本帧起始的时间偏移；单位由驱动/配置决定，需配合
//                  LidarProcessing::Config::time_scale_ 一起换算为秒。
//   - ring       : 线号（本项目当前不使用，故注释掉；如需分线处理可打开）。
// EIGEN_ALIGN16 + PCL_ADD_POINT4D 保证 SSE/AVX 对齐，避免 PCL 内部处理时的 UB。
// ------------------------------------------------------------
namespace velodyne_ros {
struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    float time;
    // std::uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace velodyne_ros
// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time, time)
    // (std::uint16_t, ring, ring)
)
// clang-format on
// ------------------------------------------------------------
// [Ouster] 逐点结构（与官方 ouster_ros 驱动一致）
//   - t            : 相对本帧起始的时间偏移，单位 = 纳秒 (uint32)
//                    ⇒ time_scale_ 通常配置为 1e-9
//   - reflectivity : 反射率（0~65535）
//   - ambient      : 环境光强度（供曝光校正用，本项目未用）
//   - range        : 距离 (uint32, 通常单位是 mm)
//   - ring         : 线号，本项目未使用，已注释
// ------------------------------------------------------------
namespace ouster_ros {
struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    std::uint16_t reflectivity;
    // std::uint8_t ring;
    std::uint16_t ambient;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace ouster_ros
// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    // (std::uint8_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)
// clang-format on
// ------------------------------------------------------------
// [Hesai] 逐点结构：Pandar 系列 (PandarXT/QT/AT) 驱动格式
//   - timestamp : 逐点绝对时间戳，单位 = 秒 (double, 通常与 UTC 对齐)
//                 ⇒ 与其他厂商不同，这里的 lidar_begin/end_time_ 直接使用
//                   逐点 timestamp 的 min/max，不再加上 header.stamp！
//   - ring      : 线号，本项目未使用
// ------------------------------------------------------------
namespace hesai_ros {
struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    double timestamp;
    // std::uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace hesai_ros
// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(hesai_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (double, timestamp, timestamp)
    // (std::uint16_t, ring, ring)
)
// clang-format on
// ------------------------------------------------------------
// [KITTI] 逐点结构：KITTI 数据集回放（无逐点时间戳，只有 ring）
//   - ring : 竖直线号 (0~63)，用于组织点云
//   - 注意 KITTI 帧的时间戳 = header.stamp，本帧内部所有点视为同时刻
//     ⇒ lidar_begin_time_ == lidar_end_time_ == header.stamp
//   - KITTI 还有已知的 0.205° 竖直安装误差，见 .cc 中的 KISS-ICP 校正实现
// ------------------------------------------------------------
namespace kitti_ros {
struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    std::uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace kitti_ros
// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(kitti_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint16_t, ring, ring)
)
// clang-format on

namespace legkilo {

// ============================================================
// 【类】LidarProcessing
// 作用：多型号 LiDAR 的统一预处理入口，把 ROS 原始消息 → 统一的 LidarScan。
// 关键成员：
//   - config_           : 型号 / 距离阈值 / 时间尺度 / 采样倍率
//   - min_range2_, max_range2_ : min/max_range 的平方（避免每点开根）
// 使用方式：
//   1. 构造：`LidarProcessing lp(cfg);`
//   2. 每帧回调里调用 `lp.processing(msg, scan)`，把 scan 塞进 lidar_cache_
//   3. `processing()` 内部按 config_.lidar_type_ 分发到对应厂商 handler
// 注意事项：
//   - handler 直接 push_back 到 scan.cloud_->points，会漏更新 width/height；
//     所以每次调用完 handler，父函数会补一句 setUnorganizedCloudMetadata()，
//     否则后续 pcl::transformPointCloud 之类可能失败。
//   - 采样：`i % filter_num_` 保证保留下标 0, filter_num_, 2*filter_num_, ...
// ============================================================
class LidarProcessing {
   public:
    // LiDAR 预处理配置（由 yaml 读入，见 interface/common/options.h）
    struct Config {
        float min_range_ = 1.0f;        // 最小有效距离 (m)，小于此值一律丢弃（防自身遮挡）
        float max_range_ = 100.0f;      // 最大有效距离 (m)，大于此值一律丢弃（防远处噪声）
        int filter_num_ = 1;            // 隔点采样倍率：每 filter_num_ 个点保留 1 个；1 = 不降采样
        common::LidarType lidar_type_ = common::LidarType::VEL;  // 型号选择：VEL/OUSTER/HESAI/KITTI/LIVOX
        double time_scale_ = 1.0;       // 时间字段单位换算到"秒"的比例。
                                        // Velodyne(秒): 1.0；Ouster/Livox(纳秒): 1e-9；Hesai(秒): 1.0
    };

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    LidarProcessing() = delete;
    /**
     * @brief 构造：拷入 config，预分配一个空的 PCL 点云，计算 min/max_range^2。
     */
    LidarProcessing(LidarProcessing::Config config);
    ~LidarProcessing();

    /** @brief 获取当前配置的 LiDAR 型号，供上层选择不同回调订阅路径。 */
    common::LidarType getLidarType() const;

    /**
     * @brief 处理常规 PointCloud2 消息（Velodyne / Ouster / Hesai / KITTI）。
     * @param msg        ROS PointCloud2 智能指针
     * @param lidar_scan [out] 填入去噪/裁剪/时间标记后的统一点云
     *
     * 内部根据 lidar_type_ 分发到具体 handler，并在最后补齐点云 width/height。
     */
    void processing(const ros_compat::PointCloud2MsgConstPtr& msg, common::LidarScan& lidar_scan);

    /**
     * @brief 处理 Livox 自定义消息（Mid-40/Mid-70/Mid-360 等）。
     *        Livox 不走 PointCloud2 而是走 livox_ros_driver 的 CustomMsg，需要单独分支。
     */
    void processing(const ros_compat::LivoxCustomMsgConstPtr& msg, common::LidarScan& lidar_scan);

    /**
     * @brief 距离裁剪：以点到原点(激光雷达坐标系)的欧氏距离过滤掉太近或太远的点。
     * @return true = 应当被丢弃；false = 保留。
     * 用平方距离比较可避免开根号，性能更好。
     */
    template <typename T>
    inline bool rangeCheck(const T& p) {
        const float d2 = p.x * p.x + p.y * p.y + p.z * p.z;
        return (d2 < min_range2_) || (d2 > max_range2_);
    }

    /**
     * @brief NaN/Inf 检查：驱动偶尔会给出 NaN 点（无回波/去畸变失败），必须丢弃。
     * @return true = 该点包含非有限值，应当丢弃。
     */
    template <typename PointT>
    inline bool nanCheck(const PointT& p) {
        return !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z);
    }

   private:
    // 分厂商实现（成员函数指针不使用，代码结构上用 switch 简单分发）
    void velodyneHandler(const ros_compat::PointCloud2MsgConstPtr& msg, common::LidarScan& lidar_scan);
    void ousterHander(const ros_compat::PointCloud2MsgConstPtr& msg, common::LidarScan& lidar_scan);
    void hesaiHandler(const ros_compat::PointCloud2MsgConstPtr& msg, common::LidarScan& lidar_scan);
    void kittiHandler(const ros_compat::PointCloud2MsgConstPtr& msg, common::LidarScan& lidar_scan);
    void livoxHandler(const ros_compat::LivoxCustomMsgConstPtr& msg, common::LidarScan& lidar_scan);

    CloudPtr cloud_pcl_;             // 预分配的临时点云缓冲（当前实现里未真正复用）

    float min_range2_ = 1.0f;        // min_range_ 的平方，加速 rangeCheck
    float max_range2_ = 10000.0f;    // max_range_ 的平方，加速 rangeCheck
    Config config_;                  // 完整配置
};

}  // namespace legkilo

#endif  // LEG_KILO_LIDAR_PROCESSING_H
