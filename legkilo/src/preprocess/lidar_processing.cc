#include "preprocess/lidar_processing.h"

namespace legkilo {
LidarProcessing::LidarProcessing(LidarProcessing::Config config) : config_(config) {
    LOG(INFO) << "Lidar Processing is Constructed";
    // 防御式编程：filter_num_ 一定要 >= 1，否则 `i % filter_num_` 会除零/负值。
    if (config_.filter_num_ <= 0) {//降频采样
        LOG(WARNING) << "filter_num must be positive; using 1";
        config_.filter_num_ = 1;
    }
    // 预分配一个空点云缓冲（当前实现里主要作为占位符，实际每帧会 reset 新点云）
    cloud_pcl_.reset(new PointCloudType());
    // 提前算好距离阈值的平方，rangeCheck 里就不用每点做 sqrt
    min_range2_ = config_.min_range_ * config_.min_range_;
    max_range2_ = config_.max_range_ * config_.max_range_;
}

LidarProcessing::~LidarProcessing() { LOG(INFO) << "Lidar Processing is Destructed"; }

common::LidarType LidarProcessing::getLidarType() const { return config_.lidar_type_; }

void LidarProcessing::processing(const ros_compat::PointCloud2MsgConstPtr& msg, common::LidarScan& lidar_scan) {
    // 按 yaml 里配置的 LidarType 分发到具体厂商 handler
    switch (config_.lidar_type_) {
        case common::LidarType::VEL: velodyneHandler(msg, lidar_scan); break;

        case common::LidarType::OUSTER: ousterHander(msg, lidar_scan); break;

        case common::LidarType::HESAI: hesaiHandler(msg, lidar_scan); break;

        case common::LidarType::KITTI: kittiHandler(msg, lidar_scan); break;

        default: LOG(ERROR) << " Lidar Type is Not Currently Available"; break;
    }

    // The handlers append directly to points, which does not update PCL's
    // width/height fields. PCL transforms require valid organization metadata.
    // ↑ 各 handler 是 push_back 到 points 数组，不会自动同步 width/height。
    //   而 pcl::transformPointCloud 等 API 依赖这两个字段判断是否组织化点云，
    //   必须在这里补齐：width = size(), height = 1, is_dense = false。
    pcl_utils::setUnorganizedCloudMetadata(lidar_scan.cloud_, false);
}

void LidarProcessing::processing(const ros_compat::LivoxCustomMsgConstPtr& msg, common::LidarScan& lidar_scan) {
    // Livox 有自定义消息类型，独立分支处理
    livoxHandler(msg, lidar_scan);
    pcl_utils::setUnorganizedCloudMetadata(lidar_scan.cloud_, false);
}

void LidarProcessing::velodyneHandler(const ros_compat::PointCloud2MsgConstPtr& msg, common::LidarScan& lidar_scan) {
    // 每帧都新建输出点云，避免残留上一帧的数据
    lidar_scan.cloud_.reset(new PointCloudType());

    // [Velodyne] 用 pcl::fromROSMsg 反序列化到我们在头文件里注册好的 velodyne_ros::Point
    pcl::PointCloud<velodyne_ros::Point> cloud_pcl_raw;
    pcl::fromROSMsg(*msg, cloud_pcl_raw);

    // 空点云保护：仍然要给 lidar_begin/end_time_ 一个合理值（用消息头时间），
    // 否则上层同步逻辑可能拿到未初始化的时间戳。
    if (cloud_pcl_raw.empty()) {
        lidar_scan.lidar_begin_time_ = ros_compat::toSec(msg->header.stamp);
        lidar_scan.lidar_end_time_ = lidar_scan.lidar_begin_time_;
        LOG(WARNING) << "Received an empty Velodyne point cloud";
        return;
    }

    // [Velodyne] 定位本帧最早/最晚的点。Velodyne 的 time 字段是"相对本帧的偏移"，
    // 单位由驱动决定：新版驱动通常直接给"秒"，因此 time_scale_ = 1.0 常见。
    auto [min_it, max_it] =
        std::minmax_element(cloud_pcl_raw.points.begin(), cloud_pcl_raw.points.end(),
                            [](const velodyne_ros::Point& a, const velodyne_ros::Point& b) { return a.time < b.time; });

    double first_point_time = config_.time_scale_ * min_it->time;
    double last_point_time = config_.time_scale_ * max_it->time;

    // [Velodyne] 帧的绝对时间 = header.stamp + 首/末点的相对偏移
    // 前端使用这两个时间做 IMU 插值 → 点云去畸变。
    lidar_scan.lidar_begin_time_ = ros_compat::toSec(msg->header.stamp) + first_point_time;
    lidar_scan.lidar_end_time_ = ros_compat::toSec(msg->header.stamp) + last_point_time;

    int cloud_size = cloud_pcl_raw.points.size();
    lidar_scan.cloud_->points.reserve(cloud_size);

    for (int i = 0; i < cloud_size; ++i) {
        // 1) 隔点采样：只保留下标能被 filter_num_ 整除的点（等价于 1/filter_num_ 采样率）
        // 2) 距离裁剪：过近/过远的点直接扔（防自身/远处噪声）
        if ((i % config_.filter_num_) || rangeCheck(cloud_pcl_raw.points[i])) continue;
        // 3) NaN 保护：驱动偶尔会发 NaN 点
        if (nanCheck(cloud_pcl_raw.points[i])) continue;
        PointType added_point;
        added_point.x = cloud_pcl_raw.points[i].x;
        added_point.y = cloud_pcl_raw.points[i].y;
        added_point.z = cloud_pcl_raw.points[i].z;
        added_point.intensity = cloud_pcl_raw.points[i].intensity;
        // [关键] 复用 PCL 的 curvature 字段存"相对本帧起始时刻的秒偏移"，
        //        圆整到 1/500 s = 2 ms 分辨率（对 10 Hz 单帧 100 ms 足够精细）。
        //        前端去畸变时会按这个字段做 IMU 位姿插值。
        double cur_point_time = config_.time_scale_ * cloud_pcl_raw.points[i].time;
        added_point.curvature = std::round((cur_point_time - first_point_time) * 500.0) / 500.0;

        lidar_scan.cloud_->points.push_back(added_point);
    }
}

void LidarProcessing::ousterHander(const ros_compat::PointCloud2MsgConstPtr& msg, common::LidarScan& lidar_scan) {
    lidar_scan.cloud_.reset(new PointCloudType());
    // [Ouster] 反序列化到 ouster_ros::Point（含 t / reflectivity / range / ambient 等）
    pcl::PointCloud<ouster_ros::Point> cloud_pcl_raw;
    pcl::fromROSMsg(*msg, cloud_pcl_raw);

    if (cloud_pcl_raw.empty()) {
        lidar_scan.lidar_begin_time_ = ros_compat::toSec(msg->header.stamp);
        lidar_scan.lidar_end_time_ = lidar_scan.lidar_begin_time_;
        LOG(WARNING) << "Received an empty Ouster point cloud";
        return;
    }

    // [Ouster] t 字段单位 = 纳秒 (uint32)。time_scale_ 应配为 1e-9 才能换算到秒。
    auto [min_it, max_it] =
        std::minmax_element(cloud_pcl_raw.points.begin(), cloud_pcl_raw.points.end(),
                            [](const ouster_ros::Point& a, const ouster_ros::Point& b) { return a.t < b.t; });

    double first_point_time = config_.time_scale_ * min_it->t;
    double last_point_time = config_.time_scale_ * max_it->t;

    // [Ouster] 帧绝对时间 = header.stamp + 逐点偏移
    lidar_scan.lidar_begin_time_ = ros_compat::toSec(msg->header.stamp) + first_point_time;
    lidar_scan.lidar_end_time_ = ros_compat::toSec(msg->header.stamp) + last_point_time;

    int cloud_size = cloud_pcl_raw.points.size();
    lidar_scan.cloud_->points.reserve(cloud_size);

    for (int i = 0; i < cloud_size; ++i) {
        // 采样 + 距离裁剪 + NaN 保护，与 Velodyne 分支保持相同策略
        if ((i % config_.filter_num_) || rangeCheck(cloud_pcl_raw.points[i])) continue;//当 i 不能被 filter_num_ 整除、余数非零时为真。
        if (nanCheck(cloud_pcl_raw.points[i])) continue;
        PointType added_point;
        added_point.x = cloud_pcl_raw.points[i].x;
        added_point.y = cloud_pcl_raw.points[i].y;
        added_point.z = cloud_pcl_raw.points[i].z;
        // [Ouster] 这里 intensity 直接用原始 intensity 字段；如需可改成 reflectivity/range
        added_point.intensity = cloud_pcl_raw.points[i].intensity;
        double cur_point_time = config_.time_scale_ * cloud_pcl_raw.points[i].t;
        // curvature = 相对本帧起始的秒偏移（同 Velodyne 分支）
        added_point.curvature = std::round((cur_point_time - first_point_time) * 500.0) / 500.0;//将时间偏移量四舍五入到最近的 1/500 秒（2 毫秒）分辨率。


        // 舍入到最近的时间格
        // double q = std::round(x / delta) * delta;

        // 向下归入时间格
        // double q = std::floor(x / delta) * delta;

        // 向上归入时间格
        // double q = std::ceil(x / delta) * delta;

        lidar_scan.cloud_->points.push_back(added_point);
    }
}

void LidarProcessing::hesaiHandler(const ros_compat::PointCloud2MsgConstPtr& msg, common::LidarScan& lidar_scan) {
    lidar_scan.cloud_.reset(new PointCloudType());
    pcl::PointCloud<hesai_ros::Point> cloud_pcl_raw;
    pcl::fromROSMsg(*msg, cloud_pcl_raw);

    if (cloud_pcl_raw.empty()) {
        lidar_scan.lidar_begin_time_ = ros_compat::toSec(msg->header.stamp);
        lidar_scan.lidar_end_time_ = lidar_scan.lidar_begin_time_;
        LOG(WARNING) << "Received an empty Hesai point cloud";
        return;
    }

    // [Hesai] timestamp 是"逐点绝对秒"（通常 UTC），不是相对偏移
    //          → 直接取最小/最大就是本帧的绝对起始/结束时间
    auto [min_it, max_it] = std::minmax_element(
        cloud_pcl_raw.points.begin(), cloud_pcl_raw.points.end(),
        [](const hesai_ros::Point& a, const hesai_ros::Point& b) { return a.timestamp < b.timestamp; });

    double first_point_time = config_.time_scale_ * min_it->timestamp;
    double last_point_time = config_.time_scale_ * max_it->timestamp;

    // [Hesai] 注意与 Velodyne/Ouster 不同：这里不 + header.stamp！
    //          因为 timestamp 本身已经是绝对时间。若配错 time_scale_ 会导致
    //          帧时间明显跑偏，进而破坏 IMU/腿式融合的同步。
    lidar_scan.lidar_begin_time_ = first_point_time;
    lidar_scan.lidar_end_time_ = last_point_time;

    int cloud_size = cloud_pcl_raw.points.size();
    lidar_scan.cloud_->points.reserve(cloud_size);

    for (int i = 0; i < cloud_size; ++i) {
        if ((i % config_.filter_num_) || rangeCheck(cloud_pcl_raw.points[i])) continue;
        if (nanCheck(cloud_pcl_raw.points[i])) continue;
        PointType added_point;
        added_point.x = cloud_pcl_raw.points[i].x;
        added_point.y = cloud_pcl_raw.points[i].y;
        added_point.z = cloud_pcl_raw.points[i].z;
        added_point.intensity = cloud_pcl_raw.points[i].intensity;
        double cur_point_time = config_.time_scale_ * cloud_pcl_raw.points[i].timestamp;
        // [Hesai] curvature 依然存"相对本帧起始的秒偏移"（用于去畸变，风格与其它厂商一致）
        added_point.curvature = std::round((cur_point_time - first_point_time) * 500.0) / 500.0;

        lidar_scan.cloud_->points.push_back(added_point);
    }
}

void LidarProcessing::kittiHandler(const ros_compat::PointCloud2MsgConstPtr& msg, common::LidarScan& lidar_scan) {
    lidar_scan.cloud_.reset(new PointCloudType());

    pcl::PointCloud<kitti_ros::Point> cloud_pcl_raw;
    pcl::fromROSMsg(*msg, cloud_pcl_raw);

    // [KITTI] 常量说明：
    //   - kScanPeriod            : KITTI 单帧扫描周期 100 ms（当前未直接使用，保留供扩展）
    //   - kVerticalAngleOffsetRad: KISS-ICP 提出的 KITTI 数据竖直安装偏差 = 0.205°，需要修正
    //   - kAxisNormEps           : 防止旋转轴退化(点几乎与 z 轴同向)时归一化除零
    //   - kZAxis                 : 世界系 z 轴，用作旋转的固定参照
    static constexpr double kScanPeriod = 0.1;  //constexpr在编译的时候就确定
    static constexpr double kVerticalAngleOffsetRad = 0.205 * M_PI / 180.0;
    static constexpr double kAxisNormEps = 1e-12;
    static const Eigen::Vector3d kZAxis(0.0, 0.0, 1.0);
    bool if_calib = true;

    const size_t cloud_size = cloud_pcl_raw.size();

    // [KITTI] 该数据集不含逐点时间戳，整帧视为同时刻 → 起止时间都用 header.stamp
    lidar_scan.lidar_begin_time_ = ros_compat::toSec(msg->header.stamp);
    lidar_scan.lidar_end_time_ = ros_compat::toSec(msg->header.stamp);

    lidar_scan.cloud_->points.reserve(cloud_size);

    for (size_t i = 0; i < cloud_size; ++i) {
        const auto& pt = cloud_pcl_raw.points[i];
        if ((i % config_.filter_num_) || rangeCheck(pt)) { continue; }
        if (nanCheck(pt)) { continue; }
        PointType q;
        q.x = pt.x;
        q.y = pt.y;
        q.z = pt.z;

        // KISS-ICP style KITTI scan correction:
        // rotation axis = p x z_axis, angle = 0.205 deg
        // ↑ KITTI 官方雷达在安装时相对水平面有约 0.205° 的竖直角偏差。
        //   KISS-ICP 的做法是：对每个点 p，绕 axis = (p × z_axis)/||·||，
        //   旋转 0.205° 得到校正后的点 p'。这等价于把点云"抬平"，
        //   显著减小回环处的 z 向漂移。参考 KISS-ICP kitti_utils。
        if (if_calib) {
            const Eigen::Vector3d p(static_cast<double>(q.x), static_cast<double>(q.y), static_cast<double>(q.z));
            const Eigen::Vector3d rotation_vec = p.cross(kZAxis);//叉乘计算得到旋转轴
            const double axis_norm = rotation_vec.norm();//计算向量模长 模长表示下旋转角度

            // Guard against degenerate normalization when point is near z-axis.
            // ↑ 当点近乎与 z 轴共线时，叉乘范数接近 0，归一化会不稳；直接跳过校正。
            if (axis_norm > kAxisNormEps) {//当旋转轴的模长大于一个很小的阈值时，才进行归一化和旋转操作
                const Eigen::Vector3d axis = rotation_vec / axis_norm;//归一化得到旋转轴
                const Eigen::AngleAxisd aa(kVerticalAngleOffsetRad, axis);//构造旋转矩阵，旋转角度为 0.205°，旋转轴为 axis
                const Eigen::Vector3d p_corr = aa * p;
                q.x = static_cast<float>(p_corr.x());
                q.y = static_cast<float>(p_corr.y());
                q.z = static_cast<float>(p_corr.z());
            }
        }

        q.intensity = pt.intensity;
        // [KITTI] 无逐点时间戳 → curvature 恒为 0，前端"去畸变"步骤实际不会调整。
        q.curvature = 0.0f;

        lidar_scan.cloud_->points.push_back(q);
    }

    // KITTI 分支直接自己维护 width/height（其他分支靠 setUnorganizedCloudMetadata 兜底）
    lidar_scan.cloud_->width = static_cast<uint32_t>(lidar_scan.cloud_->points.size());
    lidar_scan.cloud_->height = 1;
    lidar_scan.cloud_->is_dense = false;
}

void LidarProcessing::livoxHandler(const ros_compat::LivoxCustomMsgConstPtr& msg, common::LidarScan& lidar_scan) {
    lidar_scan.cloud_.reset(new PointCloudType());

    // [Livox] Livox 使用自定义 CustomMsg 而非 PointCloud2；points 是 CustomPoint 序列
    const auto& points = msg->points;
    // [Livox] header.stamp 是本帧的绝对基准时间（秒）
    const double timebase = ros_compat::toSec(msg->header.stamp);//这一帧的绝对时间

    if (points.empty()) {
        lidar_scan.lidar_begin_time_ = timebase;
        lidar_scan.lidar_end_time_ = timebase;
        LOG(WARNING) << "Received an empty Livox point cloud";
        return;
    }

    // [Livox] offset_time 单位 = 纳秒 (uint32/uint64)，需要靠 time_scale_ = 1e-9 换算到秒
    //这里是查找偏移量
    auto [min_it, max_it] = std::minmax_element(
        points.begin(), points.end(), [](const ros_compat::LivoxCustomPoint& a, const ros_compat::LivoxCustomPoint& b) {
            return a.offset_time < b.offset_time;
        });//这里返回一个 pair，min_it 是指向最小 offset_time 的迭代器，max_it 是指向最大 offset_time 的迭代器。

    double first_point_time = config_.time_scale_ * min_it->offset_time;
    double last_point_time = config_.time_scale_ * max_it->offset_time;

    // [Livox] 与 Velodyne/Ouster 一样：帧绝对时间 = header.stamp + 首/末点偏移
    lidar_scan.lidar_begin_time_ = timebase + first_point_time;
    lidar_scan.lidar_end_time_ = timebase + last_point_time;

    const size_t cloud_size = points.size();
    lidar_scan.cloud_->points.reserve(cloud_size);

    // 注意：这里从 i = 1 开始，跳过第 0 个点。这是 Livox 常见的规避——
    // 有些驱动版本第 0 个点的 offset_time 可能为 0 且不可靠。
    for (size_t i = 1; i < cloud_size; ++i) {
        const auto& pt = points[i];
        // [Livox] tag 字节的低 4~5 位描述回波质量（多回波 / 噪点标志）：
        //   位掩码 0x30 对应"回波编号"字段：
        //     0x00 = 单次回波（首回）
        //     0x10 = 强回波
        //   其余组合（0x20, 0x30）表示置信度低/次要回波，此处不采纳。
        // 参考: Livox 官方 CustomPoint.tag 说明。
        if (!((pt.tag & 0x30) == 0x10) && !((pt.tag & 0x30) == 0x00)) continue;//按位与运算，提取16进制的低4位和5位，判断是否为强回波或单次回波
        if ((i % config_.filter_num_) || rangeCheck(pt)) continue;
        if (nanCheck(pt)) continue;

        PointType added_point;
        added_point.x = pt.x;
        added_point.y = pt.y;
        added_point.z = pt.z;
        // [Livox] 只有 reflectivity 字段，语义类似 intensity；显式转 float
        added_point.intensity = static_cast<float>(pt.reflectivity);
        double cur_point_time = config_.time_scale_ * pt.offset_time;
        // curvature 记录相对本帧起始的秒偏移（同其它厂商）
        added_point.curvature = std::round((cur_point_time - first_point_time) * 500.0) / 500.0;

        lidar_scan.cloud_->points.push_back(added_point);
    }

    // 与 KITTI 分支一样，自己直接写 width/height；父函数的 setUnorganizedCloudMetadata 会再兜底一次
    lidar_scan.cloud_->width = static_cast<uint32_t>(lidar_scan.cloud_->points.size());
    lidar_scan.cloud_->height = 1;
    lidar_scan.cloud_->is_dense = false;
}

}  // namespace legkilo
