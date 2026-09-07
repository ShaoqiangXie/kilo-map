// SPDX-License-Identifier: MIT
// @file backend_submap_publisher_ros2.cc
// @brief Publish optimized backend submaps as one colorized ROS2 point cloud.
//
// ============================================================
// 【模块概览】backend_submap_publisher（后端 Submap 发布工具，ROS2 独立 node）
// 作用：把 Backend 训练/优化后落盘的所有 Submap（PCD + 全局位姿）合并染色，
//       周期性地以一整包 PointCloud2 的形式发布出去，供：
//         * RViz 直观查看全局地图
//         * 下游定位模块（例如复用该地图做重定位）
//         * 论文/汇报截图
//
// 工作原理：
//   1) 参数 `result_dir` 指向 Backend 的输出目录，默认 ${ROOT_DIR}/result/temp。
//   2) 目录里维护一个 `submaps.csv`：每行 = (id, pcd_path, ...9 列..., tx, ty, tz, qx, qy, qz, qw)。
//      其中位姿的四元数按 (x, y, z, w) 顺序存储；构造 Eigen::Quaterniond 时
//      构造顺序是 (w, x, y, z)，见 parseSubmapRecord 的调用顺序。
//   3) 定时器 `publish_period_sec` 秒触发一次 publishIfUpdated()：
//        - 用 last_write_time_ 判断 csv 是否有更新，无更新则跳过；
//        - 读入所有 SubmapRecord → 依次加载对应 PCD；
//        - 对 PCD 的每个点乘以 optimized_pose 变换到全局系；
//        - 用金分比色相 (Golden Ratio Hue) 给每个 submap id 分配不同颜色；
//        - 合并成 pcl::PointXYZRGB 点云 → 转 sensor_msgs::PointCloud2 → publish。
//   4) QoS：KeepLast(1) + reliable + transient_local。
//      transient_local 让"后加入的订阅者"也能立即收到最近一帧点云
//      （类似 latched publisher）。适合"地图这种缓变数据"。
//
// 与主 SLAM 节点的关系：完全解耦，只通过文件系统 (`submaps.csv` + PCD 文件) 共享。
// ============================================================

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "common/pcl_types.h"

namespace legkilo {
namespace {

namespace fs = std::filesystem;

// ---------- CSV 行 → SubmapRecord ----------
// SubmapRecord：内存中一条 submap 的最小信息
//   - id             : Backend 分配的 submap 数字 ID，唯一且用于染色
//   - pcd_path       : submap 局部坐标系点云 PCD 的绝对路径
//   - optimized_pose : Backend 因子图优化后的 T_world_submap（把点云搬到全局系）
struct SubmapRecord {
    int64_t id = -1;
    fs::path pcd_path;
    Eigen::Isometry3d optimized_pose = Eigen::Isometry3d::Identity();
};

/**
 * @brief 按逗号切分一行 CSV，返回列字符串数组。
 * @note  简易实现，不处理带引号的字段；submaps.csv 里没有转义需求。
 */
std::vector<std::string> splitCsvLine(const std::string& line) {
    std::vector<std::string> columns;
    std::stringstream stream(line);
    std::string column;
    while (std::getline(stream, column, ',')) columns.push_back(column);
    return columns;
}

/**
 * @brief 把一行 submaps.csv 解析成 SubmapRecord。
 * @param line       CSV 中的一整行
 * @param result_dir 结果根目录；若 pcd_path 是相对路径，会与它拼接
 * @param record     [out] 解析成功后的 SubmapRecord
 * @return 成功 true / 失败 false（列数不足、四元数退化、std::stod/stoll 抛异常等）
 *
 * 【CSV 列约定】（与 Backend 落盘保持一致，共 ≥16 列）
 *   [0]           id
 *   [1]           pcd 相对/绝对路径
 *   [2..8]        Backend 其它元数据（帧数、时间戳等，本工具不关心）
 *   [9..11]       tx, ty, tz              （T_world_submap 的平移）
 *   [12..15]      qx, qy, qz, qw          （T_world_submap 的四元数，xyzw 顺序！）
 * 【注意】Eigen::Quaterniond 的构造参数顺序是 (w, x, y, z)，
 *        所以此处显式用 (columns[15], columns[12], columns[13], columns[14])。
 */
bool parseSubmapRecord(const std::string& line, const fs::path& result_dir, SubmapRecord& record) {
    const auto columns = splitCsvLine(line);
    if (columns.size() < 16) return false;   // 列数不齐：可能是空行/损坏行

    try {
        record.id = std::stoll(columns[0]);
        record.pcd_path = fs::path(columns[1]);
        // 允许 csv 里存相对路径（相对 result_dir），便于结果目录迁移
        if (record.pcd_path.is_relative()) record.pcd_path = result_dir / record.pcd_path;

        const Eigen::Vector3d translation(std::stod(columns[9]), std::stod(columns[10]), std::stod(columns[11]));
        // 注意四元数顺序：Eigen 是 (w, x, y, z)，csv 里存的是 (x, y, z, w)
        Eigen::Quaterniond rotation(std::stod(columns[15]), std::stod(columns[12]), std::stod(columns[13]),
                                    std::stod(columns[14]));
        if (rotation.norm() < 1e-12) return false;  // 全零四元数 → 视为损坏
        rotation.normalize();

        record.optimized_pose = Eigen::Isometry3d::Identity();
        record.optimized_pose.linear() = rotation.toRotationMatrix();
        record.optimized_pose.translation() = translation;
        return true;
    } catch (const std::exception&) {
        // stod / stoll 解析失败：视为损坏行，跳过而不 crash
        return false;
    }
}

/**
 * @brief 为一个 submap ID 生成"人眼易区分"的 RGB 颜色。
 * @param id  Submap ID（非负）；负值会被截到 0
 * @return    (r, g, b) 三元组，各分量范围 0~255
 *
 * 【算法要点】
 *  - 使用 φ⁻¹ (0.6180339887…) 的黄金分割共轭作为 hue 步长：
 *      hue = (id * 0.618…) mod 1
 *    这样任意相邻 id 之间 hue 差 ≈ 0.618，色差最大化，
 *    保证 100+ 个 submap 相邻编号颜色也明显不同。
 *  - 固定饱和度 0.78、明度 1.0，再走标准 HSV→RGB 公式（6 段扇区）。
 *  - 输出四舍五入到 uint8，便于直接塞进 pcl::PointXYZRGB。
 */
std::array<uint8_t, 3> colorForSubmap(const int64_t id) {
    // Golden-ratio hue spacing keeps adjacent submap IDs visually distinct.
    constexpr double kGoldenRatioConjugate = 0.6180339887498949;
    const double hue = std::fmod(static_cast<double>(std::max<int64_t>(id, 0)) * kGoldenRatioConjugate, 1.0);
    constexpr double saturation = 0.78;
    constexpr double value = 1.0;

    // 标准 HSV → RGB（H∈[0,1] 映射到 6 个扇区）
    const double h6 = hue * 6.0;
    const int sector = static_cast<int>(std::floor(h6));
    const double fraction = h6 - static_cast<double>(sector);
    const double p = value * (1.0 - saturation);
    const double q = value * (1.0 - saturation * fraction);
    const double t = value * (1.0 - saturation * (1.0 - fraction));

    double r = 0.0;
    double g = 0.0;
    double b = 0.0;
    switch (sector % 6) {
        case 0: r = value; g = t; b = p; break;   // 红→黄
        case 1: r = q; g = value; b = p; break;   // 黄→绿
        case 2: r = p; g = value; b = t; break;   // 绿→青
        case 3: r = p; g = q; b = value; break;   // 青→蓝
        case 4: r = t; g = p; b = value; break;   // 蓝→紫
        default: r = value; g = p; b = q; break;  // 紫→红
    }

    return {static_cast<uint8_t>(std::lround(r * 255.0)), static_cast<uint8_t>(std::lround(g * 255.0)),
            static_cast<uint8_t>(std::lround(b * 255.0))};
}

}  // namespace

// ============================================================
// 【类】BackendSubmapPublisher
// 作用：ROS2 单节点，定时扫描 Backend 输出目录 → 合并染色 → 发布 PointCloud2。
// 关键成员：
//   - result_dir_          : Backend 输出根目录（含 submaps.csv 与各 submap PCD）
//   - frame_id_            : 发布点云的 frame_id（默认 camera_init）
//   - output_topic_        : 输出话题名（默认 backend/submaps_colored）
//   - publisher_ / timer_  : ROS2 发布器 + 挂钟定时器
//   - last_write_time_     : 上次成功读取 csv 时的文件修改时间，用于避免重复发布
// 使用方式：
//   ros2 run legkilo backend_submap_publisher_ros2 \
//       --ros-args -p result_dir:=/abs/path/to/result -p publish_period_sec:=1.0
// ============================================================
class BackendSubmapPublisher : public rclcpp::Node {
   public:
    BackendSubmapPublisher() : Node("backend_submap_publisher") {
        // ROOT_DIR 是 CMake 里注入的宏，指向项目源码根；这里作为默认结果目录
        const std::string default_result_dir = std::string(ROOT_DIR) + "result/temp";
        // ROS2 参数：可以通过 --ros-args -p key:=value 在启动时覆盖
        result_dir_ = declare_parameter<std::string>("result_dir", default_result_dir);
        frame_id_ = declare_parameter<std::string>("frame_id", "camera_init");
        output_topic_ = declare_parameter<std::string>("output_topic", "backend/submaps_colored");
        // 至少 0.1s 一次：避免用户配置 0 导致 CPU 打满
        const double publish_period_sec = std::max(declare_parameter<double>("publish_period_sec", 1.0), 0.1);

        // QoS：KeepLast(1) + reliable + transient_local
        //   - KeepLast(1)       : 只缓存最新一帧
        //   - reliable          : TCP 语义，可靠传输（丢包重传）
        //   - transient_local   : 后加入的订阅者能立刻收到"最近一帧"，
        //                         非常适合"地图/静态数据"这种缓变消息（类似 ROS1 latched）
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
        qos.reliable();
        qos.transient_local();
        publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, qos);

        // 挂钟定时器：每 publish_period_sec 秒回调一次 publishIfUpdated()
        const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(publish_period_sec));
        timer_ = create_wall_timer(period, std::bind(&BackendSubmapPublisher::publishIfUpdated, this));

        RCLCPP_INFO(get_logger(), "Watching backend submaps in: %s", result_dir_.c_str());
        RCLCPP_INFO(get_logger(), "Publishing colorized submaps on: %s", publisher_->get_topic_name());
    }

   private:
    /**
     * @brief 读取 result_dir_/submaps.csv 的所有行，返回解析成功的 SubmapRecord 列表。
     * @param csv_path 目标 csv 绝对路径
     *
     * 【算法要点】
     *  - 首行是表头，直接吞掉；
     *  - 空行跳过；
     *  - 解析失败的行也跳过（parseSubmapRecord 返回 false），不 crash。
     */
    std::vector<SubmapRecord> loadSubmapRecords(const fs::path& csv_path) const {
        std::ifstream stream(csv_path);
        if (!stream.is_open()) return {};

        std::vector<SubmapRecord> records;
        std::string line;
        std::getline(stream, line);  // header  ← 表头
        while (std::getline(stream, line)) {
            if (line.empty()) continue;
            SubmapRecord record;
            if (parseSubmapRecord(line, result_dir_, record)) records.push_back(std::move(record));
        }
        return records;
    }

    /**
     * @brief 定时器回调：若 submaps.csv 有变化，就重新拼接一整包点云并发布。
     *
     * 【算法要点】
     *  1) 用 filesystem::last_write_time 判断 csv 是否被更新，未变则直接返回；
     *  2) 全量加载所有 SubmapRecord（并非增量，因为每次因子图优化后所有位姿都会变）；
     *  3) 对每个 SubmapRecord：
     *      - 加载对应 PCD 到 local_cloud（submap 局部坐标系）；
     *      - 计算 optimized_pose * local_point 得到全局坐标；
     *      - 用 colorForSubmap(id) 给这段点云染色，输出到 pcl::PointXYZRGB；
     *  4) 合并成整包 PointCloud2 → 打时间戳 (now()) + frame_id → publish。
     * 【注意】不做增量更新是刻意的：因子图优化后连历史 submap 的位姿都会变化，
     *         简单粗暴的全量重发效果最好也最直观。
     */
    void publishIfUpdated() {
        const fs::path csv_path = fs::path(result_dir_) / "submaps.csv";
        std::error_code error;
        const auto write_time = fs::last_write_time(csv_path, error);
        if (error) {
            // 文件还没生成（SLAM 还没跑到第一个 submap 收尾）→ 5s 一次的节流警告
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for %s", csv_path.c_str());
            return;
        }
        // 文件时间未变说明 Backend 还没写入新的优化结果，跳过本次发布
        if (has_last_write_time_ && write_time == last_write_time_) return;

        const auto records = loadSubmapRecords(csv_path);
        if (records.empty()) {
            // csv 已生成但暂无有效记录（例如只有表头）→ 更新时间戳并等待下轮
            last_write_time_ = write_time;
            has_last_write_time_ = true;
            RCLCPP_INFO(get_logger(), "No finished submaps recorded yet");
            return;
        }

        // 使用 pcl_utils::makeCloud 屏蔽不同 PCL 版本的 shared_ptr 差异（见 common/pcl_types.h）
        auto output = pcl_utils::makeCloud<pcl::PointXYZRGB>();
        size_t loaded_submaps = 0;
        for (const auto& record : records) {
            PointCloudType local_cloud;
            // PCD 可能被误删/损坏 → 不 abort，只打 warning，继续下一 submap
            if (pcl::io::loadPCDFile(record.pcd_path.string(), local_cloud) != 0) {
                RCLCPP_WARN(get_logger(), "Failed to load submap %ld: %s", static_cast<long>(record.id),
                            record.pcd_path.c_str());
                continue;
            }

            // 为该 submap 生成一个稳定的颜色（同 id → 同颜色，方便反复观察）
            const auto color = colorForSubmap(record.id);
            output->reserve(output->size() + local_cloud.size());
            for (const auto& local_point : local_cloud.points) {
                // Each PCD is stored in its submap-local frame. The optimized graph node pose
                // T_world_submap moves it into the shared global frame after every optimization.
                // ↑ PCD 存的是"submap 局部坐标系"下的点；
                //   optimized_pose = T_world_submap，把点变换到全局系。
                const Eigen::Vector3d global_point =
                    record.optimized_pose * Eigen::Vector3d(local_point.x, local_point.y, local_point.z);
                pcl::PointXYZRGB colored_point;
                colored_point.x = static_cast<float>(global_point.x());
                colored_point.y = static_cast<float>(global_point.y());
                colored_point.z = static_cast<float>(global_point.z());
                colored_point.r = color[0];
                colored_point.g = color[1];
                colored_point.b = color[2];
                output->push_back(colored_point);
            }
            ++loaded_submaps;
        }

        // 所有 submap 都加载失败 → 什么都不发；下轮再试
        if (output->empty()) return;
        // 手动维护 width/height/is_dense（转 ROS 消息时会用到）
        output->width = static_cast<uint32_t>(output->size());
        output->height = 1;
        output->is_dense = false;

        sensor_msgs::msg::PointCloud2 message;
        pcl::toROSMsg(*output, message);
        message.header.stamp = now();          // 用节点当前时间作为发布时间戳
        message.header.frame_id = frame_id_;   // 供 RViz 关联到对应 TF
        publisher_->publish(message);

        // 更新"上次已发布的 csv 修改时间"，防止下一轮重复发同样内容
        last_write_time_ = write_time;
        has_last_write_time_ = true;
        RCLCPP_INFO(get_logger(), "Published %zu colored submaps with %zu points", loaded_submaps, output->size());
    }

    std::string result_dir_;                         // Backend 输出根目录
    std::string frame_id_;                           // 发布点云所在 frame（默认 camera_init）
    std::string output_topic_;                       // 输出话题名
    fs::file_time_type last_write_time_{};           // 上一次成功发布时 csv 的修改时间
    bool has_last_write_time_ = false;               // last_write_time_ 是否已初始化
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;  // 点云发布器
    rclcpp::TimerBase::SharedPtr timer_;             // 挂钟定时器
};

}  // namespace legkilo

int main(int argc, char** argv) {
    // 标准 ROS2 单节点入口：init → spin → shutdown
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<legkilo::BackendSubmapPublisher>());
    rclcpp::shutdown();
    return 0;
}
