// SPDX-License-Identifier: MIT
// @file backend_submap_publisher_ros2.cc
// @brief Publish optimized backend submaps as one colorized ROS2 point cloud.

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

struct SubmapRecord {
    int64_t id = -1;
    fs::path pcd_path;
    Eigen::Isometry3d optimized_pose = Eigen::Isometry3d::Identity();
};

std::vector<std::string> splitCsvLine(const std::string& line) {
    std::vector<std::string> columns;
    std::stringstream stream(line);
    std::string column;
    while (std::getline(stream, column, ',')) columns.push_back(column);
    return columns;
}

bool parseSubmapRecord(const std::string& line, const fs::path& result_dir, SubmapRecord& record) {
    const auto columns = splitCsvLine(line);
    if (columns.size() < 16) return false;

    try {
        record.id = std::stoll(columns[0]);
        record.pcd_path = fs::path(columns[1]);
        if (record.pcd_path.is_relative()) record.pcd_path = result_dir / record.pcd_path;

        const Eigen::Vector3d translation(std::stod(columns[9]), std::stod(columns[10]), std::stod(columns[11]));
        Eigen::Quaterniond rotation(std::stod(columns[15]), std::stod(columns[12]), std::stod(columns[13]),
                                    std::stod(columns[14]));
        if (rotation.norm() < 1e-12) return false;
        rotation.normalize();

        record.optimized_pose = Eigen::Isometry3d::Identity();
        record.optimized_pose.linear() = rotation.toRotationMatrix();
        record.optimized_pose.translation() = translation;
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

std::array<uint8_t, 3> colorForSubmap(const int64_t id) {
    // Golden-ratio hue spacing keeps adjacent submap IDs visually distinct.
    constexpr double kGoldenRatioConjugate = 0.6180339887498949;
    const double hue = std::fmod(static_cast<double>(std::max<int64_t>(id, 0)) * kGoldenRatioConjugate, 1.0);
    constexpr double saturation = 0.78;
    constexpr double value = 1.0;

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
        case 0: r = value; g = t; b = p; break;
        case 1: r = q; g = value; b = p; break;
        case 2: r = p; g = value; b = t; break;
        case 3: r = p; g = q; b = value; break;
        case 4: r = t; g = p; b = value; break;
        default: r = value; g = p; b = q; break;
    }

    return {static_cast<uint8_t>(std::lround(r * 255.0)), static_cast<uint8_t>(std::lround(g * 255.0)),
            static_cast<uint8_t>(std::lround(b * 255.0))};
}

}  // namespace

class BackendSubmapPublisher : public rclcpp::Node {
   public:
    BackendSubmapPublisher() : Node("backend_submap_publisher") {
        const std::string default_result_dir = std::string(ROOT_DIR) + "result/temp";
        result_dir_ = declare_parameter<std::string>("result_dir", default_result_dir);
        frame_id_ = declare_parameter<std::string>("frame_id", "camera_init");
        output_topic_ = declare_parameter<std::string>("output_topic", "backend/submaps_colored");
        const double publish_period_sec = std::max(declare_parameter<double>("publish_period_sec", 1.0), 0.1);

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
        qos.reliable();
        qos.transient_local();
        publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, qos);

        const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(publish_period_sec));
        timer_ = create_wall_timer(period, std::bind(&BackendSubmapPublisher::publishIfUpdated, this));

        RCLCPP_INFO(get_logger(), "Watching backend submaps in: %s", result_dir_.c_str());
        RCLCPP_INFO(get_logger(), "Publishing colorized submaps on: %s", publisher_->get_topic_name());
    }

   private:
    std::vector<SubmapRecord> loadSubmapRecords(const fs::path& csv_path) const {
        std::ifstream stream(csv_path);
        if (!stream.is_open()) return {};

        std::vector<SubmapRecord> records;
        std::string line;
        std::getline(stream, line);  // header
        while (std::getline(stream, line)) {
            if (line.empty()) continue;
            SubmapRecord record;
            if (parseSubmapRecord(line, result_dir_, record)) records.push_back(std::move(record));
        }
        return records;
    }

    void publishIfUpdated() {
        const fs::path csv_path = fs::path(result_dir_) / "submaps.csv";
        std::error_code error;
        const auto write_time = fs::last_write_time(csv_path, error);
        if (error) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for %s", csv_path.c_str());
            return;
        }
        if (has_last_write_time_ && write_time == last_write_time_) return;

        const auto records = loadSubmapRecords(csv_path);
        if (records.empty()) {
            last_write_time_ = write_time;
            has_last_write_time_ = true;
            RCLCPP_INFO(get_logger(), "No finished submaps recorded yet");
            return;
        }

        auto output = pcl_utils::makeCloud<pcl::PointXYZRGB>();
        size_t loaded_submaps = 0;
        for (const auto& record : records) {
            PointCloudType local_cloud;
            if (pcl::io::loadPCDFile(record.pcd_path.string(), local_cloud) != 0) {
                RCLCPP_WARN(get_logger(), "Failed to load submap %ld: %s", static_cast<long>(record.id),
                            record.pcd_path.c_str());
                continue;
            }

            const auto color = colorForSubmap(record.id);
            output->reserve(output->size() + local_cloud.size());
            for (const auto& local_point : local_cloud.points) {
                // Each PCD is stored in its submap-local frame. The optimized graph node pose
                // T_world_submap moves it into the shared global frame after every optimization.
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

        if (output->empty()) return;
        output->width = static_cast<uint32_t>(output->size());
        output->height = 1;
        output->is_dense = false;

        sensor_msgs::msg::PointCloud2 message;
        pcl::toROSMsg(*output, message);
        message.header.stamp = now();
        message.header.frame_id = frame_id_;
        publisher_->publish(message);

        last_write_time_ = write_time;
        has_last_write_time_ = true;
        RCLCPP_INFO(get_logger(), "Published %zu colored submaps with %zu points", loaded_submaps, output->size());
    }

    std::string result_dir_;
    std::string frame_id_;
    std::string output_topic_;
    fs::file_time_type last_write_time_{};
    bool has_last_write_time_ = false;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace legkilo

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<legkilo::BackendSubmapPublisher>());
    rclcpp::shutdown();
    return 0;
}
