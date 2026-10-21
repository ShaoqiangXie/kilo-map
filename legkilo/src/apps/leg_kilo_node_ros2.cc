// SPDX-License-Identifier: MIT
// @file leg_kilo_node_ros2.cc
// @brief Main node for Leg KILO SLAM system (ROS2).
//
// ============================================================
// 【模块概览】ROS2 入口节点：legkilo（ROS2 版本）
// 作用：与 ROS1 版本 `leg_kilo_node.cc` 业务逻辑完全一致，仅通信层换成 rclcpp。
// 关键差异：
//   1) 参数解析：rclcpp 会先"消费"掉自己认识的 --ros-args，
//      剩余的普通参数才交给 gflags；因此调用
//        auto non_ros_args = rclcpp::init_and_remove_ros_arguments(argc, argv);
//      之后再把剩余 argv 转成 char* 数组喂给 Logging（内部会调用 gflags parse）。
//   2) Node 用 shared_ptr 管理：`std::make_shared<rclcpp::Node>("legkilo", "legkilo")`
//      第二个参数是命名空间，与 ROS1 NodeHandle("legkilo") 对齐。
//   3) 频率控制用 rclcpp::WallRate（挂钟时间）而非 ros::Rate；
//      运行时 5000 Hz，空闲时 10 Hz，与 ROS1 完全一致。
//   4) 关闭时必须显式 rclcpp::shutdown()，否则子线程可能悬挂。
//
// 与前端/后端/viewer 的耦合关系完全相同，读者可以对照 ROS1 版本理解。
// ============================================================
#include <csignal>
#include <memory>
#include <vector>

#include <unistd.h>

#include <pcl/console/print.h>
#include <rclcpp/rclcpp.hpp>

#include "common/glog_utils.hpp"
#include "common/timer_utils.hpp"
#include "interface/ros2/ros_interface.h"
#include "viewer/viewer_slam_interface.h"

// yaml 配置文件路径，通过 --config_file=/path/to/xxx.yaml 覆盖
DEFINE_string(config_file, "config/leg_fusion.yaml", "Path to the YAML file");

namespace {
// SIGINT (Ctrl+C) 标志，与主循环协作实现优雅退出
volatile std::sig_atomic_t shutdown_requested = 0;
}

// SIGINT 回调：只置位标志，实际清理在主循环退出后
void sigHandle(int sig) { shutdown_requested = 1; }

int main(int argc, char** argv) {
    // ---- 1) rclcpp 先解析并移除 --ros-args 等 ROS2 专用参数；
    //         non_ros_args 里剩下的才是可以喂给 gflags 的普通命令行参数
    //         （比如 --config_file=xxx.yaml）。
    auto non_ros_args = rclcpp::init_and_remove_ros_arguments(argc, argv);
    std::vector<char*> gflags_argv;
    gflags_argv.reserve(non_ros_args.size());
    for (auto& arg : non_ros_args) { gflags_argv.push_back(arg.data()); }

    // ---- 2) 创建 rclcpp::Node，命名空间与节点名都是 "legkilo"（与 ROS1 一致）
    auto node = std::make_shared<rclcpp::Node>("legkilo", "legkilo");

    // ---- 3) Ctrl+C 优雅退出
    signal(SIGINT, sigHandle);

    // ---- 4) 压低 PCL 内部日志等级
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

    // ---- 5) 用"过滤后的"参数初始化 glog（Logging 内部会 gflags::ParseCommandLineFlags）
    std::unique_ptr<legkilo::Logging> logging =
        std::make_unique<legkilo::Logging>(static_cast<int>(gflags_argv.size()), gflags_argv.data(), "logs", 20);

    // ---- 6) 校验 yaml 参数；缺失时立刻 shutdown，避免 rclcpp 悬挂
    if (FLAGS_config_file.empty()) {
        LOG(ERROR) << "YAML configuration file path not provided. Use --config_file=<path>.";
        rclcpp::shutdown();
        return -1;
    }

    // ---- 7) viewer + RosInterface 的构造/初始化顺序与 ROS1 版本相同
    auto viewer_interface = std::make_unique<legkilo::ViewerSlamInterface>(FLAGS_config_file);
    viewer_interface->start();

    std::unique_ptr<legkilo::RosInterface> ros_interface_node =
        std::make_unique<legkilo::RosInterface>(node, viewer_interface.get());
    ros_interface_node->init(FLAGS_config_file);

    LOG(INFO) << "Leg KILO Node Starts";

    // ---- 8) 主循环：与 ROS1 版本完全一致，只是 Rate 类换成 rclcpp::WallRate（挂钟时间）
    //         真正的话题回调都在 RosInterface 内部的子 Executor / 子线程里跑
    rclcpp::WallRate running_rate(5000.0);
    rclcpp::WallRate idle_rate(10.0);
    while (rclcpp::ok() && !shutdown_requested && !viewer_interface->shouldTerminate()) {
        if (!ros_interface_node->isSlamStopped()) { ros_interface_node->run(); }

        if (!ros_interface_node->isSlamStopped() && viewer_interface->consumeSLAMTerminateRequest()) {
            ros_interface_node->stopSlam();
            LOG(INFO) << "SLAM stopped by viewer request";
        }

        ros_interface_node->isSlamStopped() ? idle_rate.sleep() : running_rate.sleep();
    }

    // ---- 9) 顺序停止：先停 SLAM 侧订阅/工作线程，再关 viewer
    ros_interface_node->stop();
    viewer_interface->stop();

    ros_interface_node.reset();
    viewer_interface.reset();
    // rclcpp::shutdown 必须显式调用，否则内部 Executor 可能悬挂，导致进程无法退出
    rclcpp::shutdown();

    LOG(INFO) << "RosInterface destroyed";
    LOG(INFO) << "Leg KILO Node Ends";
    legkilo::Timer::logAllAverTime();
    logging->flushLogFiles();
    return 0;
}
