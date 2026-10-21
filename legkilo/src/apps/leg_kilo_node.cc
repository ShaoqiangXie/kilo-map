// SPDX-License-Identifier: MIT
// @file leg_kilo_node.cc
// @brief Main node for Leg KILO SLAM system(ros1).
// @author Ou Guangjun
// @created 2024-12-17
// @maintainer ouguangjun98@gmail.com
//
// ============================================================
// 【模块概览】ROS1 入口节点：legkilo
// 作用：把 SLAM 各模块拼装起来，进入 ROS spin 循环。
// 启动流程（main()）：
//   1) ros::init + NodeHandle("legkilo")           // ROS1 通信初始化
//   2) 注册 SIGINT 信号处理，方便 Ctrl+C 优雅退出
//   3) 关闭 PCL 无关 warning 日志（避免刷屏）
//   4) 构造 legkilo::Logging → 初始化 glog（写到 logs/，保留 20 个文件）
//   5) 读取命令行 --config_file，作为 yaml 路径
//   6) 构造 ViewerSlamInterface（GUI/可视化），并启动其工作线程
//   7) 构造 RosInterface：内部完成
//        - LidarProcessing / Kinematics 构造
//        - KILO 前端 + Backend 后端构造
//        - 订阅 lidar/imu/joint 话题，创建独立子线程 spin
//   8) 主循环：
//        - 运行状态下 5000 Hz 空转（其实是让 CPU 让出，实际工作在子线程）
//        - 停止状态下 10 Hz 空转
//        - 若 viewer 请求"停止 SLAM"，调用 stopSlam()
//        - 若 viewer 请求"终止程序"，跳出循环
//   9) 关闭顺序：ros_interface_node->stop() → viewer_interface->stop()
//                → 打印 Timer 统计 → flush 日志 → 退出
// 与 ROS2 版本的区别：ROS1 用 ros::init/NodeHandle/ros::Rate；
//                     ROS2 版本使用 rclcpp。业务逻辑完全一致，见
//                     `apps/leg_kilo_node_ros2.cc`。
// ============================================================
#include <csignal>
#include <memory>

#include <unistd.h>

#include <pcl/console/print.h>

#include "common/glog_utils.hpp"
#include "common/timer_utils.hpp"
#include "interface/ros1/ros_interface.h"
#include "viewer/viewer_slam_interface.h"

// yaml 配置文件路径，通过 --config_file=/path/to/xxx.yaml 覆盖
DEFINE_string(config_file, "config/leg_fusion.yaml", "Path to the YAML file");//参数名 默认值 参数说明
namespace {
// 简单的原子标志：SIGINT 时置 1，主循环观察到就退出（volatile+sig_atomic_t 保证
// 信号处理函数与主线程可见性）
volatile std::sig_atomic_t shutdown_requested = 0;
}

// SIGINT (Ctrl+C) 信号回调：只做一件事——置位标志，让主循环下一轮跳出。
void sigHandle(int sig) { shutdown_requested = 1; }

int main(int argc, char** argv) {
    // ---- 1) ROS1 通信初始化：注册节点名 "legkilo"，随后创建同名命名空间的 NodeHandle
    ros::init(argc, argv, "legkilo");
    ros::NodeHandle nh("legkilo");

    // ---- 2) Ctrl+C 优雅退出：只置位 shutdown_requested，真正关闭放在主循环退出后
    signal(SIGINT, sigHandle);//程序临时执行 sigHandle(SIGINT)

    // ---- 3) 压低 PCL 内部日志的等级，避免匹配失败等 warning 刷屏
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

    // ---- 4) 初始化 glog：日志目录 "logs"，最多保留 20 个日志文件
    std::unique_ptr<legkilo::Logging> logging = std::make_unique<legkilo::Logging>(argc, argv, "logs", 20);

    // ---- 5) 校验命令行参数：必须显式提供 yaml 路径
    if (FLAGS_config_file.empty()) {
        LOG(ERROR) << "YAML configuration file path not provided. Use --config_path=<path>.";
        return -1;
    }

    // ---- 6) 构造可视化 (viewer) 并启动它自己的渲染/工作线程
    //         viewer 通过原子标志与主线程通信，可主动请求"停止 SLAM"或"终止程序"。
    auto viewer_interface = std::make_unique<legkilo::ViewerSlamInterface>(FLAGS_config_file);
    viewer_interface->start();

    // ---- 7) 构造 RosInterface：它负责所有 ROS 订阅/发布 + SLAM 前后端组装。
    //         viewer 指针作为观察者传入，SLAM 结果会 push 到 viewer 供渲染。
    std::unique_ptr<legkilo::RosInterface> ros_interface_node =
        std::make_unique<legkilo::RosInterface>(nh, viewer_interface.get());

    // 读 yaml → 构造 LidarProcessing/Kinematics/KILO/Backend；
    // 建立 lidar/imu/joint 三个订阅子线程（在 RosInterface 内部）。
    ros_interface_node->init(FLAGS_config_file);

    LOG(INFO) << "Leg KILO Node Starts";

    // ---- 8) 主循环：真正的传感器回调都跑在 RosInterface 内部子线程里。
    //         这里主线程只做两件事：驱动 run()（把已同步好的 MeasGroup 送给 KILO），
    //         以及响应 viewer 的两种控制请求。
    ros::Rate running_rate(5000);  // 高频空转 ≈ 0.2 ms，防止 while 空跑吃 CPU
    ros::Rate idle_rate(10);       // SLAM 停止后进入低频等待，节省资源
    while (ros::ok() && !shutdown_requested && !viewer_interface->shouldTerminate()) {
        // (a) 正常状态下推动 SLAM 主管线（syncPackage + KILO::process + Backend::addFrame ...）
        if (!ros_interface_node->isSlamStopped()) { ros_interface_node->run(); }

        // (b) viewer 请求"暂停/停止 SLAM"（例如用户点了 GUI 里的 Stop 按钮）
        if (!ros_interface_node->isSlamStopped() && viewer_interface->consumeSLAMTerminateRequest()) {
            ros_interface_node->stopSlam();
            LOG(INFO) << "SLAM stopped by viewer request";
        }

        // (c) 根据是否还在跑 SLAM 选择合适的睡眠频率
        ros_interface_node->isSlamStopped() ? idle_rate.sleep() : running_rate.sleep();
    }

    // ---- 9) 关闭顺序（顺序很重要，防止 viewer 还在渲染时 SLAM 已析构）
    ros_interface_node->stop();      // 停订阅子线程、清空缓冲队列
    viewer_interface->stop();        // 停可视化工作线程

    // 显式 reset 触发析构，确保内部资源在打印 log 前释放完毕
    ros_interface_node.reset();
    viewer_interface.reset();
    LOG(INFO) << "RosInterface destroyed";
    LOG(INFO) << "Leg KILO Node Ends";
    legkilo::Timer::logAllAverTime();  // 输出所有 Timer 埋点的平均耗时统计
    logging->flushLogFiles();          // 强制把 glog 缓冲写盘
    return 0;
}
