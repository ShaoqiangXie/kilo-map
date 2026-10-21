// SPDX-License-Identifier: MIT
// @file kinematics.h
// @brief Quadruped kinematics and contact detection.
// @author Ou Guangjun
// @created 2025-04-26
// @maintainer ouguangjun98@gmail.com
//
// ============================================================
// 【模块概览】腿式机器人运动学 (Kinematics) + 足端接触检测 (ContactDetector)
// 作用：把宇树 Unitree HighState 消息（关节角/关节角速度/足端力/IMU）转换成
//       Kilo-Map 前端可以直接使用的 `common::KinImuMeas`：
//         * 时间戳
//         * IMU 线加速度 / 角速度
//         * 4 条腿的足端相对机体的位置 / 速度（由正向运动学 + 雅可比推出）
//         * 4 条腿的接触状态（由带滞回的力阈值判断）
// 数据流：
//     HighStateMsg ──> Kinematics::processing() ──> common::KinImuMeas
//                                                   └─> 后续用于 ESKF 的
//                                                       腿式速度观测更新
// 备注：该模块本身不做滤波，只做几何/力学换算；与前端类 `KILO` 通过
//       `MeasGroup::kin_imus_` 队列耦合（参见 `sensor_types.hpp`）。
// ============================================================
#ifndef LEG_KILO_KINEMATICS_H
#define LEG_KILO_KINEMATICS_H

#include "common/sensor_types.hpp"
#include "interface/common/ros_compat.h"

namespace legkilo {

// ============================================================
// 【类】ContactDetector
// 作用：单条腿的足端接触状态检测器，本质是"带滞回的施密特触发器"。
// 关键成员：
//   - T_on_       : 触发"抬起→着地"的力阈值上界（力 > T_on_ 才认为接触）
//   - T_off_      : 触发"着地→抬起"的力阈值下界（力 < T_off_ 才认为离地）
//   - in_contact_ : 当前是否处于接触状态，构造时默认为 true
// 使用方式：Kinematics 内部为 4 条腿各持一个实例，每次收到 HighState
//           就把该腿的足端力 `val` 送进 `update()`，返回值即最新的接触布尔。
//           带滞回可以避免力测量抖动时接触状态在阈值附近来回翻转。
// ============================================================
class ContactDetector {
    double T_on_, T_off_;
    bool in_contact_{true};

   public:
    ContactDetector(double ton, double toff) : T_on_(ton), T_off_(toff) {}
    /**
     * @brief 用当前采样力值更新接触状态（滞回逻辑）。
     * @param val 当前采样到的足端法向力
     * @return 更新后的接触状态：true=着地，false=腾空
     *
     * 【算法要点】
     *  - 只有当前"腾空"且 val > T_on_  才切换到"着地"。
     *  - 只有当前"着地"且 val < T_off_ 才切换到"腾空"。
     *  - T_on_ 通常 > T_off_，形成滞回带，抑制噪声导致的抖动切换。
     */
    bool update(double val) {
        if (!in_contact_ && val > T_on_)
            in_contact_ = true;
        else if (in_contact_ && val < T_off_)
            in_contact_ = false;
        return in_contact_;
    }
};

// ============================================================
// 【类】Kinematics
// 作用：宇树四足机器人（Unitree A1/Go1 系列）的正向运动学与足端速度求解器。
//       输入 12 个关节角 + 关节角速度 + IMU + 足端力，输出：
//         - 4 条腿的足端相对机体系 (x,y,z) 位置    (m)
//         - 4 条腿的足端相对机体系 (vx,vy,vz) 速度 (m/s)
//         - 4 条腿的接触状态（由 ContactDetector 判断）
//       这些量随后被 `KILO` 用作"腿式速度观测"，参与 ESKF 更新。
// 关键成员：
//   - ox_, oy_      : 髋关节相对机体几何中心的 X/Y 偏置 (m)
//   - lc_, lt_      : 小腿 (calf) / 大腿 (thigh) 长度 (m)
//   - d_            : 大腿到膝盖的横向偏置 (m)
//   - contacts_     : 4 条腿各自的接触状态检测器
// 腿序约定（本工程 vs. Unitree 原始 SDK）：
//   - 本工程使用: FR(0), FL(1), RR(2), RL(3)
//   - Unitree  使用: FL(0), FR(1), RL(2), RR(3)
//   → 在 processing() 里做了下标重排（详见 .cc 注释）。
// 使用方式：由 RosInterface 在收到 HighState 回调时调用 processing()，
//           输出的 KinImuMeas 会放进 kin_imu 队列，等 lidar 帧对齐后送给 KILO。
// ============================================================
class Kinematics {
   public:
    // Kinematics 的几何 + 力阈值配置，来自 yaml（见 interface/common/options.h）
    struct Config {
        double leg_offset_x;                    // 髋关节 X 向偏置 (m)：前后方向
        double leg_offset_y;                    // 髋关节 Y 向偏置 (m)：左右方向
        double leg_calf_length;                 // 小腿长度 (m)
        double leg_thigh_length;                // 大腿长度 (m)
        double leg_thigh_offset;                // 髋部大腿到膝的横向偏置 d (m)
        double contact_force_threshold_up;      // 力阈值上界：>此值 → 判定为着地
        double contact_force_threshold_down;    // 力阈值下界：<此值 → 判定为腾空
    };

    /**
     * @brief 构造函数：注入机器人几何参数并初始化 4 个接触检测器。
     * @param config 从 yaml 解析出的 Kinematics::Config
     *
     * 4 个 ContactDetector 共用同一组力阈值，构造时默认接触状态 = true。
     */
    explicit Kinematics(const Config& config)
        : ox_(config.leg_offset_x),
          oy_(config.leg_offset_y),
          lc_(config.leg_calf_length),
          lt_(config.leg_thigh_length),
          d_(config.leg_thigh_offset),
          contacts_(std::vector<ContactDetector>(
              4, ContactDetector(config.contact_force_threshold_up, config.contact_force_threshold_down))) {}

    /**
     * @brief 用一条 HighState 消息填满一条 common::KinImuMeas。
     * @param high_state  宇树 HighState（含 IMU + 12 关节 + 4 足端力）
     * @param kin_imu_meas 输出：填入时间戳、IMU、足端 pos/vel、接触状态
     *
     * 【算法要点】
     *  - 时间戳直接取 high_state.stamp（ROS1/ROS2 兼容 toSec）。
     *  - IMU 三轴加速度 + 三轴角速度按分量复制。
     *  - 足端力按"本工程腿序"重排（见 .cc 内部的下标映射注释）。
     *  - 关节角/角速度按每腿 3 关节的顺序取出，传给 caculateFootPosVel。
     * 【注意】本函数不做任何滤波；接触检测的滞回全部封装在 ContactDetector。
     */
    void processing(const ros_compat::HighStateMsg& high_state, common::KinImuMeas& kin_imu_meas);

   private:
    /**
     * @brief 由 12 个关节角 & 角速度解出 4 个足端相对机体的 (x,y,z) 位置与速度。
     * @param foot_angle      [4][3] 4 腿×(髋外展/大腿俯仰/小腿俯仰) 角度 (rad)
     * @param foot_angle_vel  [4][3] 对应角速度 (rad/s)
     * @param foot_pos        [out] 足端相对机体的位置 (m)
     * @param foot_vel        [out] 足端相对机体的速度 (m/s)
     *
     * 【算法要点】
     *  - 正向运动学：三关节串联的解析式（见 .cc 实现），得到足端位置。
     *  - 足端速度：通过雅可比矩阵 J(q) 乘以关节角速度 dq 得到；这里
     *              为效率避免了矩阵库，直接展开 3×3 J 的元素。
     *  - 通过 lfoot / ffoot 两个 ±1 符号处理"左右腿"与"前后腿"的镜像。
     */
    void caculateFootPosVel(const double (&foot_angle)[4][3], const double (&foot_angle_vel)[4][3],
                            double (&foot_pos)[4][3], double (&foot_vel)[4][3]);

    double ox_;                              // 髋关节 X 向偏置 (m)
    double oy_;                              // 髋关节 Y 向偏置 (m)
    double lc_;                              // 小腿长度 (m)
    double lt_;                              // 大腿长度 (m)
    double d_;                               // 大腿→膝的横向偏置 (m)
    std::vector<ContactDetector> contacts_;  // 4 条腿的接触状态检测器
};
}  // namespace legkilo
#endif  // LEG_KILO_KINEMATICS_H
