// SPDX-License-Identifier: MIT
// @file state_initial.hpp
// @brief State initialization using IMU or Kin+IMU.
// @author Ou Guangjun
// @created 2025-04-26
// @maintainer ouguangjun98@gmail.com
//
// ============================================================
// 【模块概览】ESKF 初始状态估计器 (StateInitial)
// 作用：在 SLAM 正式跑起来之前，利用一小段"机器人静止"数据估计：
//         1) 初始重力方向     g_b   （在机体系下）
//         2) 陀螺仪 bias       bg    （静止时角速度均值就是 bias）
//         3) 初始姿态 R_wb    （把 g_b 对齐到世界系 -z）
//       这些量决定 ESKF 起步是否稳定。若初始重力/姿态错，会立刻整体倾斜。
//
// 【init_type 三种选择】（对应 yaml 中 `initialization.init_type`）：
//   - 0 / 未指定 → 由前端根据传感器类型选择合适的子类，通常等价于下面 1。
//   - 1 = InitType::Identity          恒等初始化：
//         * 直接令 R_wb = I，认为机器人正立摆放；
//         * 重力向量取 -g·(mean_acc / |mean_acc|)，即用 IMU 平均加速度反推；
//         * 若 |mean_acc| ≈ 0，则退化为 (0,0,-g)。
//   - 2 = InitType::GravityAlignment  重力对齐初始化：
//         * 用一段静止 IMU 计算 mean_acc；
//         * 定义机体系测得的重力方向 acc_dir = -mean_acc / |mean_acc|（因为静止时
//           IMU 会测到反向重力），把它旋转对齐到世界 -z 得到初始 R_wb；
//         * 世界重力直接固定为 (0,0,-g)。
//         * 陀螺 bias = 静止段陀螺均值。
//         * 这是最常用/最鲁棒的初始化方式。
//
// 【三种子类】
//   - StateInitialByImu    : 用纯 IMU（LIO 模式）
//   - StateInitialByKinImu : 用腿式运动学 + IMU 里的 IMU 部分（KILO 模式）
//   - StateInitialByLidar  : 纯 LiDAR 模式（LO），不做真正的重力/姿态估计，
//                            全部返回单位阵/零向量，由 LiDAR 匹配自行"漂移到位"。
//
// 使用方式：前端 KILO::initializeFromYaml 里根据 SensorType/InitType 构造对应子类，
//           每次调用 process(MeasGroup) 前先 ingest 一批数据；当 isReady() 后，
//           读取 gravityVec/gyroBias/initRotation 填 ESKF 的初值。
// ============================================================
#ifndef LEG_KILO_STATE_INITIAL_HPP
#define LEG_KILO_STATE_INITIAL_HPP

#include <algorithm>
#include <deque>

#include "common/math_utils.hpp"
#include "common/sensor_types.hpp"

namespace legkilo {

// 初始化模式枚举，与 yaml 里 `init_type` 的整数值对齐（Identity=1, GravityAlignment=2）
enum class InitType { Identity = 1, GravityAlignment = 2 };

// ============================================================
// 【基类】StateInitial
// 作用：统一的初始状态估计器接口，内部维护"IMU 增量均值 + 样本计数"。
// 关键成员：
//   - cfg_        : 重力常数、最少样本数、初始化类型
//   - N_          : 已累计的静止样本数
//   - mean_acc_   : 静止段加速度均值（机体系）
//   - mean_gyr_   : 静止段角速度均值 = 陀螺 bias 估计
//   - acc_norm_   : |mean_acc_|，用于把"IMU 平均加速度"归一化成重力方向
// 使用方式：
//   - 派生类实现 ingest()：把一批 MeasGroup 中的 IMU/腿式数据喂进来。
//   - 每次 ingest 内部会调用 updateMean() 做在线均值更新（Welford 递推）。
//   - 当 isReady() = true 时，调用 gravityVec()/gyroBias()/initRotation()
//     取到初始状态；这些结果的语义随 init_type 变化，见 hpp 顶部说明。
// ============================================================
class StateInitial {
   public:
    struct Config {
        double gravity = 9.81;             // 世界系重力大小 (m/s^2)
        size_t min_samples = 1;            // 至少累计多少样本才算就绪
        InitType init_type = InitType::Identity;  // Identity 或 GravityAlignment
    };

    explicit StateInitial(const Config& cfg) : cfg_(cfg) { reset(); }
    virtual ~StateInitial() = default;

    /**
     * @brief 派生类实现：把一个 MeasGroup 中可用的样本吸收进来。
     * @return 本次实际吸收的样本数（IMU 帧数或 KinImu 帧数）。
     */
    virtual size_t ingest(const common::MeasGroup& meas) = 0;

    /** @brief 是否已经累计了足够样本，可以拿去做 ESKF 初值。 */
    bool isReady() const { return N_ >= static_cast<int>(cfg_.min_samples); }

    /** @brief 清空计数与均值，供重新初始化时调用。 */
    void reset() {
        N_ = 0;
        mean_acc_.setZero();
        mean_gyr_.setZero();
        acc_norm_ = 0.0;
    }

    /** @brief 返回 |mean_acc_|，接近 gravity 时说明静止段稳定。 */
    virtual double accNorm() const { return acc_norm_; }

    /**
     * @brief 世界系下的重力向量（m/s^2）。
     *
     * 【算法要点】
     *  - GravityAlignment：重力被固定为 (0, 0, -g)，
     *    因为姿态那边已经把机体系加速度旋到了世界 -z。
     *  - 否则（Identity）：直接把 IMU 平均加速度归一化后乘以 -g；
     *    这样保证 world_grav 与 IMU 实际测量方向自洽。
     *  - 若 |mean_acc| == 0（比如还没喂数据），兜底返回 (0,0,-g)。
     */
    virtual Eigen::Vector3d gravityVec() const {
        if (cfg_.init_type == InitType::GravityAlignment) { return Eigen::Vector3d(0, 0, -cfg_.gravity); }
        if (acc_norm_ <= 0.0) return Eigen::Vector3d(0, 0, -cfg_.gravity);
        return -cfg_.gravity * (mean_acc_ / acc_norm_);
    }

    /** @brief 陀螺 bias 估计 = 静止段角速度均值。 */
    virtual Eigen::Vector3d gyroBias() const { return mean_gyr_; }

    /**
     * @brief 初始姿态 R_wb（把机体系变换到世界系的旋转）。
     *
     * 【算法要点】
     *  - GravityAlignment 模式下：
     *      * 机体系测得的重力方向 acc_dir = -mean_acc / |mean_acc|
     *        （静止时 IMU 会读到反向重力，所以要取负号）
     *      * 目标世界系重力方向 g_world = (0, 0, -1)
     *      * 用 Eigen::Quaterniond::FromTwoVectors 求"把 acc_dir 转到 g_world"的最小旋转
     *      * 得到 R_wb 即初始姿态；此姿态下机身"竖直"，只有航向（yaw）不确定。
     *  - Identity 模式下（或 acc_norm_ 为 0）：直接返回单位阵。
     * 【注意】yaw 无法被静态 IMU 观测到；如需 yaw 请引入磁力计或 LiDAR/视觉。
     */
    virtual Eigen::Matrix3d initRotation() const {
        if (cfg_.init_type == InitType::GravityAlignment && acc_norm_ > 0.0) {
            Eigen::Vector3d acc_dir = -mean_acc_.normalized();
            Eigen::Vector3d g_world(0, 0, -1.0);
            Eigen::Quaterniond q0 = Eigen::Quaterniond::FromTwoVectors(acc_dir, g_world);
            q0.normalize();
            return q0.toRotationMatrix();
        }
        return Eigen::Matrix3d::Identity();
    }

   protected:
    /**
     * @brief 在线均值递推（Welford / 稳定滑动均值）：mean += (x - mean) / N。
     *        每加入 1 个样本 (acc, gyr)，N 自增，mean_acc_/mean_gyr_ 也随之更新，
     *        避免累加溢出并且数值稳定。
     */
    inline void updateMean(const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr) {
        N_ += 1;
        const double invN = 1.0 / static_cast<double>(N_);
        mean_acc_ += (acc - mean_acc_) * invN;
        mean_gyr_ += (gyr - mean_gyr_) * invN;
    }

    Config cfg_;
    int N_ = 0;
    Eigen::Vector3d mean_acc_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d mean_gyr_ = Eigen::Vector3d::Zero();
    double acc_norm_ = 0.0;
};

// ============================================================
// 【子类】StateInitialByImu ── 面向 LIO 模式（只有独立 IMU 话题）
// 数据源：meas.imus_，每条是一个 sensor_msgs::Imu
// 用法：静止一段时间，累计足够 IMU 帧就 ready。
// ============================================================
class StateInitialByImu : public StateInitial {
   public:
    using StateInitial::StateInitial;
    size_t ingest(const common::MeasGroup& meas) override {
        size_t added = 0;
        // 遍历本次 MeasGroup 里的所有 IMU 帧，逐条更新均值
        for (const auto& imu : meas.imus_) {
            const auto& a = imu->linear_acceleration;
            const auto& g = imu->angular_velocity;
            const Eigen::Vector3d acc(a.x, a.y, a.z);
            const Eigen::Vector3d gyr(g.x, g.y, g.z);
            updateMean(acc, gyr);
            ++added;
        }
        // 更新平均加速度模长（用于 GravityAlignment 判断静止段是否稳定）
        if (N_ > 0) acc_norm_ = mean_acc_.norm();
        return added;
    }
};

// ============================================================
// 【子类】StateInitialByLidar ── 面向 LO 模式（纯 LiDAR，无 IMU）
// 数据源：不使用任何数据。
// 说明：纯 LiDAR 模式不做重力/姿态估计，全部返回单位/零，让第一帧 ICP 自然对齐；
//       这里 ingest 只是"把 N_ 直接抬到 min_samples"以立即返回 ready。
// ============================================================
class StateInitialByLidar : public StateInitial {
   public:
    using StateInitial::StateInitial;
    size_t ingest(const common::MeasGroup& meas) override {
        (void)meas;                                              // 明确忽略输入
        N_ = std::max(N_, static_cast<int>(cfg_.min_samples));   // 立即 ready
        acc_norm_ = cfg_.gravity;                                // 象征性写入，避免 0
        return 0;                                                // 没有真正吸收样本
    }
    // LO 模式下重力/姿态/bias 均无意义，统一返回零向量或单位阵：
    double accNorm() const override { return cfg_.gravity; }
    Eigen::Vector3d gravityVec() const override { return Eigen::Vector3d::Zero(); }
    Eigen::Vector3d gyroBias() const override { return Eigen::Vector3d::Zero(); }
    Eigen::Matrix3d initRotation() const override { return Eigen::Matrix3d::Identity(); }
};

// ============================================================
// 【子类】StateInitialByKinImu ── 面向 KILO 模式（腿式运动学 + IMU）
// 数据源：meas.kin_imus_，每条 KinImuMeas 内含 IMU 加速度/角速度
// 说明：与 ByImu 完全同思路，只是从 KinImuMeas 结构里取 IMU 分量。
// ============================================================
class StateInitialByKinImu : public StateInitial {
   public:
    using StateInitial::StateInitial;
    size_t ingest(const common::MeasGroup& meas) override {
        size_t added = 0;
        for (const auto& ki : meas.kin_imus_) {
            const Eigen::Vector3d acc(ki.acc_[0], ki.acc_[1], ki.acc_[2]);
            const Eigen::Vector3d gyr(ki.gyr_[0], ki.gyr_[1], ki.gyr_[2]);
            updateMean(acc, gyr);
            ++added;
        }
        if (N_ > 0) acc_norm_ = mean_acc_.norm();
        return added;
    }
};

}  // namespace legkilo
#endif  // LEG_KILO_STATE_INITIAL_HPP
