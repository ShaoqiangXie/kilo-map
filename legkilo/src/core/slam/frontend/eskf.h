// SPDX-License-Identifier: MIT
// @file eskf.h
// @brief 前端误差状态卡尔曼滤波 (ESKF/IESKF) 接口：预测方程 f、雅可比 F、以及四类观测更新。
// @author Ou Guangjun
// @created 2024-12-17
// @maintainer ouguangjun98@gmail.com
//
// -----------------------------------------------------------------------------
// 【模块概览】
//   本类实现了 legkilo 前端使用的误差状态滤波器：
//     ┌─────────────┐    predict()        ┌──────────────┐
//     │  上一时刻   │ ──────────────►     │  当前时刻    │
//     │  x, P       │  (IMU / CV 模型)    │  x_pred, P_p │
//     └─────────────┘                     └──────┬───────┘
//                                                │
//         updateByPoints (逐点/桶 P2P/NDT)       │
//         updateByCloud  (整帧 IESKF 迭代)       │
//         updateByImu    (Point-LIO 风格 IMU)    │  4 类观测通道
//         updateByKinImu (腿式运动学 + IMU)      │
//                                                ▼
//                                         x_post, P_post
//
// 【预测模型】(PredictModelType)
//   - Inertial          ：使用 IMU 加速度/角速度做惯性积分；标准 LIO 模式。
//   - ConstantVelocity  ：LO（纯激光）模式，用上一帧估计出的世界系速度/角速度线性外推。
//
// 【观测通道】
//   - updateByPoints : 一维/多维标量观测（P2P/NDT 残差），H 只有 6 列（作用在 rot & pos）。
//   - updateByCloud  : IESKF 整帧迭代，每次迭代先重线性化再做一次 Kalman step，直到收敛。
//   - updateByImu    : 观测量是"IMU 加速度/角速度 - 估计的体系加速度/角速度 - 零偏"，
//                      构造出对状态 [9:15) (零偏) 和 [18:24) (imu_a/imu_w) 的信息。
//   - updateByKinImu : KILO 模式，除 IMU 之外把足端运动学速度也当作观测。
// -----------------------------------------------------------------------------
#ifndef LEG_KILO_ESKF_H
#define LEG_KILO_ESKF_H

#include "common/math_utils.hpp"
#include "common/sensor_types.hpp"
#include "core/slam/frontend/State.h"

namespace legkilo {

// ============================================================
// 【结构体】ObsShared
// 作用：把不同观测通道的 z / H / R 统一打包给 ESKF::updateByXxx 使用。
//       通过前缀 `pt_*` / `ki_*` 区分：
//         - pt_*  : LiDAR 点观测（H 只有 6 列，仅作用在 rot(0:3) + pos(3:6)）
//         - ki_*  : IMU / KinImu 观测（H 是 DIM_STATE=24 列，涉及更多状态维度）
// 关键成员：
//   - pt_z  : (M,1)  单点/多点残差向量（P2P 是标量拼接；NDT 每点占 3 行）
//   - pt_h  : (M,6)  雅可比：pt_h.row(i) = [ ∂r/∂δθ  ∂r/∂δp ]
//   - pt_R  : (M,1)  观测方差对角线（robust kernel 会把它放大以降低权重）
//   - ki_z / ki_h / ki_R : IMU / KinImu 观测的 z, H, R
// ============================================================
struct ObsShared {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // ---- LiDAR 点观测（P2P / NDT）：H 只有 6 列（对应 δθ, δp）----
    Eigen::Matrix<double, Eigen::Dynamic, 1> pt_z;  // 观测残差 z（每点 1 行 P2P，或每点 3 行 NDT）
    Eigen::Matrix<double, Eigen::Dynamic, 6> pt_h;  // 观测雅可比 H（只对 rot+pos，其他列全 0）
    Eigen::Matrix<double, Eigen::Dynamic, 1> pt_R;  // 观测方差对角线（Cauchy kernel 后的方差）

    // ---- IMU / KinImu 观测：H 与整个 DIM_STATE 都可能相关 ----
    Eigen::Matrix<double, Eigen::Dynamic, 1> ki_z;
    Eigen::Matrix<double, Eigen::Dynamic, DIM_STATE> ki_h;
    Eigen::Matrix<double, Eigen::Dynamic, 1> ki_R;
};

// ============================================================
// 【结构体】EskfProcessNoise
// 作用：过程噪声 Q 的参数（连续谱密度，构建 Q 时会以对角形式填入对应块）。
//   - Inertial 模式下：Q 影响 [vel, ba_bias, bw_bias, imu_a, imu_w] 这几个状态块；
//   - ConstantVelocity 模式：只需要 gyr_process_cov 和 vel_process_cov。
// 具体 Q 的构造见 ESKF::rebuildProcessNoise。
// ============================================================
struct EskfProcessNoise {
    double vel_process_cov = 0.0;       // 速度过程噪声（LIO 用在 vel 块；LO 用在 world_vel 输入块）
    double imu_acc_process_cov = 0.0;   // IMU 加速度过程噪声（打到 imu_a_ 状态块）
    double imu_gyr_process_cov = 0.0;   // IMU 角速度过程噪声
    double acc_bias_process_cov = 0.0;  // 加速度计零偏随机游走
    double gyr_bias_process_cov = 0.0;  // 陀螺零偏随机游走
};

// 预测模型开关：Inertial = IMU 驱动；ConstantVelocity = LO 模式匀速外推
enum class PredictModelType { Inertial = 0, ConstantVelocity = 1 };

// ============================================================
// 【类】ESKF
// 作用：24 维误差状态卡尔曼滤波器；封装名义状态 + 协方差 + 过程噪声 +
//       4 种观测通道 (点云、整帧 IESKF、IMU、KinImu)。
// 关键成员：
//   - state_         : 名义状态（class State）
//   - cov_           : 24×24 协方差 P
//   - Q_             : 过程噪声，随 predict_model_ 切换而重建
//   - predict_model_ : Inertial / ConstantVelocity
// 使用方式：
//   - 由 KILO 类持有 `std::unique_ptr<ESKF> eskf_`；
//   - 每次 predict：eskf_->predict(dt, input, prop_state, prop_cov);
//   - LiDAR 点观测：eskf_->updateByPoints(obs) / eskf_->updateByCloud(obs, prior, N, k);
//   - IMU / KinImu：eskf_->updateByImu(obs) / eskf_->updateByKinImu(obs);
// ============================================================
class ESKF {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // ---------- 预测阶段的外部输入 ----------
    // Inertial 模式：使用 imu_acc / imu_gyr
    // ConstantVelocity 模式：使用 world_vel + imu_gyr（LO 场景下由前一帧估计出）
    struct PredictInput {
        Vec3D imu_acc = Vec3D::Zero();   // 机体系加速度（去零偏后? 见 eskf.cc 里 f 的实现）
        Vec3D imu_gyr = Vec3D::Zero();   // 机体系角速度
        Vec3D world_vel = Vec3D::Zero();  // 世界系速度（仅 CV 模式使用）
    };

    // 构造：直接以过程噪声参数初始化，并按默认 predict_model_ 构建 Q。
    ESKF(const EskfProcessNoise& noise);

    // ---------- 名义状态读写 ----------
    State& state() { return state_; }
    const State& state() const { return state_; }
    void setState(const State& state) { state_ = state; }
    State getState() const { return state_; }

    Mat3D getRot() const { return state_.rot_; }
    Vec3D getPos() const { return state_.pos_; }
    Vec3D getVel() const { return state_.vel_; }

    // 取协方差 P 的 3×3 子块：rot、pos、vel 各自的不确定性
    Mat3D getRotCov() const { return cov_.block<3, 3>(0, 0); }
    Mat3D getPosCov() const { return cov_.block<3, 3>(3, 3); }
    Mat3D getVelCov() const { return cov_.block<3, 3>(6, 6); }

    // ---------- 过程噪声读写 ----------
    StateQ& Q() { return Q_; }
    const StateQ& Q() const { return Q_; }
    void setQ(const StateQ& Q) { Q_ = Q; }

    PredictModelType predictModel() const { return predict_model_; }

    /**
     * @brief  切换预测模型（Inertial ↔ ConstantVelocity），并触发 Q 的重建。
     * @param  predict_model 目标模型
     * 【场景】KILO 里根据 sensor_type_(LO/LIO/KILO) 决定使用哪种预测模型。
     */
    void setPredictModel(PredictModelType predict_model);

    StateCov& cov() { return cov_; }
    const StateCov& cov() const { return cov_; }

    const EskfProcessNoise& processNoise() const { return process_noise_; }

    /**
     * @brief  计算离散状态转移增量 f(x, u)·dt（即名义状态在 dt 内的变化）。
     * @return 24 维列向量，直接可以送给 `state_ += ...` 完成 boxplus。
     *
     * 根据 predict_model_ 分派到 getFunctionfInertial 或 getFunctionfConstantVelocity。
     */
    StateVec getFunctionf(double dt, const PredictInput& input) const;

    /**
     * @brief  计算离散状态转移雅可比 Fx = I + F(x,u)·dt。
     * @return 24×24 矩阵，用于协方差传播 P ← Fx · P · Fx^T + dt^2·Q。
     */
    StateF getFx(double dt, const PredictInput& input) const;

    /**
     * @brief  ESKF 预测：一步向前推名义状态与/或协方差。
     * @param  dt        时间间隔（秒）
     * @param  input     外部输入（IMU 或 world_vel）
     * @param  prop_state 是否推名义状态 (state_ += f·dt)
     * @param  prop_cov   是否推协方差   (P = Fx P Fx^T + dt^2 Q)
     *
     * 【为什么把 state / cov 分开控制】
     *   KILO 里为了让"预测协方差"覆盖整段时间、"预测状态"从上次名义状态时刻起算：
     *     eskf.predict(dt_cov, input, false, true);   // 只推协方差 (从 update 时刻开始)
     *     eskf.predict(dt,     input, true,  false);  // 只推状态 (从 last_predict 时刻开始)
     */
    void predict(double dt, const PredictInput& input, bool prop_state, bool prop_cov);

    /**
     * @brief  用 LiDAR 点云残差（P2P/NDT，H 只有 6 列）做一次卡尔曼更新。
     *
     * 内部会根据观测维度 M 选择三种数值路径：
     *   - M == 1               ：标量更新，避免矩阵求逆（速度最快）
     *   - 1 < M ≤ DIM_STATE    ：小矩阵求逆 (M×M)
     *   - M > DIM_STATE        ：用信息形式 P⁻¹ + Hᵀ R⁻¹ H 再求逆，避免 M×M 大矩阵
     *
     * 【H 结构】只有前 6 列非零（对应 rot,pos），所以在协方差更新里也只需要取
     * cov_.block<*,6>(...) 参与运算，其他列自动被 0 消掉。
     */
    void updateByPoints(const ObsShared& obs_shared);

    /**
     * @brief  IESKF (Iterated ESKF) 整帧更新：在同一批观测上迭代重线性化。
     * @param  obs_shared   当前迭代重新构造的 z / H / R
     * @param  prior_state  本帧迭代开始之前的先验状态 x_k^{prior}
     * @param  iter_max     最大迭代次数
     * @param  iter_cur     当前迭代序号（0..iter_max-1）
     * @return              是否已收敛
     *
     * 【算法要点】
     *  - 第 0 次迭代：δx = K · z  （标准 EKF）
     *  - 之后每次：   δx = K·z + Δprior - K·H·Δprior_pose
     *     其中 Δprior = prior_state ⊟ state_，表示先验相对当前迭代点的偏差。
     *  - 收敛判据：δθ 弧度换算成度 < 0.1 且 δp 换算成 cm < 0.1。
     *  - 只有在收敛或达到最大迭代时才更新协方差 P (Joseph 型简化)，中间迭代仅更新均值。
     */
    bool updateByCloud(const ObsShared& obs_shared, const State& prior_state, int iter_max, int iter_cur);

    /**
     * @brief  Point-LIO 风格 IMU 更新：把 IMU 原始加速度/角速度作为对
     *         "imu_a_ + ba"、"imu_w_ + bw" 的联合观测。
     *
     * H 具有稀疏结构：H = [0, 0, 0, I₆, 0, 0, I₆, 0]（简化写法），
     * 因此 K, KH 计算被手工展开成对 col=9/18 / row=9/18 的 6×6 子块运算。
     */
    void updateByImu(const ObsShared& obs_shared);

    /**
     * @brief  KILO 模式：IMU + 足端运动学观测。
     * H 是稠密的 (6 + 3·contact_num) × DIM_STATE 矩阵，因此走通用稠密卡尔曼路径。
     */
    void updateByKinImu(const ObsShared& obs_shared);

   private:
    // 私有默认构造：禁止外部无参数构造（必须提供噪声）。
    ESKF() = default;

    // ---- 分派实现：Inertial 与 ConstantVelocity 分别有各自的 f(x,u), Fx ----
    StateVec getFunctionfInertial(double dt, const PredictInput& input) const;
    StateVec getFunctionfConstantVelocity(double dt, const PredictInput& input) const;
    StateF getFxInertial(double dt, const PredictInput& input) const;
    StateF getFxConstantVelocity(double dt, const PredictInput& input) const;

    // 依据当前 predict_model_ 重建 Q_（每次 setPredictModel 或构造时调用）
    void rebuildProcessNoise();

    EskfProcessNoise process_noise_;   // 过程噪声参数
    State state_;                      // 名义状态 x
    StateCov cov_;                     // 协方差 P
    StateQ Q_;                         // 过程噪声矩阵 Q
    PredictModelType predict_model_ = PredictModelType::Inertial;

    bool init_state = false;           // （历史遗留，当前逻辑未使用）
};
}  // namespace legkilo
#endif  // LEG_KILO_ESKF_H
