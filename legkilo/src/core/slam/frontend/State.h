// SPDX-License-Identifier: MIT
// @file State.h
// @brief 前端 ESKF 名义状态 (nominal state) 定义、24 维误差状态的 boxplus / boxminus 运算。
// @author Ou Guangjun
// @created 2025-09-21
// @maintainer ouguangjun98@gmail.com
//
// -----------------------------------------------------------------------------
// 【学习提示】误差状态卡尔曼滤波 (ESKF) 的两套变量：
//   1) 名义状态 nominal state  —— 由 `class State` 存放（旋转是 SO(3)，位置/速度是欧氏向量）；
//      每次预测/更新都直接积分/叠加到名义状态上，避免小量长期累计带来的数值退化。
//   2) 误差状态 error state    —— 24 维向量 `StateVec`，代表名义状态旁边的"局部切空间"扰动；
//      卡尔曼滤波器操作的对象就是这 24 维误差向量的均值 (总是保持为 0) 和协方差 `StateCov`。
//
// 24 维误差状态的实际排布（请以下方 operator+= 的 block 索引为准）：
//   [ 0: 3)  δθ    旋转在 so(3) 上的小量 (轴角，右乘约定：R = R * Exp(δθ))
//   [ 3: 6)  δp    位置误差   (m)
//   [ 6: 9)  δv    速度误差   (m/s)
//   [ 9:12)  δba   加速度计零偏误差 (m/s^2)
//   [12:15)  δbw   陀螺仪零偏误差   (rad/s)
//   [15:18)  δg    重力向量误差 (m/s^2，允许在线校正)
//   [18:21)  δa    IMU "输入加速度"状态误差 (Point-LIO 里当作被估计量而非外部输入)
//   [21:24)  δw    IMU "输入角速度"状态误差
// -----------------------------------------------------------------------------
#ifndef LEG_KILO_STATE_H
#define LEG_KILO_STATE_H

#include "common/math_utils.hpp"

namespace legkilo {

// 误差状态向量维数：24 = 3(旋转) + 3(位置) + 3(速度) + 3(加速度零偏) + 3(陀螺零偏) + 3(重力) + 3(IMU加速度) + 3(IMU角速度)
constexpr int DIM_STATE = 24;

// 24×1 列向量：一次 ESKF 更新算出的 Δx（δx = K·(z - h(x̂))）
using StateVec = Eigen::Matrix<double, DIM_STATE, 1>;

// 24×24 协方差矩阵 P
using StateCov = Eigen::Matrix<double, DIM_STATE, DIM_STATE>;

// 状态转移雅可比 F = ∂f/∂x（离散形式），常记为 Fx
using StateF = Eigen::Matrix<double, DIM_STATE, DIM_STATE>;

// 通用 24×24 矩阵别名（如 I、KH 中间量等）
using StateMat = Eigen::Matrix<double, DIM_STATE, DIM_STATE>;

// 过程噪声协方差矩阵 Q（离散化后与 dt^2 相乘参与协方差传播）
using StateQ = Eigen::Matrix<double, DIM_STATE, DIM_STATE>;

// ============================================================
// 【类】State
// 作用：存放前端 ESKF 的**名义状态** (nominal state)。名义状态本身没有协方差；
//       所有不确定性都存在滤波器的 P (StateCov) 里。
// 关键成员：
//   - rot_   : 世界系 → 机体系旋转矩阵 R_wb（右乘小量以更新，见 operator+=）
//   - pos_   : 机体系原点在世界系下的位置 p_w
//   - vel_   : 机体系原点在世界系下的速度 v_w
//   - ba_    : 加速度计零偏（机体系）
//   - bw_    : 陀螺零偏（机体系）
//   - grav_  : 重力向量（世界系，允许在线优化）
//   - imu_a_ : 当前时刻的"体系加速度"估计（Point-LIO 风格：把 IMU 原始量当作被估计变量）
//   - imu_w_ : 当前时刻的"体系角速度"估计
// 使用方式：
//   - 由 ESKF 类持有一份 `State state_`；
//   - 预测时：state_ += f(x, u)*dt   （operator+= 是 boxplus，注意旋转做 SO(3) 右乘）
//   - 更新时：state_ += K * (z - h)  （同样通过 boxplus 完成）
//   - 计算残差/迭代 ESKF (IESKF) 时：Δx_prior = prior_state - state_（operator-  即 boxminus）
// ============================================================
class State {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Mat3D rot_;    // 名义旋转 R_wb（世界 ← 机体）
    Vec3D pos_;    // 名义位置 p_w（世界系下机体原点）
    Vec3D vel_;    // 名义速度 v_w（世界系）
    Vec3D ba_;     // 加速度计零偏 (accelerometer bias)
    Vec3D bw_;     // 陀螺零偏     (gyroscope bias)
    Vec3D grav_;   // 世界系重力向量（一般初始化为 (0,0,-9.81)，允许 ESKF 在线微调）
    Vec3D imu_a_;  // 体系"当前加速度"状态量（Point-LIO 建模：不作为已知输入，而是被估计量）
    Vec3D imu_w_;  // 体系"当前角速度"状态量（同上）

    // 默认构造：把所有平移/速度/零偏清零，旋转为单位阵，重力设为 (0,0,-9.81)。
    State();

    /**
     * @brief  boxplus 运算：state ⊞ δx，把 24 维误差状态叠加到名义状态上。
     * @param  delta 24 维误差状态增量（由 predict/update 阶段算出）。
     *
     * 【关键点】
     *  - 旋转部分用右乘指数映射：R ← R * Exp(δθ)，其中 δθ = delta[0:3]。
     *  - 其它欧氏量直接向量加法：pos, vel, ba, bw, grav, imu_a, imu_w。
     *  - 完成之后误差状态被"吸收"进名义状态，δx 本身被重置为 0（本函数外部完成）。
     */
    void operator+=(const StateVec& delta);

    /**
     * @brief  boxminus 运算：this ⊟ other，把两个名义状态之间的差映射回 24 维切空间。
     * @param  other 参考名义状态（一般是 IESKF 迭代前的先验状态 x_k^{prior}）。
     * @return 24 维误差向量 δx，使得 other ⊞ δx ≈ this。
     *
     * 【关键点】
     *  - 旋转部分：ΔR = other.R^T * this.R，δθ = Log(ΔR) 对应 so(3)。
     *  - 其它欧氏量直接做减法。
     *  - 用于 IESKF 中把先验状态和迭代中的状态做差，构造带先验项的更新方程。
     */
    StateVec operator-(const State& other) const;
};

}  // namespace legkilo

#endif  // LEG_KILO_STATE_H
