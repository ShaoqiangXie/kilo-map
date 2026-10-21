#include "core/slam/frontend/State.h"

namespace legkilo {

// --------------------------------------------------------------------------
// 默认构造：给一个"零运动 + 标准重力"的先验，用于系统初始化前的占位。
// 真正的初值（尤其是重力方向）会在 StateInitial 阶段被覆盖。
// --------------------------------------------------------------------------
State::State() {
    rot_ = Mat3D::Identity();          // 旋转初始化为单位阵：机体系与世界系对齐
    pos_ = Vec3D::Zero();              // 位置在世界系原点
    vel_ = Vec3D::Zero();              // 静止起步
    ba_ = Vec3D::Zero();               // 加速度计零偏未知，先设 0
    bw_ = Vec3D::Zero();               // 陀螺零偏未知，先设 0
    grav_ = Vec3D(0.0, 0.0, -9.81);    // 默认重力沿世界 -Z 方向（NED-like 或 ENU 视配置而定）
    imu_a_ = Vec3D::Zero();            // 未观测到 IMU 时先给 0
    imu_w_ = Vec3D::Zero();
}

// --------------------------------------------------------------------------
// boxplus：state ⊞ δx。误差状态卡尔曼滤波在完成一次预测/更新后，
// 会把算出来的 24 维 δx 通过本函数"注入"名义状态。
//
// 【关键区别】旋转是流形量 (SO(3))，不能直接加；这里采用右乘约定：
//     R_new = R_old * Exp(δθ)
//     即 δθ 表示"当前机体系相对旧机体系的小旋转"。
// 其他 3D 欧氏向量则直接向量加法。
//
// 【注意】block 索引和 State.h 中【24 维排布】的注释一一对应，
// 修改任何一处都要同步。
// --------------------------------------------------------------------------
void State::operator+=(const StateVec& delta) {
    // 旋转： δθ = delta[0:3]，用罗德里格斯公式把 so(3) 映射回 SO(3) 再右乘
    this->rot_ = this->rot_ * Exp(delta(0, 0), delta(1, 0), delta(2, 0));
    this->pos_ += delta.block<3, 1>(3, 0);     // δp
    this->vel_ += delta.block<3, 1>(6, 0);     // δv
    this->ba_ += delta.block<3, 1>(9, 0);      // δba
    this->bw_ += delta.block<3, 1>(12, 0);     // δbw
    this->grav_ += delta.block<3, 1>(15, 0);   // δg（允许在线校准重力）
    this->imu_a_ += delta.block<3, 1>(18, 0);  // δa（Point-LIO 建模）
    this->imu_w_ += delta.block<3, 1>(21, 0);  // δw
}

// --------------------------------------------------------------------------
// boxminus：this ⊟ other，把两名义状态间的差落到 24 维切空间。
//
// 用途：IESKF (Iterated ESKF) 每次迭代需要引入"当前迭代状态 x_k"与"先验状态
// x_k^{prior}"之间的差，用于把先验重新参数化到当前工作点。
// 具体见 ESKF::updateByCloud 中的：
//     delta_prior = prior_state - state_;         //  ⇒  ⊟
//     delta_x = K*z + delta_prior - K*H*delta_prior.head(6);
//
// 旋转部分：ΔR = other.R^T * this.R  ⇒  δθ = Log(ΔR)  （从旧 → 新 的右扰动）
// 其他量：直接欧氏差。
// --------------------------------------------------------------------------
StateVec State::operator-(const State& other) const {
    StateVec delta;
    Mat3D rot_delta = other.rot_.transpose() * this->rot_;  // ΔR = R_other^T · R_this
    delta.block<3, 1>(0, 0) = Log(rot_delta);               // δθ ∈ so(3)
    delta.block<3, 1>(3, 0) = this->pos_ - other.pos_;
    delta.block<3, 1>(6, 0) = this->vel_ - other.vel_;
    delta.block<3, 1>(9, 0) = this->ba_ - other.ba_;
    delta.block<3, 1>(12, 0) = this->bw_ - other.bw_;
    delta.block<3, 1>(15, 0) = this->grav_ - other.grav_;
    delta.block<3, 1>(18, 0) = this->imu_a_ - other.imu_a_;
    delta.block<3, 1>(21, 0) = this->imu_w_ - other.imu_w_;
    return delta;
}

}  // namespace legkilo
