#include "core/slam/frontend/eskf.h"

namespace legkilo {

// --------------------------------------------------------------------------
// 构造：接收过程噪声参数，立即构建 Q_ 矩阵。
// 注意：cov_ 未在此处初始化 —— 由外部（KILO 首帧）以极小对角阵 (1e-6 * I) 显式赋值。
// --------------------------------------------------------------------------
ESKF::ESKF(const EskfProcessNoise& noise) : process_noise_(noise) { rebuildProcessNoise(); }

// 切换预测模型（Inertial ↔ CV），因为 Q 的填充位置不同，必须重建
void ESKF::setPredictModel(PredictModelType predict_model) {
    predict_model_ = predict_model;
    rebuildProcessNoise();
}

// --------------------------------------------------------------------------
// f(x,u)·dt 的分派入口。真正的模型见 Inertial / ConstantVelocity 两个具体实现。
// --------------------------------------------------------------------------
StateVec ESKF::getFunctionf(double dt, const PredictInput& input) const {
    switch (predict_model_) {
        case PredictModelType::Inertial: return getFunctionfInertial(dt, input);
        case PredictModelType::ConstantVelocity: return getFunctionfConstantVelocity(dt, input);
    }
    return StateVec::Zero();
}

// --------------------------------------------------------------------------
// Inertial（IMU 惯性）预测模型 f(x,u)·dt：
//   δθ_pred = ω · dt                (旋转积分：右乘 Exp(ω·dt) ⇒ boxplus 里 δθ = ω·dt)
//   δp_pred = v · dt                (匀速位置积分)
//   δv_pred = (R·a + g) · dt        (体系加速度旋到世界再补重力，再积分)
// 其余零偏 / imu_a / imu_w 建模为随机游走：期望值零变化，不出现在 f 里。
// 【注意】这里 `input.imu_acc` 是"体系加速度"，已经在上层（Point-LIO 用状态量替代）里
// 处理过 IMU 原始量与偏差之间的关系；本函数不再显式减 ba。
// --------------------------------------------------------------------------
StateVec ESKF::getFunctionfInertial(double dt, const PredictInput& input) const {
    StateVec vec = StateVec::Zero();
    vec.segment<3>(0) = dt * input.imu_gyr;                              // [步骤 1] δθ = ω·dt (对应 rot 块)
    vec.segment<3>(3) = dt * state_.vel_;                                // [步骤 2] δp = v·dt (匀速)
    vec.segment<3>(6) = dt * (state_.rot_ * input.imu_acc + state_.grav_); // [步骤 3] δv = (R·a + g)·dt
    return vec;
}

// --------------------------------------------------------------------------
// ConstantVelocity（LO 模式）预测模型：没有 IMU，只用上一帧估计的 world_vel / gyr。
//   δθ_pred = ω · dt
//   δp_pred = world_vel · dt        (直接在世界系匀速外推)
// 不再传播速度/重力/零偏/imu_* —— 全部维持不变（其协方差通过 Q 的对应块增长）。
// --------------------------------------------------------------------------
StateVec ESKF::getFunctionfConstantVelocity(double dt, const PredictInput& input) const {
    StateVec vec = StateVec::Zero();
    vec.segment<3>(0) = dt * input.imu_gyr;      // δθ = ω·dt
    vec.segment<3>(3) = dt * input.world_vel;    // δp = v_world·dt
    return vec;
}

// --------------------------------------------------------------------------
// Fx 分派入口。Fx 是"离散状态转移雅可比"（本代码中 Fx 已把 I 打包在里面）。
// --------------------------------------------------------------------------
StateF ESKF::getFx(double dt, const PredictInput& input) const {
    switch (predict_model_) {
        case PredictModelType::Inertial: return getFxInertial(dt, input);
        case PredictModelType::ConstantVelocity: return getFxConstantVelocity(dt, input);
    }
    return StateF::Identity();
}

// --------------------------------------------------------------------------
// Inertial 模型的 Fx（24×24）。以下矩阵推导来自误差状态动力学：
//   δθ̇  = -[ω]×·δθ  +  I·δω              → 离散化后见 Fx[0:3,0:3] 与 Fx[0:3, 21:24]
//   δṗ  = δv                              → Fx[3:6, 6:9] = dt·I
//   δv̇  = -R·[a]×·δθ + δg + R·δa           → Fx[6:9, 0:3], [6:9,15:18], [6:9,18:21]
// 其余状态（ba, bw, g, imu_a, imu_w）为随机游走：Fx 对角块保持 I（Fx.Identity 里已有）。
//
// 【非平凡块】
//   Fx[0:3, 0:3]   = Exp(-ω·dt)   —— δθ 自身在 ω 作用下的旋转（右乘约定下的一阶）
//   Fx[0:3, 21:24] = dt · I        —— δθ 对 imu_w 状态误差的直接吸收
//   Fx[3:6, 6:9]   = dt · I        —— 位置对速度
//   Fx[6:9, 0:3]   = -dt · R · [a]× —— 速度对 δθ（旋转扰动会把 R·a 投到不同方向）
//   Fx[6:9, 15:18] = dt · I        —— 速度对 g（重力误差直接进速度）
//   Fx[6:9, 18:21] = dt · R        —— 速度对 imu_a 状态误差（把体系加速度旋到世界）
// --------------------------------------------------------------------------
StateF ESKF::getFxInertial(double dt, const PredictInput& input) const {
    StateF Fx = StateF::Identity();
    // [块 (0,0)]  δθ_{k+1} 相对 δθ_k 的转移：Exp(-ω·dt)（右乘约定下的一阶展开）
    Fx.block<3, 3>(0, 0) = Exp(Eigen::Matrix<double, 3, 1>(-dt * input.imu_gyr));
    // [块 (0,21)] δθ 对 imu_w 状态误差的敏感度：δω 直接积分成 δθ
    Fx.block<3, 3>(0, 21) = dt * Mat3D::Identity();
    // [块 (3,6)]  δp 对 δv：位置 = 速度 * dt
    Fx.block<3, 3>(3, 6) = dt * Mat3D::Identity();
    // [块 (6,0)]  δv 对 δθ：旋转扰动引起的比例加速度改变 = -R·[a]×·δθ
    Fx.block<3, 3>(6, 0) = (-dt) * state_.rot_ * SKEW_SYM_MATRIX(input.imu_acc);
    // [块 (6,15)] δv 对 δg：重力误差直接积到速度
    Fx.block<3, 3>(6, 15) = dt * Mat3D::Identity();
    // [块 (6,18)] δv 对 imu_a 状态误差：体系加速度扰动经过 R 旋到世界系
    Fx.block<3, 3>(6, 18) = dt * state_.rot_;
    return Fx;
}

// --------------------------------------------------------------------------
// ConstantVelocity 模型的 Fx（24×24）：只有旋转部分有非平凡转移。
// 位置 = world_vel·dt，但 world_vel 是外部输入 (input.world_vel) 而非状态，
// 因此 δp 与状态之间没有雅可比耦合（Fx[3:6,*] 保持 I 与 0）。
// --------------------------------------------------------------------------
StateF ESKF::getFxConstantVelocity(double dt, const PredictInput& input) const {
    StateF Fx = StateF::Identity();
    Fx.block<3, 3>(0, 0) = Exp(Eigen::Matrix<double, 3, 1>(-dt * input.imu_gyr));
    return Fx;
}

// --------------------------------------------------------------------------
// 根据当前预测模型构建 Q_ 矩阵。
//
// 【Inertial 模式下 Q 的对角布局】（其余块为 0）
//   Q[6:9, 6:9]     ← vel_process_cov       (速度过程噪声)
//   Q[9:12, 9:12]   ← acc_bias_process_cov  (ba 随机游走)
//   Q[12:15,12:15]  ← gyr_bias_process_cov  (bw 随机游走)
//   Q[18:21,18:21]  ← imu_acc_process_cov   (imu_a 状态白噪声)
//   Q[21:24,21:24]  ← imu_gyr_process_cov   (imu_w 状态白噪声)
//
// 【ConstantVelocity 模式下 Q 的对角布局】（LO 无 IMU）
//   Q[0:3, 0:3]  ← imu_gyr_process_cov  (直接充当 δθ 的过程噪声)
//   Q[3:6, 3:6]  ← vel_process_cov      (world_vel 输入不确定性打到位置块)
// --------------------------------------------------------------------------
void ESKF::rebuildProcessNoise() {
    Q_.setZero();

    switch (predict_model_) {
        case PredictModelType::Inertial:
            Q_.block<3, 3>(6, 6) = process_noise_.vel_process_cov * Mat3D::Identity();
            Q_.block<3, 3>(9, 9) = process_noise_.acc_bias_process_cov * Mat3D::Identity();
            Q_.block<3, 3>(12, 12) = process_noise_.gyr_bias_process_cov * Mat3D::Identity();
            Q_.block<3, 3>(18, 18) = process_noise_.imu_acc_process_cov * Mat3D::Identity();
            Q_.block<3, 3>(21, 21) = process_noise_.imu_gyr_process_cov * Mat3D::Identity();
            break;
        case PredictModelType::ConstantVelocity:
            Q_.block<3, 3>(0, 0) = process_noise_.imu_gyr_process_cov * Mat3D::Identity();
            Q_.block<3, 3>(3, 3) = process_noise_.vel_process_cov * Mat3D::Identity();
            break;
    }
}

// --------------------------------------------------------------------------
// 一步预测：可独立选择"推名义状态"和/或"推协方差"。
//   - 名义状态：state_ += f(x,u)·dt              (直接 boxplus)
//   - 协方差 ：  P ← Fx · P · Fxᵀ  +  dt² · Q    (标准 ESKF 协方差传播)
//
// 【为什么 dt² 而不是 dt】
//   这里的 Q 已经作为"参数化的过程噪声强度"传入 (类似离散化后 σ² 的意义)；
//   代码选择在预测处再乘 dt²，等价于把连续谱密度 → 离散协方差的经验缩放。
//   若上层 Q 是连续谱密度 (units^2 / s)，正确离散化应为 dt·Q；此处按 dt²·Q 处理，
//   意味着传入的 vel/acc/gyr_*_process_cov 参数被视作离散步长归一化后的方差量。
// --------------------------------------------------------------------------
void ESKF::predict(double dt, const PredictInput& input, bool prop_state, bool prop_cov) {
    if (prop_state) { state_ += getFunctionf(dt, input); }   // [步骤 1] 名义状态推进
    if (prop_cov) {
        StateF Fx = getFx(dt, input);                        // [步骤 2] 拿到状态转移雅可比
        cov_ = Fx * cov_ * Fx.transpose() + (dt * dt) * Q_;  // [步骤 3] P 传播
    }
}

// --------------------------------------------------------------------------
// 用点云残差做一次卡尔曼更新（非迭代版本，供 Stage-1 逐点/桶更新使用）。
//
// H 的结构：每行观测只与 rot(0:3) 和 pos(3:6) 有关（6 列），其他列为 0。
// 因此协方差里只有 `cov_.block<DIM_STATE, 6>(0, 0)` 和 `cov_.block<6, DIM_STATE>(0, 0)`
// 是必需的子块，其他列相乘都被 0 消掉，无需显式扩充成 DIM_STATE 列。
//
// 三种数值路径（按观测维度 M 分派，重点是"避开 M×M 大矩阵求逆"）：
//   [Case A]  M = 1     ：所有量化为标量，无求逆，最快；加了 1e-6 防除零。
//   [Case B]  1 < M ≤ 24：小 M×M 求逆，普通 Kalman 形式。
//   [Case C]  M > 24    ：走"信息形式"—— (P⁻¹ + Hᵀ R⁻¹ H)⁻¹，避开 M×M 求逆，只做 24×24 求逆两次。
//
// 【协方差更新】使用简化的 P ← P - K·H·P（而非 Joseph 型），依赖 K 精确解使得
// 数值上仍能保持对称正定；因为 H 只有 6 列，KH 只影响 6 列。
// --------------------------------------------------------------------------
void ESKF::updateByPoints(const ObsShared& obs_shared) {
    const Eigen::Matrix<double, Eigen::Dynamic, 1>& z = obs_shared.pt_z;
    const Eigen::Matrix<double, Eigen::Dynamic, 6>& h = obs_shared.pt_h;
    const Eigen::Matrix<double, Eigen::Dynamic, 1>& r = obs_shared.pt_R;

    int dof_measurements = static_cast<int>(h.rows());

    if (dof_measurements == 1) {
        // ---- Case A: 标量更新（Stage-1 单点/单 IMU 增量最常走这条路径） ----
        // K = P·Hᵀ · (H·P·Hᵀ + R)⁻¹，这里 H·P·Hᵀ + R 是标量
        Eigen::Matrix<double, DIM_STATE, 1> PHT = cov_.block<DIM_STATE, 6>(0, 0) * h.transpose();  // (24,1)
        double HPHT_R_inv = 1 / (1e-6 + (h * PHT.topRows(6))(0) + r(0));                           // 标量 (加 1e-6 防 0)
        Eigen::Matrix<double, DIM_STATE, 1> K = HPHT_R_inv * PHT;                                  // K = P Hᵀ / s
        StateVec delta_x = K * z;                                                                  // δx = K·z
        state_ += delta_x;                                                                         // boxplus
        cov_ = cov_ - K * h * cov_.block<6, DIM_STATE>(0, 0);                                      // P ← P - K H P（只 6 列）
    } else if (dof_measurements <= DIM_STATE) {
        // ---- Case B: 小 M×M 求逆（M ≤ 24）----
        Eigen::MatrixXd PHT = cov_.block<DIM_STATE, 6>(0, 0) * h.transpose();  // (24, M)
        Eigen::MatrixXd HPHT_R = h * PHT.topRows(6);                           // (M, M)
        HPHT_R.diagonal() += r;                                                // + R (只在对角)
        Eigen::MatrixXd K = PHT * HPHT_R.inverse();                            // (24, M)
        StateVec delta_x = K * z;
        state_ += delta_x;
        cov_ = cov_ - K * h * cov_.block<6, DIM_STATE>(0, 0);
    } else {
        // ---- Case C: 大观测维度（M > 24），走信息形式避开 M×M 求逆 ----
        // K = (P⁻¹ + Hᵀ R⁻¹ H)⁻¹ · Hᵀ R⁻¹
        Eigen::MatrixXd H_T_R_inv = h.transpose() * r.cwiseInverse().asDiagonal();  // (6, M)
        Eigen::MatrixXd P_temp = cov_.inverse();                                     // 一次 24×24 求逆
        P_temp.block<6, 6>(0, 0) += H_T_R_inv * h;                                   // 只有 6×6 子块被 Hᵀ R⁻¹ H 修正
        Eigen::MatrixXd K = P_temp.inverse().block<DIM_STATE, 6>(0, 0) * H_T_R_inv;  // 再一次 24×24 求逆
        StateVec delta_x = K * z;
        state_ += delta_x;
        cov_ = cov_ - K * h * cov_.block<6, DIM_STATE>(0, 0);
    }
}

// --------------------------------------------------------------------------
// IESKF (Iterated ESKF) 整帧更新：Stage-2 里对整帧点云的 P2P+NDT 观测进行迭代重线性化。
//
// 【核心公式】(H 只有 6 列，同样走信息形式避开大矩阵求逆)
//   K = (P⁻¹ + Hᵀ R⁻¹ H)⁻¹ · Hᵀ R⁻¹                     (24 × M)
//   第 0 次迭代：δx = K · z                                (等价 EKF)
//   之后每次： δx = K·z + Δprior - K·H·Δprior_pose        (IESKF 一次线性化步)
//     其中 Δprior = prior_state ⊟ state_                  (24 维 boxminus)
//     并且 Δprior_pose = Δprior[0:6]，因为 H 只作用在 rot+pos 上，K·H·Δprior 只依赖前 6 维
//
// 【收敛判据】
//   δθ 换算成度：dtheta.norm() * 57.3 < 0.1  （<0.1°）
//   δp 换算成 cm：dpos.norm()   * 100 < 0.1  （<0.1cm，即 <1e-3 m）
//   57.3 ≈ 180/π（弧度→度），100 是 m→cm。
//
// 【协方差策略】
//   - 只有在收敛或达到最大迭代次数时才更新 P（P ← P - K H P）；
//   - 中间迭代不更新 P，避免多次 K H P 造成 P 过度收缩。
// --------------------------------------------------------------------------
bool ESKF::updateByCloud(const ObsShared& obs_shared, const State& prior_state, int iter_max, int iter_cur) {
    const Eigen::Matrix<double, Eigen::Dynamic, 1>& z = obs_shared.pt_z;
    const Eigen::Matrix<double, Eigen::Dynamic, 6>& h = obs_shared.pt_h;
    const Eigen::Matrix<double, Eigen::Dynamic, 1>& r = obs_shared.pt_R;

    // [步骤 1] 计算 Hᵀ R⁻¹（R 是对角阵，直接用 cwiseInverse 后 asDiagonal 加速）
    Eigen::MatrixXd H_T_R_inv = h.transpose() * r.cwiseInverse().asDiagonal();  // (6, M)
    // [步骤 2] 信息形式的 K：K = (P⁻¹ + Hᵀ R⁻¹ H)⁻¹ · Hᵀ R⁻¹，避开 M×M 大矩阵求逆
    Eigen::MatrixXd P_temp = cov_.inverse();                                     // 一次 24×24 求逆
    P_temp.block<6, 6>(0, 0) += H_T_R_inv * h;                                   // 只有 6×6 子块受 H 影响
    Eigen::MatrixXd K = P_temp.inverse().block<DIM_STATE, 6>(0, 0) * H_T_R_inv;  // K: (24, M)

    // [步骤 3] δx 的构造：EKF 首次 vs IESKF 后续（带先验校正项）
    StateVec delta_x;
    if (iter_cur == 0) {
        delta_x = K * z;                                    // 首次迭代等价 EKF
    } else {
        StateVec delta_prior = prior_state - state_;        // Δprior = x^{prior} ⊟ x_current
        // δx = K·z + Δprior - K·H·Δprior_pose
        //  第 3 项之所以只取 delta_prior.segment<6>(0)，因为 H 只有 6 列 → K·H 只对应前 6 维
        delta_x = K * z + delta_prior - K * h * delta_prior.segment<6>(0);
    }
    // [步骤 4] 应用 boxplus
    state_ += delta_x;

    // [步骤 5] 收敛检测（rot 换算度、pos 换算 cm）
    Vec3D dtheta = delta_x.segment<3>(0);
    Vec3D dpos = delta_x.segment<3>(3);
    bool flag_converge = false;
    if ((dtheta.norm() * 57.3 < 0.1) && (dpos.norm() * 100 < 0.1)) { flag_converge = true; }  // 阈值：<0.1° 且 <0.1cm
    // [步骤 6] 仅在收敛或最后一次迭代才更新 P
    if (flag_converge || (iter_cur >= iter_max - 1)) { cov_ = cov_ - K * h * cov_.block<6, DIM_STATE>(0, 0); }
    return flag_converge;
}

// --------------------------------------------------------------------------
// Point-LIO 风格的 IMU 观测更新。
//
// 【观测模型】(在 KILO::predictUpdateImu 里构造 z 与 R)
//   z_acc = (g/|a|)·acc_meas - state.imu_a_ - state.ba_    (体系加速度残差)
//   z_gyr = gyr_meas        - state.imu_w_ - state.bw_    (体系角速度残差)
//   ⇒ H = [ 0_6×3, 0_6×3, 0_6×3, I_6, 0_6×3, 0_6×3, I_6, 0_6×3 ]
//        (对应 ba/bw 列 (9:15) 与 imu_a/imu_w 列 (18:24))
//
// 因为 H 只在两组 6×6 子块处非零，Kalman 各矩阵可以手工拆开：
//   PHT = P[:, 9:15] + P[:, 18:24]        (24 × 6)
//   HP  = P[9:15, :] + P[18:24, :]        (6 × 24)
//   HPHT = PHT[9:15, :] + PHT[18:24, :]   (6 × 6)
//     ↑ 等价 (P[9:15,9:15] + P[9:15,18:24] + P[18:24,9:15] + P[18:24,18:24])
//   R 加到 HPHT 的对角线上
//   K = PHT · HPHT⁻¹                      (24 × 6)
//   P ← P - K · HP                        (标准协方差收缩)
//
// 这一手工展开由 MATLAB 化简得到（注释 "simplify by matlab"），比通用 H·P·Hᵀ 快很多。
// --------------------------------------------------------------------------
void ESKF::updateByImu(const ObsShared& obs_shared) {
    // simplify by matlab
    // 【等价说明】ki_h 的稀疏结构：ki_h.block<6,6>(0, 9) = I, ki_h.block<6,6>(0, 18) = I
    Eigen::Matrix<double, DIM_STATE, 6> PHT = cov_.block<DIM_STATE, 6>(0, 9) + cov_.block<DIM_STATE, 6>(0, 18);
    Eigen::Matrix<double, 6, DIM_STATE> HP = cov_.block<6, DIM_STATE>(9, 0) + cov_.block<6, DIM_STATE>(18, 0);
    Eigen::Matrix<double, 6, 6> HPHT = PHT.block<6, 6>(9, 0) + PHT.block<6, 6>(18, 0);
    HPHT.diagonal() += obs_shared.ki_R;                            // + R（对角）
    Eigen::Matrix<double, DIM_STATE, 6> K = PHT * HPHT.inverse();  // 6×6 求逆即可
    StateVec delta_x = K * obs_shared.ki_z;
    state_ += delta_x;
    cov_ -= K * HP;                                                // P ← P - K·H·P
}

// --------------------------------------------------------------------------
// KILO 模式：IMU + 足端运动学联合观测（H 是稠密 (6+3·contact) × 24 矩阵）。
// 因为稀疏结构复杂，这里直接走通用稠密 Kalman。
//
//   K = P·Hᵀ · (H·P·Hᵀ + R)⁻¹
//   δx = K·z
//   P ← P - K·H·P
// --------------------------------------------------------------------------
void ESKF::updateByKinImu(const ObsShared& obs_shared) {
    Eigen::MatrixXd PHT = cov_ * obs_shared.ki_h.transpose();       // (24, M)
    Eigen::MatrixXd HPHT = obs_shared.ki_h * PHT;                   // (M, M)
    HPHT.diagonal() += obs_shared.ki_R;                             // + R
    Eigen::MatrixXd K = PHT * HPHT.inverse();                       // (24, M)
    StateVec delta_x = K * obs_shared.ki_z;
    state_ += delta_x;
    cov_ = cov_ - K * obs_shared.ki_h * cov_;                       // P ← P - K H P
}
}  // namespace legkilo
