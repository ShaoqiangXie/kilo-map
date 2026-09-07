#include "preprocess/kinematics.h"

namespace legkilo {

void Kinematics::processing(const ros_compat::HighStateMsg& high_state, common::KinImuMeas& kin_imu_meas) {
    // ---- 1) 时间戳：把 ROS 时间戳统一转成 double 秒（ros_compat 屏蔽 ROS1/ROS2 差异）
    kin_imu_meas.time_stamp_ = ros_compat::toSec(high_state.stamp);

    // ---- 2) IMU：直接把机身 IMU 的加速度与角速度按分量复制过去
    //           accelerometer 单位 m/s^2，gyroscope 单位 rad/s（与后续 ESKF 假设一致）
    for (int i = 0; i < 3; ++i) {
        kin_imu_meas.acc_[i] = high_state.imu.accelerometer[i];
        kin_imu_meas.gyr_[i] = high_state.imu.gyroscope[i];
    }

    /*  4 legs, 3 moters
        this project leg order: FR FL RR RL
        unitree leg order: FL FR RL RR
    */
    // ---- 3) 足端力 + 关节状态：Unitree 与本工程的腿序不一致，这里手动做映射
    //         Unitree:      FL(0)  FR(1)  RL(2)  RR(3)
    //         本工程:        FR(0)  FL(1)  RR(2)  RL(3)
    //         → 本工程[0]=Unitree[1]，本工程[1]=Unitree[0]，本工程[2]=Unitree[3]，本工程[3]=Unitree[2]
    const auto& foot_force = ros_compat::footForces(high_state);   // 4 只脚的法向力（Unitree 腿序）
    const auto& motor_state = ros_compat::motorStates(high_state); // 12 个电机状态（Unitree 腿序）
    // 每条腿的接触状态由带滞回的 ContactDetector 判定（详见 h 文件说明）
    kin_imu_meas.contact_[0] = contacts_[0].update(foot_force[1]); // 本工程 FR ← Unitree FR
    kin_imu_meas.contact_[1] = contacts_[1].update(foot_force[0]); // 本工程 FL ← Unitree FL
    kin_imu_meas.contact_[2] = contacts_[2].update(foot_force[3]); // 本工程 RR ← Unitree RR
    kin_imu_meas.contact_[3] = contacts_[3].update(foot_force[2]); // 本工程 RL ← Unitree RL

    // ---- 4) 关节角 & 角速度：每条腿有 3 个电机(髋外展/大腿俯仰/小腿俯仰)，
    //         Unitree motor_state 是长度 12 的一维数组，按 (腿×3 + 关节) 排列。
    //         这里把它重排成本工程腿序下的 [4][3] 数组。
    double foot_angle[4][3];      // (rad)
    double foot_angle_vel[4][3];  // (rad/s)
    for (int i = 0; i < 3; ++i) {
        foot_angle[0][i] = motor_state[3 + i].q;      // 本工程 FR ← Unitree FR (下标 3~5)
        foot_angle_vel[0][i] = motor_state[3 + i].dq;
        foot_angle[1][i] = motor_state[0 + i].q;      // 本工程 FL ← Unitree FL (下标 0~2)
        foot_angle_vel[1][i] = motor_state[0 + i].dq;
        foot_angle[2][i] = motor_state[9 + i].q;      // 本工程 RR ← Unitree RR (下标 9~11)
        foot_angle_vel[2][i] = motor_state[9 + i].dq;
        foot_angle[3][i] = motor_state[6 + i].q;      // 本工程 RL ← Unitree RL (下标 6~8)
        foot_angle_vel[3][i] = motor_state[6 + i].dq;
    }

    // ---- 5) 正向运动学 + 雅可比：算出足端相对机体系的 pos/vel（写入 kin_imu_meas）
    this->caculateFootPosVel(foot_angle, foot_angle_vel, kin_imu_meas.foot_pos_, kin_imu_meas.foot_vel_);

    // static int counts = 0;
    // int index = 1;
    // int uni_index = 0;
    // if(!(counts++  % 50)){
    //     std::cout <<  "contact: " <<kin_imu_meas.contact_[index] << std::endl;
    //     std::cout << "force: " << high_state.footForce[uni_index] << std::endl;
    //     std::cout << "my pos: " << kin_imu_meas.foot_pos_[index][0] << " " << kin_imu_meas.foot_pos_[index][1] << " "
    //     << kin_imu_meas.foot_pos_[index][2] << std::endl; std::cout << "uni pos: " <<
    //     high_state.footPosition2Body[uni_index].x << " " << high_state.footPosition2Body[uni_index].y << " "<<
    //     high_state.footPosition2Body[uni_index].z <<  std::endl; std::cout << "my vel: " <<
    //     kin_imu_meas.foot_vel_[index][0] << " " << kin_imu_meas.foot_vel_[index][1] << " " <<
    //     kin_imu_meas.foot_vel_[index][2] << std::endl; std::cout << "uni vel: " <<
    //     high_state.footSpeed2Body[uni_index].x << " " << high_state.footSpeed2Body[uni_index].y << " " <<
    //     high_state.footSpeed2Body[uni_index].z << std::endl << std::endl;
    // }
}

void Kinematics::caculateFootPosVel(const double (&foot_angle)[4][3], const double (&foot_angle_vel)[4][3],
                                    double (&foot_pos)[4][3], double (&foot_vel)[4][3]) {
    // 遍历四条腿：本工程腿序 FR(0) FL(1) RR(2) RL(3)
    for (int i = 0; i < 4; ++i) {
        // 通过 ±1 符号处理"前/后腿"和"左/右腿"的镜像：
        //   ffoot = +1 → 前腿（FR/FL），髋部 X 偏置指向前；ffoot = -1 → 后腿
        //   lfoot = +1 → 右腿（FR/RR），髋部 Y 偏置指向右；lfoot = -1 → 左腿
        int lfoot = -1, ffoot = -1;
        if (i < 2) ffoot = 1;              // FR / FL 都是前腿
        if (i == 0 || i == 2) lfoot = 1;   // FR / RR 都是右腿

        // 关节角三角函数缩写：
        //   角标 0 = 髋外展 (hip abduction, q1)
        //   角标 1 = 大腿俯仰 (thigh pitch, q2)
        //   角标 2 = 小腿俯仰 (calf pitch,  q3)
        // 组合角 (2+3) 描述小腿相对机体的绝对俯仰
        double s1 = sin(foot_angle[i][0]);
        double s2 = sin(foot_angle[i][1]);
        double s23 = sin(foot_angle[i][1] + foot_angle[i][2]);

        double c1 = cos(foot_angle[i][0]);
        double c2 = cos(foot_angle[i][1]);
        double c23 = cos(foot_angle[i][1] + foot_angle[i][2]);

        // ------ 正向运动学：3 关节串联的解析式（机体系 → 足端） ------
        // X（前后）：只被大腿+小腿俯仰影响 + 前后腿的髋部 X 偏置
        foot_pos[i][0] = -lt_ * s2 - lc_ * s23 + ffoot * ox_;
        // Y（左右）：由髋外展 q1 决定，加上左右腿的髋部 Y 偏置
        foot_pos[i][1] = lfoot * d_ * c1 + lc_ * s1 * c23 + lt_ * c2 * s1 + lfoot * oy_;
        // Z（竖直，向下为负）：髋外展 + 大腿/小腿俯仰共同决定
        foot_pos[i][2] = lfoot * d_ * s1 - lc_ * c1 * c23 - lt_ * c1 * c2;

        // ------ 雅可比矩阵 J(q)：足端速度 = J · [dq1, dq2, dq3]^T ------
        // 手动展开 3×3 矩阵，避免动态分配；行 = xyz，列 = 关节 1/2/3
        double jacb[3][3];
        jacb[0][0] = 0.0;                                   // X 不受髋外展 q1 影响
        jacb[0][1] = -lc_ * c23 - lt_ * c2;                 // X 对 q2
        jacb[0][2] = -lc_ * c23;                            // X 对 q3
        jacb[1][0] = lt_ * c1 * c2 - lfoot * d_ * s1 + lc_ * c1 * c23;  // Y 对 q1
        jacb[1][1] = -s1 * (lc_ * s23 + lt_ * s2);          // Y 对 q2
        jacb[1][2] = -lc_ * s23 * s1;                       // Y 对 q3
        jacb[2][0] = lt_ * c2 * s1 + lfoot * d_ * c1 + lc_ * s1 * c23;  // Z 对 q1
        jacb[2][1] = c1 * (lc_ * s23 + lt_ * s2);           // Z 对 q2
        jacb[2][2] = lc_ * s23 * c1;                        // Z 对 q3

        // ------ 速度：v = J · dq（注意 X 分量只用到第 2、3 列） ------
        foot_vel[i][0] = jacb[0][1] * foot_angle_vel[i][1] + jacb[0][2] * foot_angle_vel[i][2];
        foot_vel[i][1] =
            jacb[1][0] * foot_angle_vel[i][0] + jacb[1][1] * foot_angle_vel[i][1] + jacb[1][2] * foot_angle_vel[i][2];
        foot_vel[i][2] =
            jacb[2][0] * foot_angle_vel[i][0] + jacb[2][1] * foot_angle_vel[i][1] + jacb[2][2] * foot_angle_vel[i][2];
    }
    // 说明：此处求得的 foot_vel 是"足端相对机体系的速度"。
    //       在 KILO / ESKF 里，如果该足处于接触（contact_[i] = true），
    //       就近似认为足端相对世界系静止 → 机体速度 = -R_wb·foot_vel_i - ω × p_i
    //       从而得到腿式速度观测（配合 IMU 完成融合）。
}

}  // namespace legkilo
