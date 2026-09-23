// SPDX-License-Identifier: MIT
// @file KILO.h
// @brief KILO 前端 SLAM 编排器：把"传感器测量 → 两阶段 ESKF → 高斯体素地图更新"
//        整条流水线粘合到一起的顶层类。
// @author Ou Guangjun
// @created 2025-09-08
// @maintainer ouguangjun98@gmail.com
//
// -----------------------------------------------------------------------------
// 【模块地位】KILO 是前端的入口类，被 launch/接口层每帧调用一次 process(measure)。
//
// 【两阶段 ESKF 流水线】
//   Stage-1 (逐点/桶级增量 ESKF，负责运动去畸变)
//     ┌──────────────────────────────────────────────────────┐
//     │  按点云 curvature (=相对帧起始时间偏移) 排序           │
//     │  循环 (同一 curvature 桶 = 一次微时间步)：             │
//     │     1) 用 IMU / KinImu 做小步 predict + update        │
//     │     2) 把当前桶的点转换到"当前时刻的机体系→世界系"     │
//     │     3) 在高斯体素地图上找 P2P/NDT 邻域，构造 z, H, R   │
//     │     4) updateByPoints → 顺便完成去畸变                │
//     └──────────────────────────────────────────────────────┘
//
//   Stage-2 (整帧 IESKF，负责全局精化) —— 可通过 two_step_lidar_eskf 关闭
//     ┌──────────────────────────────────────────────────────┐
//     │  1) 把 Stage-1 得到的世界系点云"反投影"回机体系        │
//     │  2) 在最终状态附近迭代 IESKF：                        │
//     │     for iter in 0..N-1: 重构 P2P+NDT 残差 →           │
//     │        updateByCloud(...) 直到收敛                    │
//     └──────────────────────────────────────────────────────┘
//
//   最后 gaussian_voxel_map_->insertPoints(cloud_world) 更新地图。
//
// 【预测模型分派】(sensor_type_)
//   - LO   ：ConstantVelocity 预测模型 + 无 IMU/KinImu 通道，纯激光里程计
//   - LIO  ：Inertial 预测模型 + updateByImu
//   - KILO ：Inertial 预测模型 + updateByKinImu (足端运动学融合)
// -----------------------------------------------------------------------------
#ifndef LEG_KILO_CORE_SLAM_KILO_H_
#define LEG_KILO_CORE_SLAM_KILO_H_

#include <cstdint>
#include <deque>
#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "common/kernel.hpp"
#include "common/math_utils.hpp"
#include "common/pcl_types.h"
#include "common/sensor_types.hpp"
#include "interface/common/ros_compat.h"
namespace legkilo {
class ESKF;
class StateInitial;
class VoxelGrid;
class GaussianVoxelMap;
struct ObsShared;
template <int DIM> struct KNearestRes;
struct GaussPoint;
using GaussCloud = std::vector<GaussPoint, Eigen::aligned_allocator<GaussPoint>>;
}  // namespace legkilo

namespace legkilo {

// ============================================================
// 【结构体】ProcessResult
// 作用：一帧 process() 的返回结果，供 launch 层可视化 / 落盘。
// 关键字段：
//   - valid                : 本帧是否成功（首帧初始化会置 true，但数据不齐时置 false）
//   - total_pts_size       : 原始点云点数
//   - down_pts_size        : 体素降采样后的点数（Stage-1/2 都基于此）
//   - success_pts_size     : 有效 P2P + NDT 残差数量之和
//   - p2p_count / ndt_count: 最终参与状态更新的 P2P、NDT 有效点数
//   - cloud_world/body/lidar : 用于可视化 / 保存的三种坐标系下的降采样点云
//   - match_types          : 每个降采样点最终被划分为 P2P / NDT / Unused
// ============================================================
struct ProcessResult {
    bool valid = false;
    size_t total_pts_size = 0;
    size_t down_pts_size = 0;
    size_t success_pts_size = 0;  // 有效点数（P2P + NDT 块之和）
    size_t p2p_count = 0;         // 有效 P2P 点数
    size_t ndt_count = 0;         // 有效 NDT 观测块数
    size_t intensity_count = 0;   // additional scalar constraints; not added to success_pts_size
    CloudPtr cloud_world = nullptr;   // 世界系降采样点云
    CloudPtr cloud_body = nullptr;    // 机体系降采样点云
    CloudPtr cloud_lidar = nullptr;   // 激光雷达自身坐标系点云
    LidarMatchTypesPtr match_types = nullptr;

    friend std::ostream& operator<<(std::ostream& os, const ProcessResult& result);
};

// ============================================================
// 【类】KILO
// 作用：前端 SLAM 编排器。持有 ESKF、初始化器、降采样器、高斯体素地图；
//       封装两阶段 ESKF 流水线 + 每帧诊断。
// 关键成员：
//   - eskf_               : 24 维 ESKF/IESKF 滤波器
//   - state_initial_      : 首帧的重力对齐/静止初始化
//   - voxel_grid_         : 输入点云体素降采样（KILO 特有的采样策略）
//   - gaussian_voxel_map_ : 高斯体素地图（P2P / NDT 残差构造）
//   - sensor_type_        : LO / LIO / KILO，决定预测模型 & 观测通道
//   - two_step_lidar_eskf_: 是否启用 Stage-2 整帧 IESKF
//   - lo_predict_*        : LO 模式下的匀速外推输入缓存
// 使用方式：
//   auto kilo = std::make_unique<KILO>(config_yaml);
//   for (each frame) {
//       auto result = kilo->process(meas);
//       // 用 kilo->getRotLidar() / getPosLidar() 输出激光雷达位姿
//   }
// ============================================================
class KILO {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // 用配置文件初始化：会读取传感器类型、外参、噪声、体素参数等
    explicit KILO(const std::string& config_file);
    ~KILO();

    /**
     * @brief  处理一帧同步好的测量包（一段激光扫描 + 该段时间内的 IMU/KinImu）。
     * @param  measure 同步好的 MeasGroup（LiDAR 扫描 + IMU 队列 + KinImu 队列）
     * @return         ProcessResult：包含世界系/机体系降采样点云、点数统计等
     *
     * 【算法要点】
     *  - 首帧走 StateInitial（估重力、陀螺零偏、初始旋转），并把首帧点云直接插入地图
     *  - 之后：Stage-1 (逐点桶级 ESKF 去畸变) → Stage-2 (可选，整帧 IESKF 精化)
     *  - 最后 gaussian_voxel_map_->insertPoints(gauss_cloud_world) 更新地图
     */
    ProcessResult process(common::MeasGroup& measure);

    // 位姿查询接口（IMU 坐标系 = 机体系；Lidar 坐标系 = 通过外参左乘得到）
    Vec3D getPosImu() const;
    Mat3D getRotImu() const;
    Vec3D getPosLidar() const;
    Mat3D getRotLidar() const;

   private:
    // ============================================================
    // 【结构体】LidarUpdateDiagnostics
    // 作用：一次 Stage 内点云更新的诊断快照，用于线上打印和 CSV 落盘。
    //   - p2p/ndt_count       : 该次更新用到的有效点/块数
    //   - p2p/ndt_residuals   : 每个有效残差的绝对值（用于计算中位数/95 分位）
    //   - p2p/ndt_weight_sum  : Cauchy kernel 后的等效权重和
    //   - p2p/ndt_nonfinite_count : 出现 NaN/Inf 的次数（提前发现异常）
    //   - information         : 6×6 信息矩阵 Σ w · Jᵀ J，用于评估位姿可观测性
    //   - ieskf_iterations_used / ieskf_converged : Stage-2 迭代情况
    // merge() 会把多次 Stage-1 桶级更新合并为一整帧的统计。
    // ============================================================
    struct LidarUpdateDiagnostics {
        size_t p2p_count = 0;
        size_t ndt_count = 0;
        size_t intensity_count = 0;
        std::vector<double> intensity_residuals;
        double intensity_weight_sum = 0.0;
        Mat6D intensity_information = Mat6D::Zero();
        std::vector<double> p2p_residuals;
        std::vector<double> ndt_residuals;
        double p2p_weight_sum = 0.0;
        double ndt_weight_sum = 0.0;
        size_t p2p_nonfinite_count = 0;
        size_t ndt_nonfinite_count = 0;
        Mat6D information = Mat6D::Zero();  // Σ w · Jᵀ J（只覆盖位姿 6 维）
        size_t ieskf_iterations_used = 0;
        bool ieskf_converged = false;

        // 把 other 的统计并入自己（Stage-1 里桶级增量更新会多次调用）
        void merge(const LidarUpdateDiagnostics& other) {
            p2p_count += other.p2p_count;
            ndt_count += other.ndt_count;
            intensity_count += other.intensity_count;
            intensity_residuals.insert(intensity_residuals.end(), other.intensity_residuals.begin(),
                                       other.intensity_residuals.end());
            intensity_weight_sum += other.intensity_weight_sum;
            intensity_information += other.intensity_information;
            p2p_residuals.insert(p2p_residuals.end(), other.p2p_residuals.begin(), other.p2p_residuals.end());
            ndt_residuals.insert(ndt_residuals.end(), other.ndt_residuals.begin(), other.ndt_residuals.end());
            p2p_weight_sum += other.p2p_weight_sum;
            ndt_weight_sum += other.ndt_weight_sum;
            p2p_nonfinite_count += other.p2p_nonfinite_count;
            ndt_nonfinite_count += other.ndt_nonfinite_count;
            information += other.information;
        }
    };

    // 读取 YAML 并初始化所有子模块（ESKF、地图、降采样、外参、诊断开关等）
    void initializeFromYaml(const std::string& config_file);

    void appendIntensityObservations(const std::vector<KNearestRes<1>>& results, ObsShared& obs,
                                     LidarUpdateDiagnostics& diagnostics, bool collect_diagnostics) const;

    /**
     * @brief  首帧初始化时把降采样点直接注入地图（不做匹配，只贡献先验）。
     * @return 用于地图更新的世界系高斯点云
     */
    std::shared_ptr<GaussCloud> initializeMap(const CloudPtr& cloud_lidar);

    /**
     * @brief  把机体系下的点协方差 + 位姿协方差合并为世界系点协方差。
     *   cov_world = R · cov_body · Rᵀ
     *             + (R · [p_body]×) · rot_cov · (R · [p_body]×)ᵀ  ← 旋转不确定性
     *             + pos_cov                                        ← 位置不确定性
     * 第二/三项仅在 use_state_covariance_for_lidar_points_=true 时启用。
     */
    Mat3D computeWorldPointCov(const Vec3D& point_body, const Mat3D& cov_body, const Mat3D& rot, const Mat3D& rot_cov,
                               const Mat3D& pos_cov) const;

    /**
     * @brief  Stage-2 前的反投影：把 Stage-1 得到的世界系点云用**当前(最终) 位姿**
     *         打回机体系；同时按新的机体系坐标重算点测量噪声协方差。
     * 【物理意义】Stage-1 每个点被去畸变到自己的时刻，最终位姿更新完成后需要以
     * "最终一个统一位姿"重新提供机体系表达，供 Stage-2 迭代反复变换。
     */
    void backpropagate(const GaussCloud& cloud_world, GaussCloud& cloud_body) const;

    /**
     * @brief  基于 current_time 做一次 ESKF 预测：先推协方差（从上次 update 时刻起算 dt_cov），
     *         再推名义状态（从上次 predict 时刻起算 dt）。
     *
     * 【为何分开】协方差要覆盖整段自 update 以来的时间，而状态推进是从上次 predict 时刻
     * 起的增量步长；两者时间基准不同，因此调用两次 predict 分别控制 prop_state/prop_cov。
     */
    bool eskfPredict(double current_time);

    // IMU 到来时的 predict + updateByImu (Point-LIO 风格观测)
    bool predictUpdateImu(const ros_compat::ImuMsgPtr& imu);

    // KinImu (足端运动学 + IMU) 到来时的 predict + updateByKinImu
    bool predictUpdateKinImu(const common::KinImuMeas& kin_imu);

    /**
     * @brief  Stage-1 桶级点云更新：对 [idx_i, idx_j) 这一段"同一 curvature 时刻"的点
     *         做一次 predict → 构建 P2P/NDT 残差 → updateByPoints。
     *         这个函数是"运动畸变去除"发生的地方（每个点用自己时刻的位姿去展开）。
     */
    LidarUpdateDiagnostics predictUpdatePoint(const double current_time, const size_t idx_i, const size_t idx_j,
                                              const CloudPtr& cloud_lidar_pcl, GaussCloud& cloud_body,
                                              GaussCloud& cloud_world, LidarMatchTypes* match_types);

    /**
     * @brief  Stage-2 整帧 IESKF：在 backpropagate 之后，对整个 cloud_body 反复迭代
     *         重线性化 + updateByCloud，直到收敛或达到 ieskf_max_iterations_。
     */
    LidarUpdateDiagnostics predictUpdateCloud(const GaussCloud& cloud_body, GaussCloud& cloud_world,
                                              LidarMatchTypes* match_types);

    // 【LO 模式专用】重置 CV 预测输入缓存（首帧或长时间无有效更新后调用）
    void resetLoPredictInput(double current_time);

    // 【LO 模式专用】用最近两次校正后的位姿差分出 world_vel 和 gyr，供下一帧 CV 预测使用
    void updateLoPredictInput(double current_time);

   private:
    // --------- 子模块 ---------
    std::unique_ptr<ESKF> eskf_;                       // 24 维 ESKF/IESKF
    std::unique_ptr<StateInitial> state_initial_;      // 首帧初始化（重力对齐/静止零偏估计）
    std::unique_ptr<VoxelGrid> voxel_grid_;            // 输入点云降采样
    std::unique_ptr<GaussianVoxelMap> gaussian_voxel_map_;  // 高斯体素地图（P2P/NDT 匹配）

    // --------- ESKF / 观测噪声配置 ---------
    common::SensorType sensor_type_ = common::SensorType::LIO;  // 传感器组合类型
    bool two_step_lidar_eskf_ = true;                           // 是否启用 Stage-2 整帧 IESKF
    bool p2p_enable_ = true;                                    // 是否使用点到平面 (P2P) 残差
    bool ndt_enable_ = false;                                   // 是否使用 NDT (分布到分布) 残差
    bool intensity_enable_ = false;
    bool use_state_covariance_for_lidar_points_ = true;         // 世界系点协方差是否叠加位姿 P
    int ieskf_max_iterations_ = 3;                              // Stage-2 IESKF 最大迭代次数
    double gravity_ = 9.81;                                     // 世界系重力量级 (m/s^2)
    double acc_norm_ = 1.0;                                     // 初始化时估算的加速度模长（用来把 IMU 单位统一到 m/s^2）
    float range_noise_ = 0.1;                                   // 激光测距噪声（模型见 voxel_map_utils）
    float degree_noise_ = 0.04;                                 // 激光测角噪声（度）
    double imu_acc_meas_noise_ = 0.0;                           // IMU 加速度测量噪声
    double imu_gyr_meas_noise_ = 0.0;                           // IMU 角速度测量噪声
    double kin_meas_noise_ = 0.0;                               // 足端运动学速度观测噪声
    CauchyKernel p2plane_kernel_{1.0};                          // P2P 残差的 Cauchy 鲁棒核
    CauchyKernel ndt_kernel_{1.0};                              // NDT 残差的 Cauchy 鲁棒核
    double last_state_predict_time_ = 0.0;                      // 上一次"推名义状态"的时刻
    double last_state_update_time_ = 0.0;                       // 上一次"完成 update"的时刻
    bool init_flag_ = true;                                     // 是否还未完成首帧初始化

    // --------- LO 模式匀速外推缓存 ---------
    // 每帧结束后从"最近两次校正后位姿"差分得到 world_vel / gyr，供下一帧 CV 预测使用
    Vec3D lo_predict_world_vel_ = Vec3D::Zero();
    Vec3D lo_predict_gyr_ = Vec3D::Zero();
    Mat3D lo_last_corrected_rot_ = Mat3D::Identity();
    Vec3D lo_last_corrected_pos_ = Vec3D::Zero();
    double lo_last_corrected_time_ = 0.0;
    bool lo_has_corrected_state_ = false;

    // --------- 前端逐帧诊断 ---------
    // 用来快速定位"从哪一帧开始漂移"，可打表也可落 CSV
    bool frontend_diagnostic_print_ = true;
    bool frontend_diagnostic_csv_ = false;
    std::string frontend_diagnostic_csv_path_;
    std::ofstream frontend_diagnostic_csv_stream_;
    uint64_t diagnostic_frame_id_ = 0;
    bool diagnostic_has_previous_pose_ = false;
    Mat3D diagnostic_previous_rot_ = Mat3D::Identity();
    Vec3D diagnostic_previous_pos_ = Vec3D::Zero();

    // --------- 外参 & 降采样 ---------
    // ext: 从激光雷达坐标系到机体系 IMU 坐标系的外参 (p_body = ext_rot_ · p_lidar + ext_t_)
    Mat3D ext_rot_ = Mat3D::Identity();
    Vec3D ext_t_ = Vec3D::Zero();
};

}  // namespace legkilo

#endif  // LEG_KILO_CORE_SLAM_KILO_H_
