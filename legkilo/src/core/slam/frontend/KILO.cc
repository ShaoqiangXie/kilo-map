#include "core/slam/frontend/KILO.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>

#include <colorful_terminal.hpp>
#include <tbb/blocked_range.h>
#include <tbb/concurrent_vector.h>
#include <tbb/parallel_for.h>

#include "common/file_io.hpp"
#include "common/glog_utils.hpp"
#include "common/math_utils.hpp"
#include "common/timer_utils.hpp"
#include "common/voxel_grid.hpp"
#include "common/yaml_helper.hpp"
#include "core/slam/frontend/eskf.h"
#include "core/slam/frontend/gaussian_voxel_map.h"
#include "preprocess/state_initial.hpp"

namespace legkilo {

namespace {
// 按点云的 curvature 字段（在本工程里被复用为"相对帧起始时刻的时间偏移"）升序排序，
// 这是 Stage-1 逐点/桶级 ESKF 能"边预测边观测"的关键前提。
inline bool time_list(PointType& x, PointType& y) { return (x.curvature < y.curvature); }

// 把鲁棒核输出的权重 w ∈ (0, 1] 换算为"方差放大系数" σ²_scale = 1/w：
//   Cauchy w = 1/(1 + r²/δ²)   ⇒   1/w = 1 + r²/δ²
// 意义：残差越大 → w 越小 → 方差放大 → 该点在 R 中权重变低。
// 加 kMinWeight=1e-6 兜底，避免除 0。
inline double robustVarianceScale(double weight) {
    static constexpr double kMinWeight = 1e-6;
    return 1.0 / std::max(weight, kMinWeight);
}

// 【两阶段的观测噪声档位】
//   ResidualNoise 字段顺序 (见 gaussian_voxel_map.h)：
//     { p2plane_meas_ratio, p2plane_min_noise, ndt_meas_ratio, ndt_min_noise }
//   Stage-1 (First)  ：p2p_min_noise=1e-2，稍松，以便先"粗匹配"完成去畸变
//   Stage-2 (Second) ：p2p_min_noise=1e-3，更严，以便在 IESKF 中精化
constexpr GaussianVoxelMap::ResidualNoise kFirstStepLidarNoise{1.0, 1e-2, 10.0, 1e-2};
constexpr GaussianVoxelMap::ResidualNoise kSecondStepLidarNoise{1.0, 1e-3, 10.0, 1e-2};

struct FrameDiagnosticStats {
    uint64_t frame_id = 0;
    double lidar_begin_time = 0.0;
    double lidar_end_time = 0.0;
    size_t imu_input_num = 0;
    size_t imu_remaining_num = 0;
    size_t raw_points = 0;
    size_t down_points = 0;
    size_t p2p_first = 0;
    size_t ndt_first = 0;
    size_t p2p_second = 0;
    size_t ndt_second = 0;
    bool second_step_executed = false;
    const char* lidar_diagnostic_stage = "none";
    double p2p_residual_median = std::numeric_limits<double>::quiet_NaN();
    double p2p_residual_p95 = std::numeric_limits<double>::quiet_NaN();
    double p2p_weight_mean = std::numeric_limits<double>::quiet_NaN();
    size_t p2p_nonfinite_count = 0;
    double ndt_residual_median = std::numeric_limits<double>::quiet_NaN();
    double ndt_residual_p95 = std::numeric_limits<double>::quiet_NaN();
    double ndt_weight_mean = std::numeric_limits<double>::quiet_NaN();
    size_t ndt_nonfinite_count = 0;
    double information_min_eigenvalue = std::numeric_limits<double>::quiet_NaN();
    size_t ieskf_iterations_used = 0;
    bool ieskf_converged = false;
};

struct ResidualSummary {
    double median = std::numeric_limits<double>::quiet_NaN();
    double p95 = std::numeric_limits<double>::quiet_NaN();
    double weight_mean = std::numeric_limits<double>::quiet_NaN();
};

ResidualSummary summarizeResiduals(const std::vector<double>& residuals, double weight_sum) {
    ResidualSummary summary;
    if (residuals.empty()) return summary;

    std::vector<double> sorted = residuals;
    std::sort(sorted.begin(), sorted.end());
    const auto percentile = [&sorted](double quantile) {
        const double index = quantile * static_cast<double>(sorted.size() - 1);
        const size_t lower = static_cast<size_t>(std::floor(index));
        const size_t upper = static_cast<size_t>(std::ceil(index));
        const double alpha = index - static_cast<double>(lower);
        return sorted[lower] + alpha * (sorted[upper] - sorted[lower]);
    };

    summary.median = percentile(0.5);
    summary.p95 = percentile(0.95);
    summary.weight_mean = weight_sum / static_cast<double>(residuals.size());
    return summary;
}

double informationMinEigenvalue(const Mat6D& information) {
    if (!information.allFinite() || information.isZero(0.0)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const Mat6D symmetric_information = 0.5 * (information + information.transpose());
    Eigen::SelfAdjointEigenSolver<Mat6D> solver(symmetric_information, Eigen::EigenvaluesOnly);
    if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return std::max(0.0, solver.eigenvalues().minCoeff());
}

std::string formatScalar(double value, int precision) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(precision) << value;
    return stream.str();
}

std::string formatVector(const Vec3D& value) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(6) << "[" << value.x() << ", " << value.y() << ", " << value.z()
           << "]";
    return stream.str();
}

Vec3D rotationToRpyDegrees(const Mat3D& rotation) {
    const double pitch = std::asin(std::clamp(-rotation(2, 0), -1.0, 1.0));
    const double roll = std::atan2(rotation(2, 1), rotation(2, 2));
    const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
    constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;
    return kRadiansToDegrees * Vec3D(roll, pitch, yaw);
}

double rotationDeltaDegrees(const Mat3D& from, const Mat3D& to) {
    const Mat3D delta = from.transpose() * to;
    const double cosine = std::clamp(0.5 * (delta.trace() - 1.0), -1.0, 1.0);
    constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;
    return kRadiansToDegrees * std::acos(cosine);
}

template <typename T>
void addDiagnosticItem(ctl::table_out& table, const char* name, const char* type, const T& value) {
    // Avoid CTL_TABLE_ADD_VAR here: its type demangling allocates memory on every
    // call, while this table is emitted for every LiDAR frame.
    table.add_item(name, type, value);
}

void printFrameBegin(const FrameDiagnosticStats& stats) {
    ctl::table_out table("KILO::process begin / frame " + std::to_string(stats.frame_id));
    table.line_color.front_color = ctl::DGREEN;
    table.title_color.front_color = ctl::GREEN;
    table.context_color.front_color = ctl::WHITE;

    addDiagnosticItem(table, "frame_id", "uint64", stats.frame_id);
    addDiagnosticItem(table, "lidar_begin_time", "s", formatScalar(stats.lidar_begin_time, 9));
    addDiagnosticItem(table, "lidar_end_time", "s", formatScalar(stats.lidar_end_time, 9));
    addDiagnosticItem(table, "scan_dt", "s", formatScalar(stats.lidar_end_time - stats.lidar_begin_time, 6));
    addDiagnosticItem(table, "imu_input_num", "size_t", stats.imu_input_num);
    addDiagnosticItem(table, "raw_points", "size_t", stats.raw_points);
    table.make_table_and_out();
}

void printFrameEnd(const FrameDiagnosticStats& stats, const State& state, bool has_previous_pose,
                   const Mat3D& previous_rot, const Vec3D& previous_pos, const char* status) {
    const size_t final_p2p = stats.second_step_executed ? stats.p2p_second : stats.p2p_first;
    const size_t final_ndt = stats.second_step_executed ? stats.ndt_second : stats.ndt_first;
    const double match_ratio = static_cast<double>(final_p2p + final_ndt) /
                               static_cast<double>(std::max<size_t>(1, stats.down_points));
    const double frame_delta_translation = has_previous_pose ? (state.pos_ - previous_pos).norm() : 0.0;
    const double frame_delta_rotation = has_previous_pose ? rotationDeltaDegrees(previous_rot, state.rot_) : 0.0;

    ctl::table_out table("KILO::process end / frame " + std::to_string(stats.frame_id));
    table.line_color.front_color = ctl::BLUE;
    table.title_color.front_color = ctl::PURPLE;
    table.context_color.front_color = ctl::WHITE;

    addDiagnosticItem(table, "status", "string", status);
    addDiagnosticItem(table, "frame_id", "uint64", stats.frame_id);
    addDiagnosticItem(table, "lidar_begin_time", "s", formatScalar(stats.lidar_begin_time, 9));
    addDiagnosticItem(table, "lidar_end_time", "s", formatScalar(stats.lidar_end_time, 9));
    addDiagnosticItem(table, "scan_dt", "s", formatScalar(stats.lidar_end_time - stats.lidar_begin_time, 6));
    addDiagnosticItem(table, "imu_input_num", "size_t", stats.imu_input_num);
    addDiagnosticItem(table, "imu_remaining_num", "size_t", stats.imu_remaining_num);
    addDiagnosticItem(table, "raw_points", "size_t", stats.raw_points);
    addDiagnosticItem(table, "down_points", "size_t", stats.down_points);
    addDiagnosticItem(table, "p2p_first", "size_t", stats.p2p_first);
    addDiagnosticItem(table, "ndt_first", "size_t", stats.ndt_first);
    addDiagnosticItem(table, "p2p_second", "size_t", stats.p2p_second);
    addDiagnosticItem(table, "ndt_second", "size_t", stats.ndt_second);
    addDiagnosticItem(table, "match_ratio", "ratio", formatScalar(match_ratio, 6));
    addDiagnosticItem(table, "lidar_diagnostic_stage", "string", stats.lidar_diagnostic_stage);
    addDiagnosticItem(table, "p2p_residual_median", "normalized", formatScalar(stats.p2p_residual_median, 6));
    addDiagnosticItem(table, "p2p_residual_p95", "normalized", formatScalar(stats.p2p_residual_p95, 6));
    addDiagnosticItem(table, "p2p_weight_mean", "ratio", formatScalar(stats.p2p_weight_mean, 6));
    addDiagnosticItem(table, "ndt_residual_median", "normalized", formatScalar(stats.ndt_residual_median, 6));
    addDiagnosticItem(table, "ndt_residual_p95", "normalized", formatScalar(stats.ndt_residual_p95, 6));
    addDiagnosticItem(table, "ndt_weight_mean", "ratio", formatScalar(stats.ndt_weight_mean, 6));
    addDiagnosticItem(table, "information_min_eigenvalue", "H^T R^-1 H", stats.information_min_eigenvalue);
    addDiagnosticItem(table, "position", "m [x,y,z]", formatVector(state.pos_));
    addDiagnosticItem(table, "rpy", "deg [r,p,y]", formatVector(rotationToRpyDegrees(state.rot_)));
    addDiagnosticItem(table, "velocity", "m/s [x,y,z]", formatVector(state.vel_));
    addDiagnosticItem(table, "ba", "m/s^2 [x,y,z]", formatVector(state.ba_));
    addDiagnosticItem(table, "bw", "rad/s [x,y,z]", formatVector(state.bw_));
    addDiagnosticItem(table, "gravity_norm", "m/s^2", formatScalar(state.grav_.norm(), 6));
    addDiagnosticItem(table, "frame_delta_translation", "m", formatScalar(frame_delta_translation, 6));
    addDiagnosticItem(table, "frame_delta_rotation", "deg", formatScalar(frame_delta_rotation, 6));
    table.make_table_and_out();
}

void appendOptionalDouble(std::ostream& stream, double value) {
    stream << ',';
    if (std::isfinite(value)) stream << value;
}

bool writeFrameDiagnosticCsv(std::ofstream& stream, const std::string& path, const FrameDiagnosticStats& stats,
                             const State& state, bool has_previous_pose, const Mat3D& previous_rot,
                             const Vec3D& previous_pos, const char* status) {
    if (!stream.is_open()) {
        stream.open(path, std::ios::out | std::ios::trunc);
        if (!stream.is_open()) {
            LOG(ERROR) << "Failed to open frontend diagnostic CSV: " << path;
            return false;
        }
        stream << "status,frame_id,lidar_begin_time,lidar_end_time,scan_dt,imu_input_num,imu_remaining_num,"
                  "raw_points,down_points,p2p_first,ndt_first,p2p_second,ndt_second,match_ratio,"
                  "lidar_diagnostic_stage,p2p_residual_median,p2p_residual_p95,p2p_weight_mean,"
                  "p2p_nonfinite_count,ndt_residual_median,ndt_residual_p95,ndt_weight_mean,ndt_nonfinite_count,"
                  "information_min_eigenvalue,ieskf_iterations_used,ieskf_converged,pos_x,pos_y,pos_z,roll_deg,"
                  "pitch_deg,yaw_deg,vel_x,vel_y,vel_z,ba_x,ba_y,ba_z,bw_x,bw_y,bw_z,gravity_norm,"
                  "frame_delta_translation,frame_delta_rotation_deg\n";
        LOG(INFO) << "Frontend diagnostic CSV: " << path;
    }

    const size_t final_p2p = stats.second_step_executed ? stats.p2p_second : stats.p2p_first;
    const size_t final_ndt = stats.second_step_executed ? stats.ndt_second : stats.ndt_first;
    const double match_ratio = static_cast<double>(final_p2p + final_ndt) /
                               static_cast<double>(std::max<size_t>(1, stats.down_points));
    const double frame_delta_translation = has_previous_pose ? (state.pos_ - previous_pos).norm() : 0.0;
    const double frame_delta_rotation = has_previous_pose ? rotationDeltaDegrees(previous_rot, state.rot_) : 0.0;
    const Vec3D rpy = rotationToRpyDegrees(state.rot_);

    stream << std::setprecision(17) << status << ',' << stats.frame_id << ',' << stats.lidar_begin_time << ','
           << stats.lidar_end_time << ',' << stats.lidar_end_time - stats.lidar_begin_time << ','
           << stats.imu_input_num << ',' << stats.imu_remaining_num << ',' << stats.raw_points << ','
           << stats.down_points << ',' << stats.p2p_first << ',' << stats.ndt_first << ',' << stats.p2p_second << ','
           << stats.ndt_second << ',' << match_ratio << ',' << stats.lidar_diagnostic_stage;
    appendOptionalDouble(stream, stats.p2p_residual_median);
    appendOptionalDouble(stream, stats.p2p_residual_p95);
    appendOptionalDouble(stream, stats.p2p_weight_mean);
    stream << ',' << stats.p2p_nonfinite_count;
    appendOptionalDouble(stream, stats.ndt_residual_median);
    appendOptionalDouble(stream, stats.ndt_residual_p95);
    appendOptionalDouble(stream, stats.ndt_weight_mean);
    stream << ',' << stats.ndt_nonfinite_count;
    appendOptionalDouble(stream, stats.information_min_eigenvalue);
    stream << ',' << stats.ieskf_iterations_used << ',' << (stats.ieskf_converged ? 1 : 0) << ',' << state.pos_.x()
           << ',' << state.pos_.y() << ',' << state.pos_.z() << ',' << rpy.x() << ',' << rpy.y() << ',' << rpy.z()
           << ',' << state.vel_.x() << ',' << state.vel_.y() << ',' << state.vel_.z() << ',' << state.ba_.x() << ','
           << state.ba_.y() << ',' << state.ba_.z() << ',' << state.bw_.x() << ',' << state.bw_.y() << ','
           << state.bw_.z() << ',' << state.grav_.norm() << ',' << frame_delta_translation << ','
           << frame_delta_rotation << '\n';
    stream.flush();
    if (!stream.good()) {
        LOG(ERROR) << "Failed to write frontend diagnostic CSV: " << path;
        return false;
    }
    return true;
}
}  // namespace

std::ostream& operator<<(std::ostream& os, const ProcessResult& result) {
    os << "Lidar ProcessResult: {"
       << " valid: " << result.valid << ", total_pts_size: " << result.total_pts_size
       << ", down_pts_size: " << result.down_pts_size << ", success_pts_size: " << result.success_pts_size
       << ", p2p_count: " << result.p2p_count << ", ndt_count: " << result.ndt_count << "}";
    return os;
}

// 【构造函数】把所有配置读进来即可；真正的模块 (ESKF/StateInitial/VoxelGrid/...)
// 都在 initializeFromYaml 内部创建。
KILO::KILO(const std::string& config_file) { this->initializeFromYaml(config_file); }

// 需要 default 析构，且必须放在 .cc 中，因为头文件里 ESKF 等模块是前向声明的（PIMPL 风格），
// 编译器需要看到完整定义才能生成 unique_ptr 的销毁代码。
KILO::~KILO() = default;

// ---- IMU (= 机体系) 位姿：直接来自 ESKF ----
Vec3D KILO::getPosImu() const { return eskf_->getPos(); }
Mat3D KILO::getRotImu() const { return eskf_->getRot(); }

// ---- Lidar 位姿：机体系 → 激光系需要用外参 (R_bl, t_bl) 左乘 ----
//   p_world_lidar_origin = p_world_body_origin + R_wb · t_bl
//   R_wl = R_wb · R_bl
Vec3D KILO::getPosLidar() const { return eskf_->getPos() + eskf_->getRot() * ext_t_; }
Mat3D KILO::getRotLidar() const { return eskf_->getRot() * ext_rot_; }

// --------------------------------------------------------------------------
// 从 YAML 配置文件初始化整个前端 (在构造函数中调用一次)：
//   1) 传感器类型 & ESKF 过程/观测噪声 & 预测模型选择
//   2) 观测/迭代配置：two_step_lidar_eskf / p2p_enable / ndt_enable / IESKF 迭代次数
//   3) 初始化方式（重力对齐 or 单位阵）
//   4) 高斯体素地图参数（体素大小、容量、近邻类型、平面判据、NDT 稳定化）
//   5) 外参 (Lidar → Body) 与降采样体素栅格
// --------------------------------------------------------------------------
void KILO::initializeFromYaml(const std::string& config_file) {
    YamlHelper yaml_helper(config_file);

    // [1] 传感器组合：LO / LIO / KILO
    sensor_type_ = common::parseSensorType(yaml_helper.get<std::string>("sensor_type"));
    LOG(INFO) << "SensorType = " << common::toString(sensor_type_);

    // [2] ESKF 过程噪声（LO 与 LIO/KILO 使用不同键，因为它们打到 Q 的不同块）
    EskfProcessNoise eskf_noise;
    const double vel_process_cov = yaml_helper.get<double>("vel_process_cov", 20.0);
    const double imu_acc_process_cov = yaml_helper.get<double>("imu_acc_process_cov", 500.0);
    const double imu_gyr_process_cov = yaml_helper.get<double>("imu_gyr_process_cov", 1000.0);
    eskf_noise.vel_process_cov =
        common::usesLidarOnly(sensor_type_) ? yaml_helper.get<double>("lo_vel_process_cov") : vel_process_cov;
    eskf_noise.imu_acc_process_cov = imu_acc_process_cov;
    eskf_noise.imu_gyr_process_cov =
        common::usesLidarOnly(sensor_type_) ? yaml_helper.get<double>("lo_imu_gyr_process_cov") : imu_gyr_process_cov;
    eskf_noise.acc_bias_process_cov = yaml_helper.get<double>("acc_bias_process_cov", 0.001);
    eskf_noise.gyr_bias_process_cov = yaml_helper.get<double>("gyr_bias_process_cov", 0.001);
    imu_acc_meas_noise_ = yaml_helper.get<double>("imu_acc_meas_noise");
    imu_gyr_meas_noise_ = yaml_helper.get<double>("imu_gyr_meas_noise");
    if (common::usesKinematics(sensor_type_)) kin_meas_noise_ = yaml_helper.get<double>("kin_meas_noise");
    p2plane_kernel_ = CauchyKernel(yaml_helper.get<double>("p2plane_kernel_threshold", 5.0));
    ndt_kernel_ = CauchyKernel(yaml_helper.get<double>("ndt_kernel_threshold", 5.0));
    if (yaml_helper.hasKey("frontend_diagnostic_print")) {
        frontend_diagnostic_print_ = yaml_helper.get<bool>("frontend_diagnostic_print");
    }
    if (yaml_helper.hasKey("frontend_diagnostic_csv")) {
        frontend_diagnostic_csv_ = yaml_helper.get<bool>("frontend_diagnostic_csv");
    }
    if (frontend_diagnostic_csv_) {
        const std::string result_folder = yaml_helper.get<std::string>("temp_result_save_folder", "temp");
        const std::string diagnostic_dir = std::string(ROOT_DIR) + "result/" + result_folder;
        file_io::ensureDirectory(diagnostic_dir);
        frontend_diagnostic_csv_path_ = diagnostic_dir + "/frontend_diagnostics.csv";
    }

    two_step_lidar_eskf_ = yaml_helper.get<bool>("two_step_lidar_eskf", true);
    p2p_enable_ = yaml_helper.get<bool>("p2plane_enable", true);
    ndt_enable_ = yaml_helper.get<bool>("ndt_enable", false);
    if (!p2p_enable_ && !ndt_enable_) { p2p_enable_ = true; }
    use_state_covariance_for_lidar_points_ = yaml_helper.get<bool>("use_state_covariance_for_lidar_points", true);
    ieskf_max_iterations_ = yaml_helper.get<int>("ieskf_max_iterations", 3);
    eskf_ = std::make_unique<ESKF>(eskf_noise);
    eskf_->setPredictModel(common::usesLidarOnly(sensor_type_) ? PredictModelType::ConstantVelocity
                                                               : PredictModelType::Inertial);

    gravity_ = yaml_helper.get<double>("gravity", 9.81);
    StateInitial::Config init_cfg;
    init_cfg.gravity = gravity_;
    int init_type_val = yaml_helper.get<int>("init_type", 1);
    init_cfg.init_type = (init_type_val == 2) ? InitType::GravityAlignment : InitType::Identity;
    if (common::usesLidarOnly(sensor_type_) && init_cfg.init_type == InitType::GravityAlignment) {
        LOG(WARNING) << "GravityAlignment is ignored in LO mode, fallback to Identity initialization";
        init_cfg.init_type = InitType::Identity;
    }
    switch (sensor_type_) {
        case common::SensorType::LO: state_initial_ = std::make_unique<StateInitialByLidar>(init_cfg); break;
        case common::SensorType::LIO: state_initial_ = std::make_unique<StateInitialByImu>(init_cfg); break;
        case common::SensorType::KILO: state_initial_ = std::make_unique<StateInitialByKinImu>(init_cfg); break;
    }

    // Voxel map
    GaussianVoxelMap::Config voxel_map_config;
    VoxelMapUtils::range_noise = yaml_helper.get<float>("dept_err");
    VoxelMapUtils::degree_noise = yaml_helper.get<float>("beam_err");
    voxel_map_config.voxel_size = yaml_helper.get<double>("voxel_size");
    voxel_map_config.capacity = yaml_helper.get<size_t>("capacity", 1000000u);
    voxel_map_config.nearby_type = static_cast<NearByType>(yaml_helper.get<int>("nearby_type", 0));
    voxel_map_config.voxel_max_num = yaml_helper.get<size_t>("voxel_max_num", 50);
    voxel_map_config.planar_ratio = yaml_helper.get<float>("planar_ratio", 0.1f);
    voxel_map_config.planar_thickness = yaml_helper.get<float>("planar_thickness", 0.05f);
    voxel_map_config.ndt_enable = ndt_enable_;
    voxel_map_config.p2p_enable = p2p_enable_;
    voxel_map_config.ndt_min_points = yaml_helper.get<size_t>("ndt_min_points", 5u);
    voxel_map_config.ndt_jitter = yaml_helper.get<double>("ndt_jitter", 1e-6);
    voxel_map_config.ndt_eigenvalue_regularization = yaml_helper.get<bool>("ndt_eigenvalue_regularization", true);
    voxel_map_config.ndt_min_eigenvalue = yaml_helper.get<double>("ndt_min_eigenvalue", 1e-6);
    voxel_map_config.ndt_max_condition = yaml_helper.get<double>("ndt_max_condition", 1000.0);
    gaussian_voxel_map_ = std::make_unique<GaussianVoxelMap>(voxel_map_config);

    // Extrinsic
    std::vector<double> ext_t = yaml_helper.get<std::vector<double>>("extrinsic_T");
    std::vector<double> ext_R = yaml_helper.get<std::vector<double>>("extrinsic_R");
    ext_rot_ << MAT_FROM_ARRAY(ext_R);
    ext_t_ << VEC_FROM_ARRAY(ext_t);

    // Downsample
    float voxel_grid_resolution = yaml_helper.get<float>("voxel_grid_resolution");
    int vg_mode = yaml_helper.get<int>("voxel_grid_mode", 1);
    size_t vg_target = yaml_helper.get<size_t>("voxel_grid_target_num", 2000u);
    float vg_overshoot = yaml_helper.get<float>("voxel_grid_overshoot", 1.5f);
    voxel_grid_ = std::make_unique<VoxelGrid>(voxel_grid_resolution, static_cast<SamplingMode>(vg_mode), vg_target,
                                              vg_overshoot, 42);
}

// --------------------------------------------------------------------------
// 首帧地图初始化：把降采样后的第一帧点云直接注入高斯体素地图。
// 【流程】
//   for each 点：
//     1) 激光系 → 机体系  (外参 R_bl, t_bl)
//     2) 机体系 → 世界系  (首帧位姿 R=I 或经过 GravityAlignment 的初始旋转)
//     3) 用测距/测角噪声模型 (voxel_map_utils::calculatePointMeasureCov) 算激光系点协方差
//     4) 传播到世界系：cov_world = R·cov_body·Rᵀ + 位姿不确定性项
//     5) 组成 GaussPoint(pt_world, cov_world) 加入地图
// --------------------------------------------------------------------------
GaussCloudPtr KILO::initializeMap(const CloudPtr& cloud_lidar) {
    const size_t N = cloud_lidar->size();
    GaussCloudPtr gauss_cloud_ptr = std::make_shared<GaussCloud>();
    gauss_cloud_ptr->resize(N);
    const Mat3D rot = eskf_->getRot();       // 首帧位姿（可能已经被 GravityAlignment 校准）
    const Vec3D pos = eskf_->getPos();
    const Mat3D rot_cov = eskf_->getRotCov();
    const Mat3D pos_cov = eskf_->getPosCov();
    for (size_t i = 0; i < N; ++i) {
        const Vec3D pt_lidar = cloud_lidar->points[i].getVector3fMap().cast<double>();
        const Vec3D pt_body = ext_rot_ * pt_lidar + ext_t_;   // 激光 → 机体
        const Vec3D pt_world = rot * pt_body + pos;           // 机体 → 世界
        Mat3D cov_lidar;
        VoxelMapUtils::calculatePointMeasureCov(pt_lidar, cov_lidar);  // 激光系测量协方差

        gauss_cloud_ptr->at(i).pt = pt_world;
        // 机体系点协方差 = R_bl · cov_lidar · R_bl^T (只有旋转外参)
        // 再由 computeWorldPointCov 传播到世界系并叠加位姿不确定性
        gauss_cloud_ptr->at(i).cov =
            computeWorldPointCov(pt_body, ext_rot_ * cov_lidar * ext_rot_.transpose(), rot, rot_cov, pos_cov);
    }
    gaussian_voxel_map_->insertPoints(*gauss_cloud_ptr);
    return gauss_cloud_ptr;
}

// --------------------------------------------------------------------------
// 把一个"机体系点 + 机体系协方差"传播到世界系。
//
// 【推导（一阶传播）】设 p_w = R · p_b + t，扰动模型：
//   R_perturbed = R · Exp(δθ),   t_perturbed = t + δp
//   δp_w ≈ (-R·[p_b]×) · δθ + R · δp_b + δp
//     ↑↑ 关键：δθ 前面的雅可比是 -R·[p_b]× ，
//        但对协方差传播只关心两边平方，(±R·[p_b]×) 结果相同，所以本函数用了正号。
//
//   Σ_pw = R Σ_pb Rᵀ                                     ← 点测量本身
//        + (R·[p_b]×) Σ_θ (R·[p_b]×)ᵀ                    ← 旋转不确定性引起
//        + Σ_p                                             ← 位置不确定性引起
//
// 后两项只有在 use_state_covariance_for_lidar_points_=true 时才叠加。
// --------------------------------------------------------------------------
Mat3D KILO::computeWorldPointCov(const Vec3D& point_body, const Mat3D& cov_body, const Mat3D& rot, const Mat3D& rot_cov,
                                 const Mat3D& pos_cov) const {
    Mat3D cov_world = rot * cov_body * rot.transpose();                       // [项 1] 点测量传播
    if (use_state_covariance_for_lidar_points_) {
        const Mat3D rot_crossmat = rot * SKEW_SYM_MATRIX(point_body);         // R·[p_b]×
        cov_world.noalias() += rot_crossmat * rot_cov * rot_crossmat.transpose()  // [项 2] 位姿旋转不确定性
                             + pos_cov;                                       // [项 3] 位姿位置不确定性
    }
    return cov_world;
}

// --------------------------------------------------------------------------
// Stage-2 前的反投影：用当前(Stage-1 完成后)统一位姿把世界系点云打回机体系。
//
// 【为什么需要？】Stage-1 里每个点被"用自己那一时刻的位姿"从激光系→机体系→世界系；
//   完成 Stage-1 之后，我们要在 Stage-2 里对整帧点云在"最终位姿"附近做 IESKF 迭代，
//   因此需要一个统一的机体系表示 p_body = R^T · (p_world - t)。
//
// 【为什么协方差要重算，不能直接反变换？】
//   反投影得到的 p_body 与原始激光系点已经有本质距离（去畸变会造成时刻位姿差异吸收进
//   世界系坐标），因此更保险的做法是"以新机体系坐标为输入，用同一个测量噪声模型重算"，
//   得到与新几何吻合的协方差。
// --------------------------------------------------------------------------
void KILO::backpropagate(const GaussCloud& cloud_world, GaussCloud& cloud_body) const {
    CHECK_EQ(cloud_world.size(), cloud_body.size());

    const Mat3D rot = eskf_->getRot();
    const Mat3D rotT = rot.transpose();
    const Vec3D pos = eskf_->getPos();

    for (size_t i = 0; i < cloud_body.size(); ++i) {
        const GaussPoint& gs_world = cloud_world[i];
        GaussPoint& gs_body = cloud_body[i];

        // 世界系 → 机体系：p_b = R^T · (p_w - t)
        gs_body.pt.noalias() = rotT * (gs_world.pt - pos);
        // 用新的机体系坐标重新计算测量噪声协方差（等价于把点当作从激光系刚测出来的）
        VoxelMapUtils::calculatePointMeasureCov(gs_body.pt, gs_body.cov);
    }
}

// --------------------------------------------------------------------------
// LO 模式：重置 CV 预测输入缓存（首帧完成或长时间没有 update 后调用）。
// 把 world_vel / gyr 清零，把"最近一次校正位姿"设为当前 ESKF 状态。
// --------------------------------------------------------------------------
void KILO::resetLoPredictInput(double current_time) {
    lo_predict_world_vel_.setZero();
    lo_predict_gyr_.setZero();
    lo_last_corrected_rot_ = eskf_->getRot();
    lo_last_corrected_pos_ = eskf_->getPos();
    lo_last_corrected_time_ = current_time;
    lo_has_corrected_state_ = true;
}

// --------------------------------------------------------------------------
// LO 模式：用最近两次校正后的位姿差分，估计 world_vel 与体系角速度 gyr，
// 供下一帧 ConstantVelocity 预测使用。
//
// 【公式】
//   dt = current_time - lo_last_corrected_time_
//   gyr        = Log(R_prev^T · R_cur) / dt            (右扰动约定下的 so(3) 角速度)
//   world_vel  = (p_cur - p_prev) / dt                 (世界系速度)
//
// 【易错】不要用 rot_cur * rot_prev^T ⇒ 那是左扰动约定；本工程一律右乘 (见 State::operator+=)
// --------------------------------------------------------------------------
void KILO::updateLoPredictInput(double current_time) {
    if (!common::usesLidarOnly(sensor_type_)) return;   // 非 LO 模式无需维护

    const Mat3D rot_cur = eskf_->getRot();
    const Vec3D pos_cur = eskf_->getPos();
    if (lo_has_corrected_state_) {
        const double dt = current_time - lo_last_corrected_time_;
        if (dt > 1e-6) {                               // 避免除 0
            const Eigen::Matrix3d diff_rot = lo_last_corrected_rot_.transpose() * rot_cur;  // ΔR = R_prev^T · R_cur
            lo_predict_gyr_ = Log(diff_rot) / dt;                                            // 角速度 (rad/s)
            lo_predict_world_vel_ = (pos_cur - lo_last_corrected_pos_) / dt;                 // 世界系速度 (m/s)
        }
    }

    // 滚动更新"最近一次校正位姿"
    lo_last_corrected_rot_ = rot_cur;
    lo_last_corrected_pos_ = pos_cur;
    lo_last_corrected_time_ = current_time;
    lo_has_corrected_state_ = true;
}

// --------------------------------------------------------------------------
// 通用 ESKF 预测：按 sensor_type_ 选择 predict_input 的来源，然后调 ESKF::predict
// 两次以正确处理"协方差时间基准 ≠ 状态时间基准"的问题。
//
// 【为什么调两次 predict？】
//   - 协方差 P 需要覆盖 (last_update_time → current_time) 这整段时间 (dt_cov)
//   - 名义状态 x 只需要从 (last_predict_time → current_time) 增量推进 (dt)
//   这样避免"状态被 predict 多次而 P 只吸收一次噪声"或反之的错配。
// --------------------------------------------------------------------------
bool KILO::eskfPredict(double current_time) {
    ESKF::PredictInput predict_input;
    if (common::usesLidarOnly(sensor_type_)) {
        // LO：使用上一帧差分得到的 world_vel / gyr
        predict_input.imu_gyr = lo_predict_gyr_;
        predict_input.world_vel = lo_predict_world_vel_;
    } else {
        // LIO / KILO：Point-LIO 建模，imu_a_ / imu_w_ 是被估计的"体系当前 a/ω"
        predict_input.imu_acc = eskf_->state().imu_a_;
        predict_input.imu_gyr = eskf_->state().imu_w_;
    }

    // [步骤 1] 只推协方差：从上次 update 时刻到 current_time
    double dt_cov = current_time - last_state_update_time_;
    eskf_->predict(dt_cov, predict_input, false, true);
    // [步骤 2] 只推名义状态：从上次 predict 时刻到 current_time
    double dt = current_time - last_state_predict_time_;
    eskf_->predict(dt, predict_input, true, false);
    last_state_predict_time_ = current_time;
    return true;
}

// --------------------------------------------------------------------------
// Stage-1 桶级点云更新：对 [idx_i, idx_j) 这段"同一 curvature 时刻"的点做增量 ESKF。
//
// 【为什么这么做能"去畸变"】
//   激光雷达一帧内的每个点在不同时刻被打出，若直接用一个位姿把整帧点变到世界系会有
//   运动畸变。Stage-1 里：
//     for 每个 curvature 桶 (=时间片)：
//       1) 先 eskfPredict 把位姿推进到该时刻
//       2) 用"当前时刻的位姿"把该桶内的点转到机体系/世界系
//       3) 在地图上找匹配 → 更新位姿 (updateByPoints)
//   于是每个点等价于用"它自己的时刻"下的位姿去展开，天然完成运动畸变去除。
//
// 【流程】
//   [1] predict：把 ESKF 状态外推到 current_time
//   [2] 遍历桶内点：
//       a) 激光系 → 机体系 → 世界系（用 predict 后位姿）
//       b) 在高斯体素地图找最近平面 → P2P 残差；若失败并且启用 NDT 则用 NDT
//   [3] 若有有效残差，组装 (z, H, R)，调用 ESKF::updateByPoints 完成一次 Kalman step
//   [4] 用更新后的位姿重算世界系点云与协方差（供后续步骤/可视化）
// --------------------------------------------------------------------------
KILO::LidarUpdateDiagnostics KILO::predictUpdatePoint(const double current_time, const size_t idx_i,
                                                      const size_t idx_j, const CloudPtr& cloud_lidar_pcl,
                                                      GaussCloud& cloud_body, GaussCloud& cloud_world,
                                                      LidarMatchTypes* match_types) {
    // [1] 预测：把状态推进到当前桶时刻
    this->eskfPredict(current_time);
    const Mat3D rot_predict = eskf_->getRot();
    const Vec3D pos_predict = eskf_->getPos();
    const Mat3D rot_cov_predict = eskf_->getRotCov();
    const Mat3D pos_cov_predict = eskf_->getPosCov();

    // [2] 构建残差
    size_t points_size = idx_j - idx_i;
    std::vector<KNearestRes<1>> p2p_results;   // 每点 1 维（点到平面的有符号距离）
    std::vector<KNearestRes<3>> ndt_results;   // 每点 3 维（NDT 白化后的位移残差）
    p2p_results.reserve(points_size);
    if (ndt_enable_) ndt_results.reserve(points_size);

    for (size_t i = 0; i < points_size; ++i) {
        const size_t cur_idx = i + idx_i;
        const Vec3D pt_lidar = cloud_lidar_pcl->points[cur_idx].getArray3fMap().cast<double>();

        // [2.1] 计算点在机体系 / 世界系下的均值与协方差
        GaussPoint& gs_body = cloud_body[cur_idx];
        GaussPoint& gs_world = cloud_world[cur_idx];

        Mat3D pt_lidar_cov;
        VoxelMapUtils::calculatePointMeasureCov(pt_lidar, pt_lidar_cov);   // 激光系测量协方差
        gs_body.pt = ext_rot_ * pt_lidar + ext_t_;                          // 激光 → 机体
        gs_body.cov = ext_rot_ * pt_lidar_cov * ext_rot_.transpose();       // 协方差外参传播 (仅 R_bl)

        gs_world.pt = rot_predict * gs_body.pt + pos_predict;                // 机体 → 世界 (用 predict 位姿)
        gs_world.cov = computeWorldPointCov(gs_body.pt, gs_body.cov, rot_predict, rot_cov_predict, pos_cov_predict);

        // [2.2] 在高斯体素地图中查邻居并构建残差
        KNearestInput knn_input(&gs_body.pt, &gs_body.cov, &gs_world.pt, &gs_world.cov, &rot_predict);
        KNearestRes<1> p2p_result;
        if (p2p_enable_) { gaussian_voxel_map_->buildPoint2PlaneResidual(knn_input, p2p_result, kFirstStepLidarNoise); }

        // [2.3] 优先 P2P；P2P 失败并且启用 NDT 则尝试 NDT
        if (p2p_result.valid) {
            p2p_results.push_back(p2p_result);
            if (match_types && cur_idx < match_types->size()) { (*match_types)[cur_idx] = LidarMatchType::Point2Plane; }
        } else if (ndt_enable_) {
            KNearestRes<3> ndt_result;
            gaussian_voxel_map_->buildNdtResidual(knn_input, ndt_result, kFirstStepLidarNoise);
            if (ndt_result.valid) {
                ndt_results.push_back(ndt_result);
                if (match_types && cur_idx < match_types->size()) { (*match_types)[cur_idx] = LidarMatchType::Ndt; }
            }
        }
    }

    const size_t p2p_count = p2p_results.size();
    const size_t ndt_count = ndt_results.size();
    LidarUpdateDiagnostics diagnostics;
    diagnostics.p2p_count = p2p_count;
    diagnostics.ndt_count = ndt_count;
    const bool collect_diagnostics =
        !two_step_lidar_eskf_ && (frontend_diagnostic_print_ || frontend_diagnostic_csv_);
    if (collect_diagnostics) {
        diagnostics.p2p_residuals.reserve(p2p_count);
        diagnostics.ndt_residuals.reserve(ndt_count);
    }
    bool eskf_update_enable = (p2p_count + ndt_count) > 0;

    if (eskf_update_enable) {
        // [3] 组装观测：H 只有 6 列（rot+pos）。每 P2P 占 1 行，每 NDT 占 3 行
        ObsShared obs_shared;
        size_t h_total_rows = p2p_results.size() + 3 * ndt_results.size();
        obs_shared.pt_h.resize(h_total_rows, 6);
        obs_shared.pt_R.resize(h_total_rows);
        obs_shared.pt_z.resize(h_total_rows);

        // ---- 填充 P2P 部分 ----
        for (size_t k = 0; k < p2p_results.size(); ++k) {
            obs_shared.pt_h.row(k) = p2p_results[k].J;
            obs_shared.pt_z(k) = p2p_results[k].r(0, 0);
            // Cauchy 鲁棒核：weight = 1 / (1 + r²/δ²)
            const double p2plane_mahalanobis2 = p2p_results[k].r.squaredNorm();
            const double p2plane_weight = p2plane_kernel_.weight(p2plane_mahalanobis2);
            const double variance_scale = robustVarianceScale(p2plane_weight);   // σ²_scale = 1/w
            const double effective_weight = 1.0 / variance_scale;                 // = w
            obs_shared.pt_R(k) = variance_scale;                                  // 大残差 → 大方差 → 弱权重
            const double residual_abs = std::abs(p2p_results[k].r(0, 0));
            if (collect_diagnostics) {
                if (std::isfinite(residual_abs) && std::isfinite(effective_weight) &&
                    p2p_results[k].J.allFinite()) {
                    diagnostics.p2p_residuals.push_back(residual_abs);
                    diagnostics.p2p_weight_sum += effective_weight;
                    // 信息矩阵累加 w·Jᵀ J，用来评估当前观测的位姿可观测性
                    diagnostics.information.noalias() +=
                        effective_weight * p2p_results[k].J.transpose() * p2p_results[k].J;
                } else {
                    ++diagnostics.p2p_nonfinite_count;   // NaN/Inf 计数，方便定位异常帧
                }
            }
        }

        // ---- 填充 NDT 部分（紧接在 P2P 之后）----
        const size_t p2p_end_row = p2p_results.size();
        for (size_t m = 0; m < ndt_results.size(); ++m) {
            obs_shared.pt_h.block<3, 6>(p2p_end_row + 3 * m, 0) = ndt_results[m].J;
            obs_shared.pt_z.segment<3>(p2p_end_row + 3 * m) = ndt_results[m].r;
            const double ndt_weight = ndt_kernel_.weight(ndt_results[m].r.squaredNorm());
            const double variance_scale = robustVarianceScale(ndt_weight);
            const double effective_weight = 1.0 / variance_scale;
            obs_shared.pt_R.segment<3>(p2p_end_row + 3 * m) = variance_scale * Eigen::Vector3d::Ones();
            const double residual_norm = ndt_results[m].r.norm();
            if (collect_diagnostics) {
                if (std::isfinite(residual_norm) && std::isfinite(effective_weight) &&
                    ndt_results[m].J.allFinite()) {
                    diagnostics.ndt_residuals.push_back(residual_norm);
                    diagnostics.ndt_weight_sum += effective_weight;
                    diagnostics.information.noalias() +=
                        effective_weight * ndt_results[m].J.transpose() * ndt_results[m].J;
                } else {
                    ++diagnostics.ndt_nonfinite_count;
                }
            }
        }

        // [4] 一次 ESKF 更新（非迭代，因为 Stage-1 追求速度而非精度）
        eskf_->updateByPoints(obs_shared);
        last_state_update_time_ = current_time;

        // [5] 用更新后的位姿重算世界系点云与协方差 —— 关键：这里等价于"把该桶去畸变一次"
        // 之后同一桶的世界系点云会被写入 cloud_world 供 Stage-2 或地图更新使用
        const Mat3D rot_update = eskf_->getRot();
        const Vec3D pos_update = eskf_->getPos();
        const Mat3D rot_cov_update = eskf_->getRotCov();
        const Mat3D pos_cov_update = eskf_->getPosCov();
        for (size_t i = 0; i < points_size; ++i) {
            const size_t cur_idx = i + idx_i;
            const GaussPoint& gs_body = cloud_body[cur_idx];
            GaussPoint& gs_world = cloud_world[cur_idx];

            gs_world.pt = rot_update * gs_body.pt + pos_update;
            gs_world.cov = computeWorldPointCov(gs_body.pt, gs_body.cov, rot_update, rot_cov_update, pos_cov_update);
        }
    }
    return diagnostics;
}

// --------------------------------------------------------------------------
// Stage-2 整帧 IESKF：在 backpropagate 已经把点云"翻回机体系"之后，对整帧点做迭代精化。
//
// 【流程】
//   state_before_iter = 当前 ESKF 状态 (作为 IESKF 的先验 x^{prior})
//   for iter in 0..ieskf_max_iterations_-1:
//     [1] 用当前迭代状态把 cloud_body → cloud_world (更新位姿协方差)
//     [2] 并行为每个点在体素地图上构建 P2P / NDT 残差
//     [3] 用 Cauchy 核加权后组装 (z, H, R)
//     [4] updateByCloud(obs, x^{prior}, N, iter) 完成一步 IESKF 修正
//         若收敛则 break
//   [5] 迭代结束后，最后再用"最终位姿"重刷 cloud_world 供地图更新使用
//
// 【为什么用 TBB 并行】每个点独立查询高斯体素地图，没有共享状态争用，非常适合并行。
// --------------------------------------------------------------------------
KILO::LidarUpdateDiagnostics KILO::predictUpdateCloud(const GaussCloud& cloud_body, GaussCloud& cloud_world,
                                                      LidarMatchTypes* match_types) {
    CHECK_EQ(cloud_body.size(), cloud_world.size());

    const State state_before_iter = eskf_->getState();   // x^{prior}：IESKF 每次迭代都要用来构造修正项
    size_t iteration = 0;
    size_t p2p_valid_num = 0;
    size_t ndt_valid_num = 0;
    bool converge = false;
    LidarUpdateDiagnostics diagnostics;
    const bool collect_diagnostics = frontend_diagnostic_print_ || frontend_diagnostic_csv_;

    for (; iteration < ieskf_max_iterations_; ++iteration) {
        p2p_valid_num = 0;
        ndt_valid_num = 0;
        // [1] 用当前迭代状态刷新 cloud_world（因为位姿在每次迭代都变了）
        const Mat3D R_cur = eskf_->getRot();
        const Vec3D pos_cur = eskf_->getPos();
        const Mat3D rot_cov_cur = eskf_->getRotCov();
        const Mat3D pos_cov_cur = eskf_->getPosCov();
        const size_t N = cloud_body.size();

        for (size_t j = 0; j < N; ++j) {
            const GaussPoint& gs_body = cloud_body[j];
            GaussPoint& gs_world = cloud_world[j];
            gs_world.pt = R_cur * gs_body.pt + pos_cur;
            gs_world.cov = computeWorldPointCov(gs_body.pt, gs_body.cov, R_cur, rot_cov_cur, pos_cov_cur);
        }

        // [2] 并行为每个点构造残差
        std::vector<KNearestRes<1>> p2p_results(N);
        std::vector<KNearestRes<3>> ndt_results;
        if (ndt_enable_) ndt_results.resize(N);

        const auto process_func = [&](size_t i) {
            KNearestInput knn_input(&cloud_body[i].pt, &cloud_body[i].cov, &cloud_world[i].pt, &cloud_world[i].cov,
                                    &R_cur);
            if (p2p_enable_) {
                gaussian_voxel_map_->buildPoint2PlaneResidual(knn_input, p2p_results[i], kSecondStepLidarNoise);
            }
            // 若 P2P 未启用 或 P2P 匹配失败，回退尝试 NDT
            if ((!p2p_enable_ || p2p_results[i].valid == false) && ndt_enable_) {
                gaussian_voxel_map_->buildNdtResidual(knn_input, ndt_results[i], kSecondStepLidarNoise);
            }
        };

        // TBB 并行调度：粒度 kGain=128（经验值，兼顾任务切分开销与负载均衡）
        static constexpr size_t kGain = 128;
        tbb::parallel_for(tbb::blocked_range<size_t>(0, N, kGain), [&](const tbb::blocked_range<size_t>& r) {
            for (size_t i = r.begin(); i < r.end(); ++i) process_func(i);
        });

        p2p_valid_num =
            std::count_if(p2p_results.begin(), p2p_results.end(), [](const KNearestRes<1>& res) { return res.valid; });
        ndt_valid_num = ndt_enable_ ? std::count_if(ndt_results.begin(), ndt_results.end(),
                                                    [](const KNearestRes<3>& res) { return res.valid; })
                                    : 0;
        diagnostics = LidarUpdateDiagnostics{};
        diagnostics.p2p_count = p2p_valid_num;
        diagnostics.ndt_count = ndt_valid_num;
        if (collect_diagnostics) {
            diagnostics.p2p_residuals.reserve(p2p_valid_num);
            diagnostics.ndt_residuals.reserve(ndt_valid_num);
        }
        diagnostics.ieskf_iterations_used = iteration + 1;
        if (match_types && match_types->size() == N) {
            std::fill(match_types->begin(), match_types->end(), LidarMatchType::Unused);
            for (size_t i = 0; i < N; ++i) {
                if (p2p_results[i].valid) {
                    (*match_types)[i] = LidarMatchType::Point2Plane;
                } else if (ndt_enable_ && ndt_results[i].valid) {
                    (*match_types)[i] = LidarMatchType::Ndt;
                }
            }
        }
        bool eskf_update_enable = (p2p_valid_num + ndt_valid_num) > 0;
        if (eskf_update_enable) {
            ObsShared obs_shared;
            size_t h_total_rows = p2p_valid_num + 3 * ndt_valid_num;
            obs_shared.pt_h.resize(h_total_rows, 6);
            obs_shared.pt_R.resize(h_total_rows);
            obs_shared.pt_z.resize(h_total_rows);

            size_t row = 0;
            for (size_t k = 0; k < p2p_results.size(); ++k) {
                if (!p2p_results[k].valid) continue;
                obs_shared.pt_h.row(row) = p2p_results[k].J;
                obs_shared.pt_z(row) = p2p_results[k].r(0, 0);
                const double p2plane_mahalanobis2 = p2p_results[k].r.squaredNorm();
                const double p2plane_weight = p2plane_kernel_.weight(p2plane_mahalanobis2);
                const double variance_scale = robustVarianceScale(p2plane_weight);
                const double effective_weight = 1.0 / variance_scale;
                obs_shared.pt_R(row) = variance_scale;
                const double residual_abs = std::abs(p2p_results[k].r(0, 0));
                if (collect_diagnostics) {
                    if (std::isfinite(residual_abs) && std::isfinite(effective_weight) &&
                        p2p_results[k].J.allFinite()) {
                        diagnostics.p2p_residuals.push_back(residual_abs);
                        diagnostics.p2p_weight_sum += effective_weight;
                        diagnostics.information.noalias() +=
                            effective_weight * p2p_results[k].J.transpose() * p2p_results[k].J;
                    } else {
                        ++diagnostics.p2p_nonfinite_count;
                    }
                }
                ++row;
            }
            for (size_t m = 0; m < ndt_results.size(); ++m) {
                if (!ndt_results[m].valid) continue;
                obs_shared.pt_h.block<3, 6>(row, 0) = ndt_results[m].J;
                obs_shared.pt_z.segment<3>(row) = ndt_results[m].r;
                const double ndt_weight = ndt_kernel_.weight(ndt_results[m].r.squaredNorm());
                const double variance_scale = robustVarianceScale(ndt_weight);
                const double effective_weight = 1.0 / variance_scale;
                obs_shared.pt_R.segment<3>(row) = variance_scale * Eigen::Vector3d::Ones();
                const double residual_norm = ndt_results[m].r.norm();
                if (collect_diagnostics) {
                    if (std::isfinite(residual_norm) && std::isfinite(effective_weight) &&
                        ndt_results[m].J.allFinite()) {
                        diagnostics.ndt_residuals.push_back(residual_norm);
                        diagnostics.ndt_weight_sum += effective_weight;
                        diagnostics.information.noalias() +=
                            effective_weight * ndt_results[m].J.transpose() * ndt_results[m].J;
                    } else {
                        ++diagnostics.ndt_nonfinite_count;
                    }
                }
                row += 3;
            }

            // [4] 一步 IESKF 修正（首次等价 EKF，之后带先验校正项）；返回是否收敛
            converge = eskf_->updateByCloud(obs_shared, state_before_iter, ieskf_max_iterations_, iteration);
            diagnostics.ieskf_converged = converge;
            if (converge) break;    // 提前收敛就跳出，节约时间
        }
    }
    // LOG(INFO) << "IESKF iteration times: " << iteration + 1 << ", converge: " << converge;

    // [5] 用最终位姿再刷一遍 cloud_world，保证下游地图更新时几何一致
    const Mat3D R_final = eskf_->getRot();
    const Vec3D pos_final = eskf_->getPos();
    const Mat3D rot_cov_final = eskf_->getRotCov();
    const Mat3D pos_cov_final = eskf_->getPosCov();
    for (size_t j = 0; j < cloud_body.size(); ++j) {
        const GaussPoint& gs_body = cloud_body[j];
        GaussPoint& gs_world = cloud_world[j];
        gs_world.pt = R_final * gs_body.pt + pos_final;
        gs_world.cov = computeWorldPointCov(gs_body.pt, gs_body.cov, R_final, rot_cov_final, pos_cov_final);
    }
    return diagnostics;
}

// --------------------------------------------------------------------------
// Point-LIO 风格 IMU 更新：把 IMU 原始 (acc, gyr) 作为对"imu_a + ba"、"imu_w + bw"的观测。
//
// 【观测方程】
//   z_acc = (g/|a|) · acc_meas - imu_a - ba          ⇒ 期望为 0（若一致则残差为零）
//   z_gyr =            gyr_meas - imu_w - bw
//   H 结构见 ESKF::updateByImu：只在列 [9..15) 与 [18..24) 有单位阵子块。
//
// 【比例项 gravity_/acc_norm_ 的意义】
//   IMU 原始加速度单位可能不是 m/s²（有的驱动输出 g 单位，或 mg）。
//   初始化阶段（StateInitial）用静止段测出加速度模长 acc_norm_，
//   然后用 gravity_/acc_norm_ 把它"归一化"到 m/s²。
// --------------------------------------------------------------------------
bool KILO::predictUpdateImu(const ros_compat::ImuMsgPtr& imu) {
    // [1] 先把状态推到 IMU 时刻
    double current_time = ros_compat::toSec(imu->header.stamp);
    this->eskfPredict(current_time);

    // [2] 构造 IMU 观测 (z, R)；H 由 ESKF::updateByImu 内部利用稀疏结构隐式处理
    ObsShared obs_shared;
    obs_shared.ki_R.resize(6);
    obs_shared.ki_z.resize(6);
    Vec3D imu_acc(imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z);
    Vec3D imu_gyr(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z);
    // 前 3 维：加速度残差；后 3 维：角速度残差（体系）
    obs_shared.ki_z.block<3, 1>(0, 0) = (gravity_ / acc_norm_) * imu_acc - eskf_->state().imu_a_ - eskf_->state().ba_;
    obs_shared.ki_z.block<3, 1>(3, 0) = imu_gyr - eskf_->state().imu_w_ - eskf_->state().bw_;

    // R 为对角阵（各轴方差相同）
    obs_shared.ki_R << imu_acc_meas_noise_, imu_acc_meas_noise_, imu_acc_meas_noise_, imu_gyr_meas_noise_,
        imu_gyr_meas_noise_, imu_gyr_meas_noise_;

    eskf_->updateByImu(obs_shared);
    last_state_update_time_ = current_time;
    return true;
}

// --------------------------------------------------------------------------
// KILO 模式：IMU + 足端运动学联合更新。
//
// 【腿式运动学观测模型】(仅对处于接触状态的足端 i)
//   假设足端在世界系静止（接触点无滑动）：v_world_foot ≈ 0
//     v_world_foot = R · (ω × p_foot_body + v_foot_body) + v_body_world
//   ⇒  0 = v_world + R · (ω × p_foot + v_foot)
//   ⇒  z_kin_i = -v_world - R · (ω × p_foot + v_foot)         (期望为 0)
//
//   对状态求扰动导数：
//     ∂z/∂δθ (第 0..3 列) = -R · [ω × p_foot + v_foot]×
//     ∂z/∂δv (第 6..9 列) = -I
//     ∂z/∂δω (第 21..24 列，对应 imu_w) = -R · [p_foot]×
//   其余列为 0。
//
// 【观测拼装】前 6 行是 IMU（同 predictUpdateImu），
// 之后每个 contact 足端追加 3 行运动学观测。
// --------------------------------------------------------------------------
bool KILO::predictUpdateKinImu(const common::KinImuMeas& kin_imu) {
    double current_time = kin_imu.time_stamp_;
    // [1] 先把状态推到当前时刻
    this->eskfPredict(current_time);

    // [2] 统计当前接触地面的足端数（腿序：FR FL RR RL）
    int contact_nums = 0;
    for (int i = 0; i < 4; ++i) {
        if (kin_imu.contact_[i]) { contact_nums++; }
    }

    // [3] 分配 (z, H, R)：6 行 IMU + 3 行×每个接触足
    ObsShared obs_shared;
    obs_shared.ki_R.resize(6 + 3 * contact_nums);
    obs_shared.ki_z.resize(6 + 3 * contact_nums);
    obs_shared.ki_h.resize(6 + 3 * contact_nums, DIM_STATE);
    obs_shared.ki_h.setZero();

    // ---- IMU 部分：H 稀疏结构 (同 updateByImu) ----
    obs_shared.ki_h.block<6, 6>(0, 9) = Eigen::Matrix<double, 6, 6>::Identity();   // 对 ba/bw
    obs_shared.ki_h.block<6, 6>(0, 18) = Eigen::Matrix<double, 6, 6>::Identity();  // 对 imu_a/imu_w
    Vec3D imu_acc(kin_imu.acc_[0], kin_imu.acc_[1], kin_imu.acc_[2]);
    Vec3D imu_gyr(kin_imu.gyr_[0], kin_imu.gyr_[1], kin_imu.gyr_[2]);
    obs_shared.ki_z.block<3, 1>(0, 0) = (gravity_ / acc_norm_) * imu_acc - eskf_->state().imu_a_ - eskf_->state().ba_;
    obs_shared.ki_z.block<3, 1>(3, 0) = imu_gyr - eskf_->state().imu_w_ - eskf_->state().bw_;

    obs_shared.ki_R.block<6, 1>(0, 0) << imu_acc_meas_noise_, imu_acc_meas_noise_, imu_acc_meas_noise_,
        imu_gyr_meas_noise_, imu_gyr_meas_noise_, imu_gyr_meas_noise_;

    // ---- 足端运动学部分：接触足才贡献观测 ----
    int idx = 0;
    Mat3D w_skew = SKEW_SYM_MATRIX(eskf_->state().imu_w_);  // [ω]×，用于 ω × p_foot 与 [ω × p_foot + v]×
    for (int i = 0; i < 4; ++i) {
        if (kin_imu.contact_[i]) {
            Vec3D foot_pos(kin_imu.foot_pos_[i][0], kin_imu.foot_pos_[i][1], kin_imu.foot_pos_[i][2]);
            Vec3D foot_vel(kin_imu.foot_vel_[i][0], kin_imu.foot_vel_[i][1], kin_imu.foot_vel_[i][2]);

            // 机体系下足端相对机体的速度：ω × p_foot + v_foot
            Vec3D w_skew_pos_vel = w_skew * foot_pos + foot_vel;

            // ∂z/∂δθ = -R · [ω × p_foot + v_foot]×
            obs_shared.ki_h.block<3, 3>(6 + 3 * idx, 0) = -eskf_->getRot() * SKEW_SYM_MATRIX(w_skew_pos_vel);
            // ∂z/∂δv = -I（残差里减 v_world）
            obs_shared.ki_h.block<3, 3>(6 + 3 * idx, 6) = Mat3D::Identity();
            // ∂z/∂δω(imu_w) = -R · [p_foot]×
            obs_shared.ki_h.block<3, 3>(6 + 3 * idx, 21) = -eskf_->getRot() * SKEW_SYM_MATRIX(foot_pos);

            // z_kin = -v_world - R · (ω × p_foot + v_foot)  (期望为 0)
            obs_shared.ki_z.block<3, 1>(6 + 3 * idx, 0) = -eskf_->getVel() - eskf_->getRot() * w_skew_pos_vel;

            obs_shared.ki_R.block<3, 1>(6 + 3 * idx, 0) << kin_meas_noise_, kin_meas_noise_, kin_meas_noise_;
            idx++;
        }
    }

    eskf_->updateByKinImu(obs_shared);
    last_state_update_time_ = current_time;
    return true;
}

// ==========================================================================
// 【顶层入口】process(measure)
//   一帧完整的前端流水线：预处理 → 首帧初始化 (仅首帧) →
//                      Stage-1 逐点 ESKF 去畸变 → Stage-2 整帧 IESKF 精化 →
//                      高斯体素地图更新 → 组装 ProcessResult
//
// 【关键时间轴】
//   begin_time                                            end_time
//     ├─── lidar frame (含 curvature 时间偏移) ──────────────┤
//     ├──── IMU / KinImu 队列 (时间戳落在这段内) ─────────────┤
//
//   Stage-1 会按点云 curvature（=相对 begin_time 的偏移）分桶，
//   每桶前先把 IMU/KinImu 里时间戳更早的消息全部消费掉。
// ==========================================================================
ProcessResult KILO::process(common::MeasGroup& measure) {
    ProcessResult result;

    // ---- 从测量包中取出原始点云与 IMU/KinImu 队列 ----
    CloudPtr cloud_raw = measure.lidar_scan_.cloud_;
    CloudPtr cloud_down_lidar(new PointCloudType());   // 降采样点云 (激光系)
    CloudPtr cloud_down_body(new PointCloudType());    // 降采样点云 (机体系)
    CloudPtr cloud_down_world(new PointCloudType());   // 降采样点云 (世界系)
    GaussCloudPtr gauss_cloud_body(new GaussCloud());  // 带协方差的高斯点 (机体系)
    GaussCloudPtr gauss_cloud_world(new GaussCloud()); // 带协方差的高斯点 (世界系)

    auto& imus = measure.imus_;
    auto& kin_imus = measure.kin_imus_;
    double begin_time = measure.lidar_scan_.lidar_begin_time_;
    double end_time = measure.lidar_scan_.lidar_end_time_;

    FrameDiagnosticStats diagnostic;
    diagnostic.frame_id = diagnostic_frame_id_++;
    diagnostic.lidar_begin_time = begin_time;
    diagnostic.lidar_end_time = end_time;
    diagnostic.imu_input_num = common::usesImu(sensor_type_)
                                   ? imus.size()
                                   : (common::usesKinematics(sensor_type_) ? kin_imus.size() : 0);
    diagnostic.imu_remaining_num = diagnostic.imu_input_num;
    diagnostic.raw_points = cloud_raw->size();
    if (frontend_diagnostic_print_) { printFrameBegin(diagnostic); }
    const auto emitFrameDiagnostic = [this, &diagnostic](const char* status) {
        if (frontend_diagnostic_print_) {
            printFrameEnd(diagnostic, eskf_->state(), diagnostic_has_previous_pose_, diagnostic_previous_rot_,
                          diagnostic_previous_pos_, status);
        }
        if (frontend_diagnostic_csv_ &&
            !writeFrameDiagnosticCsv(frontend_diagnostic_csv_stream_, frontend_diagnostic_csv_path_, diagnostic,
                                     eskf_->state(), diagnostic_has_previous_pose_, diagnostic_previous_rot_,
                                     diagnostic_previous_pos_, status)) {
            frontend_diagnostic_csv_ = false;
        }
    };

    if (cloud_raw->points.empty() || (common::usesImu(sensor_type_) && imus.empty()) ||
        (common::usesKinematics(sensor_type_) && kin_imus.empty())) {
        LOG(WARNING) << "Data packet is not ready";
        emitFrameDiagnostic("data_not_ready");
        return result;
    }

    // ---- 体素栅格降采样：把成千上万点减到 target_num 附近，Stage-1/2 都基于此 ----
    Timer::measure("Downsampling", [&, this]() { voxel_grid_->filter(cloud_raw, cloud_down_lidar); });
    diagnostic.down_points = cloud_down_lidar->size();

    // ---- 首帧初始化 ----
    // 用 StateInitial 估计重力方向、陀螺零偏、初始旋转，然后把首帧点云直接放入地图当作先验
    if (init_flag_) {
        state_initial_->ingest(measure);

        eskf_->state().grav_ = state_initial_->gravityVec();     // 重力向量 (世界系)
        eskf_->state().bw_ = state_initial_->gyroBias();         // 陀螺零偏
        eskf_->state().rot_ = state_initial_->initRotation();    // 初始旋转 (可能已重力对齐)
        eskf_->cov() = 0.000001 * StateCov::Identity();          // P0 = 1e-6 · I（初始状态强先验）

        gauss_cloud_world = this->initializeMap(cloud_down_lidar);
        pcl_utils::GaussCloudToPclCloud(*gauss_cloud_world, cloud_down_world);
        pcl::transformPointCloud(*cloud_down_lidar, *cloud_down_body,
                                 makeIsometry3d(ext_rot_, ext_t_).matrix().cast<float>());

        auto gravity_vec = eskf_->state().grav_;
        auto bw = eskf_->state().bw_;
        LOG(INFO) << "Gravity is initialized to " << std::fixed << std::setprecision(3) << gravity_vec.transpose();
        LOG(INFO) << "IMU bw is initialized to " << bw.transpose();

        acc_norm_ = state_initial_->accNorm();
        last_state_predict_time_ = end_time;
        last_state_update_time_ = end_time;
        if (common::usesLidarOnly(sensor_type_)) { resetLoPredictInput(end_time); }
        init_flag_ = false;

        result.valid = true;
        result.total_pts_size = cloud_raw->size();
        result.down_pts_size = cloud_down_lidar->size();
        result.success_pts_size = cloud_down_lidar->size();
        result.cloud_world = cloud_down_world;
        result.cloud_lidar = cloud_down_lidar;
        result.cloud_body = cloud_down_body;
        result.match_types = std::make_shared<LidarMatchTypes>(cloud_down_lidar->size(), LidarMatchType::Unused);
        emitFrameDiagnostic("initialized");
        diagnostic_previous_rot_ = eskf_->getRot();
        diagnostic_previous_pos_ = eskf_->getPos();
        diagnostic_has_previous_pose_ = true;
        return result;
    }

    // ---- 为每个降采样点分配"高斯点"存储 (机体系 / 世界系各一份) ----
    size_t p2p_total = 0;
    size_t ndt_total = 0;
    const size_t N = cloud_down_lidar->size();
    gauss_cloud_body->resize(N);
    gauss_cloud_world->resize(N);
    auto match_types = std::make_shared<LidarMatchTypes>(N, LidarMatchType::Unused);
    bool frame_had_lidar_update = false;
    LidarUpdateDiagnostics first_diagnostics;

    // =================== Stage-1: 逐点 ESKF (去畸变) ===================
    Timer::measure("State predict/update - 1st", [&, this]() {
        // [1] 按 curvature (=相对帧起始的时间偏移) 升序排序，让点云"按时间流"处理
        auto& pts = cloud_down_lidar->points;
        std::sort(pts.begin(), pts.end(), time_list);

        // [2] 遍历"相同 curvature 的桶"：每个桶等价于一个时间片
        const size_t pts_size = pts.size();
        size_t idx_i = 0;
        while (idx_i < pts_size) {
            double cur_point_time = begin_time + pts[idx_i].curvature;   // 绝对时刻
            size_t idx_j = idx_i + 1;
            while (idx_j < pts_size && pts[idx_i].curvature == pts[idx_j].curvature) { idx_j++; }

            // [2a] 消费所有比 cur_point_time 更早的 IMU / KinImu 消息 —— 保证滤波器时间戳单调
            if (common::usesImu(sensor_type_)) {
                while (!imus.empty() && ros_compat::toSec(imus.front()->header.stamp) < cur_point_time) {
                    this->predictUpdateImu(imus.front());
                    imus.pop_front();
                }
            } else if (common::usesKinematics(sensor_type_)) {
                while (!kin_imus.empty() && kin_imus.front().time_stamp_ < cur_point_time) {
                    this->predictUpdateKinImu(kin_imus.front());
                    kin_imus.pop_front();
                }
            }

            // [2b] 对当前桶做一次 predict + updateByPoints ⇒ 顺便完成去畸变
            auto bucket_diagnostics = this->predictUpdatePoint(cur_point_time, idx_i, idx_j, cloud_down_lidar,
                                                               *gauss_cloud_body, *gauss_cloud_world,
                                                               match_types.get());
            p2p_total += bucket_diagnostics.p2p_count;
            ndt_total += bucket_diagnostics.ndt_count;
            frame_had_lidar_update =
                frame_had_lidar_update || ((bucket_diagnostics.p2p_count + bucket_diagnostics.ndt_count) > 0);
            first_diagnostics.merge(bucket_diagnostics);   // 汇总本帧的诊断
            idx_i = idx_j;
        }
    });
    diagnostic.p2p_first = p2p_total;
    diagnostic.ndt_first = ndt_total;
    diagnostic.imu_remaining_num = common::usesImu(sensor_type_)
                                       ? imus.size()
                                       : (common::usesKinematics(sensor_type_) ? kin_imus.size() : 0);

    // =================== Stage-2: 整帧 IESKF (可选) ===================
    LidarUpdateDiagnostics final_diagnostics = first_diagnostics;
    if (two_step_lidar_eskf_) {
        diagnostic.second_step_executed = true;
        p2p_total = 0;
        ndt_total = 0;
        std::fill(match_types->begin(), match_types->end(), LidarMatchType::Unused);   // 重置匹配类型：以 Stage-2 结果为准

        // [Backpropagate] 把 Stage-1 得到的世界系点云用"最终位姿"打回机体系
        Timer::measure("Backpropagate", [&, this]() { this->backpropagate(*gauss_cloud_world, *gauss_cloud_body); });

        // [IESKF] 对整帧点云反复重线性化直到收敛
        Timer::measure("State predict/update - 2nd", [&, this]() {
            final_diagnostics =
                this->predictUpdateCloud(*gauss_cloud_body, *gauss_cloud_world, match_types.get());
            p2p_total = final_diagnostics.p2p_count;
            ndt_total = final_diagnostics.ndt_count;
            frame_had_lidar_update =
                frame_had_lidar_update || ((final_diagnostics.p2p_count + final_diagnostics.ndt_count) > 0);
        });
        diagnostic.p2p_second = p2p_total;
        diagnostic.ndt_second = ndt_total;
    }

    diagnostic.lidar_diagnostic_stage = two_step_lidar_eskf_ ? "second_final_iteration" : "first_aggregate";
    const ResidualSummary p2p_summary =
        summarizeResiduals(final_diagnostics.p2p_residuals, final_diagnostics.p2p_weight_sum);
    const ResidualSummary ndt_summary =
        summarizeResiduals(final_diagnostics.ndt_residuals, final_diagnostics.ndt_weight_sum);
    diagnostic.p2p_residual_median = p2p_summary.median;
    diagnostic.p2p_residual_p95 = p2p_summary.p95;
    diagnostic.p2p_weight_mean = p2p_summary.weight_mean;
    diagnostic.p2p_nonfinite_count = final_diagnostics.p2p_nonfinite_count;
    diagnostic.ndt_residual_median = ndt_summary.median;
    diagnostic.ndt_residual_p95 = ndt_summary.p95;
    diagnostic.ndt_weight_mean = ndt_summary.weight_mean;
    diagnostic.ndt_nonfinite_count = final_diagnostics.ndt_nonfinite_count;
    diagnostic.information_min_eigenvalue = informationMinEigenvalue(final_diagnostics.information);
    diagnostic.ieskf_iterations_used = final_diagnostics.ieskf_iterations_used;
    diagnostic.ieskf_converged = final_diagnostics.ieskf_converged;

    // =================== 地图更新 ===================
    // 把本帧最终的世界系点云插入高斯体素地图（更新每个体素的平面/NDT 统计）
    Timer::measure("Voxel map update", [&, this]() { gaussian_voxel_map_->insertPoints(*gauss_cloud_world); });

    // LO 模式下用最新位姿差分刷新 CV 预测输入，供下一帧使用
    if (common::usesLidarOnly(sensor_type_) && frame_had_lidar_update) { updateLoPredictInput(end_time); }

    // =================== 组装 ProcessResult ===================
    // 顺序：把 GaussCloud (含协方差) 输出成 pcl PointCloud → 世界系 → 机体系 → 激光系
    pcl_utils::GaussCloudToPclCloud(*gauss_cloud_world, cloud_down_world);
    // 世界系 → 机体系：左乘 (R_wb, t_wb)^{-1}
    pcl::transformPointCloud(*cloud_down_world, *cloud_down_body,
                             makeIsometry3d(eskf_->getRot(), eskf_->getPos()).inverse().matrix().cast<float>());
    // 机体系 → 激光系：左乘 (R_bl, t_bl)^{-1}
    pcl::transformPointCloud(*cloud_down_body, *cloud_down_lidar,
                             makeIsometry3d(ext_rot_, ext_t_).inverse().matrix().cast<float>());
    result.valid = true;
    result.total_pts_size = cloud_raw->size();
    result.down_pts_size = cloud_down_lidar->size();
    result.p2p_count = p2p_total;
    result.ndt_count = ndt_total;
    result.success_pts_size = p2p_total + ndt_total;
    result.cloud_world = cloud_down_world;
    result.cloud_lidar = cloud_down_lidar;
    result.cloud_body = cloud_down_body;
    result.match_types = match_types;
    emitFrameDiagnostic("ok");
    diagnostic_previous_rot_ = eskf_->getRot();
    diagnostic_previous_pos_ = eskf_->getPos();
    diagnostic_has_previous_pose_ = true;
    return result;
}

}  // namespace legkilo
