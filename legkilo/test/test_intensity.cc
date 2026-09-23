// SPDX-License-Identifier: MIT
#include <Eigen/Eigenvalues>
#include <glog/logging.h>
#include <tbb/parallel_for.h>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unistd.h>

#include "common/voxel_grid.hpp"
#include "core/slam/frontend/KILO.h"
#include "core/slam/frontend/eskf.h"
#include "core/slam/frontend/gaussian_voxel_map.h"

using namespace legkilo;
namespace {
void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}
void near(double actual, double expected, double tolerance, const std::string& message) {
    require(std::isfinite(actual) && std::abs(actual - expected) <= tolerance,
            message + ": actual=" + std::to_string(actual) + ", expected=" + std::to_string(expected));
}
GaussianVoxelMap::Config config(bool enabled = true) {
    GaussianVoxelMap::Config cfg;
    cfg.voxel_size = 1.0;
    cfg.nearby_type = NearByType::NEARBY26;
    cfg.intensity.enable = enabled;
    return cfg;
}
GaussCloud planeCloud(int side = 1, bool flat = false, bool one_direction = false) {
    GaussCloud cloud;
    for (int vx = 0; vx < side; ++vx) {
        for (int vy = 0; vy < side; ++vy) {
            for (int x = 0; x < 7; ++x) {
                for (int y = 0; y < 7; ++y) {
                    const double px = 0.2 + 0.1 * x, py = 0.2 + 0.1 * y;
                    const bool along_x = one_direction || (vx + vy) % 2 == 0;
                    const double intensity = flat ? 0.5 : 0.5 + 0.4 * ((along_x ? px : py) - 0.5);
                    cloud.emplace_back(Vec3D(vx + px, vy + py, 1.25), Mat3D::Identity() * 1e-6, 255 * intensity);
                }
            }
        }
    }
    return cloud;
}
KNearestRes<1> query(const GaussianVoxelMap& map, const Vec3D& body, const Mat3D& rotation,
                      const Vec3D& position, double intensity, const Mat3D& covariance = Mat3D::Zero()) {
    const Vec3D world = rotation * body + position;
    KNearestInput input(&body, &covariance, &world, &covariance, &rotation, intensity);
    KNearestRes<1> result;
    map.buildIntensityResidual(input, result);
    return result;
}
void testModel() {
    auto cfg = config();
    cfg.intensity.validate();
    for (int invalid_case = 0; invalid_case < 7; ++invalid_case) {
        auto bad = cfg.intensity;
        if (invalid_case == 0) bad.scale = 0;
        if (invalid_case == 1) bad.measurement_sigma = -1;
        if (invalid_case == 2) bad.min_points = 3;
        if (invalid_case == 3) bad.max_points = 2;
        if (invalid_case == 4) bad.weight = std::numeric_limits<double>::quiet_NaN();
        if (invalid_case == 5) bad.min_fit_r2 = 2;
        if (invalid_case == 6) bad.max_gradient = 0.001;
        bool threw = false;
        try { bad.validate(); } catch (const std::invalid_argument&) { threw = true; }
        require(threw, "invalid configuration must fail");
    }
    IntensityModel model;
    const Vec3D offset(1e6, -2e6, 3e6);
    for (const auto& point : planeCloud()) model.addPoint(point.pt + offset, point.intensity / 255, 200);
    model.fit(Vec3D::UnitZ(), cfg.intensity);
    require(model.valid(), "translated planar texture should fit");
    near((model.gradient() - Vec3D(0.4, 0, 0)).norm(), 0, 1e-8, "centered gradient");
    for (int i = 0; i < 500; ++i) model.addPoint(offset, 0.5, 49);
    require(model.count() == 49, "point budget bounded");
    IntensityModel line, sparse, constant, noisy, steep;
    int index = 0;
    for (const auto& point : planeCloud()) {
        line.addPoint(Vec3D(point.pt.x(), 0, 0), point.intensity / 255, 200);
        if (index < 5) sparse.addPoint(point.pt, point.intensity / 255, 200);
        constant.addPoint(point.pt, 0.5, 200);
        noisy.addPoint(point.pt, (index % 2) ? 0.0 : 1.0, 200);
        steep.addPoint(point.pt, 10 * point.pt.x(), 200);
        ++index;
    }
    for (auto* rejected : {&line, &sparse, &constant, &noisy, &steep}) {
        rejected->fit(Vec3D::UnitZ(), cfg.intensity);
        require(!rejected->valid(), "unreliable model rejected");
    }
    GaussianVoxelMap map(cfg), disabled(config(false)), flat(cfg);
    const auto cloud = planeCloud();
    map.insertPoints(cloud);
    disabled.insertPoints(cloud);
    flat.insertPoints(planeCloud(1, true));
    const Vec3D p(0.5, 0.5, 1.25);
    const Mat3D identity = Mat3D::Identity();
    auto q = query(map, p, identity, Vec3D::Zero(), 127.5);
    require(q.valid, "valid intensity correspondence");
    near(q.r(0), 0.0, 1e-12, "zero innovation");
    near(q.J(0, 5), 0.0, 1e-12, "no invented normal gradient");
    require(!query(disabled, p, identity, Vec3D::Zero(), 127.5).valid, "disabled residual");
    require(!query(flat, p, identity, Vec3D::Zero(), 127.5).valid, "flat texture fallback");
    for (double raw : {-1.0, 256.0, std::numeric_limits<double>::infinity(),
                       std::numeric_limits<double>::quiet_NaN(), 255.0}) {
        require(!query(map, p, identity, Vec3D::Zero(), raw).valid, "invalid intensity or outlier rejected");
    }
    const auto robust = query(map, p, identity, Vec3D::Zero(), 127.5 + 255 * 0.075);
    require(robust.valid && robust.R(0, 0) > q.R(0, 0), "Huber downweights moderate outlier");
    require(!query(map, p, identity, Vec3D(0, 0, 0.2), 127.5).valid, "normal distance gate");
    require(!query(map, p, identity, Vec3D(0.9, 0, 0), 127.5).valid, "tangent support gate");
    require(!query(map, Vec3D::Constant(std::numeric_limits<double>::quiet_NaN()), identity,
                    Vec3D::Zero(), 127.5).valid, "nonfinite coordinate gate");
    // Enabling intensity must leave the geometric residual and variance unchanged.
    const Mat3D cov = Mat3D::Identity() * 0.001;
    KNearestInput input(&p, &cov, &p, &cov, &identity, 127.5);
    KNearestRes<1> geom_on, geom_off;
    map.buildPoint2PlaneResidual(input, geom_on, {});
    disabled.buildPoint2PlaneResidual(input, geom_off, {});
    require(geom_on.valid && geom_off.valid, "geometric correspondence");
    near((geom_on.J - geom_off.J).norm(), 0, 1e-12, "geometry backward compatibility");
    CloudPtr pcl_cloud(new PointCloudType()), sampled(new PointCloudType());
    pcl_utils::GaussCloudToPclCloud(cloud, pcl_cloud);
    VoxelGrid sampler(0.01, SamplingMode::RandomPerVoxelQuota, 1000);
    sampler.filter(pcl_cloud, sampled);
    require(sampled->size() == cloud.size(), "downsampling metadata");
    for (size_t i = 0; i < cloud.size(); ++i) near(sampled->points[i].intensity, cloud[i].intensity, 1e-4, "intensity preserved");
    // const lookups are used concurrently by the production frontend.
    std::vector<double> parallel(1000);
    tbb::parallel_for(size_t(0), parallel.size(), [&](size_t i) {
        parallel[i] = query(map, p, identity, Vec3D::Zero(), 127.5).J(0, 3);
    });
    for (double value : parallel) near(value, q.J(0, 3), 1e-12, "parallel determinism");
    auto ndt_cfg = cfg;
    ndt_cfg.p2p_enable = false;
    ndt_cfg.ndt_enable = true;
    GaussianVoxelMap ndt_map(ndt_cfg);
    ndt_map.insertPoints(cloud);
    require(query(ndt_map, p, identity, Vec3D::Zero(), 127.5).valid, "NDT-only geometry supports intensity model");
    auto missing_cloud = cloud;
    for (auto& point : missing_cloud) point.intensity = std::numeric_limits<double>::quiet_NaN();
    GaussianVoxelMap missing_map(cfg);
    missing_map.insertPoints(missing_cloud);
    require(!query(missing_map, p, identity, Vec3D::Zero(), 127.5).valid, "map with missing intensity falls back");
    map.insertPoints(missing_cloud);
    require(query(map, p, identity, Vec3D::Zero(), 127.5).valid, "invalid intensity cannot poison valid model");
    auto scaled_cfg = cfg;
    scaled_cfg.intensity.scale = 1.0;
    GaussianVoxelMap scaled(scaled_cfg);
    auto normalized_cloud = cloud;
    for (auto& point : normalized_cloud) point.intensity /= 255;
    scaled.insertPoints(normalized_cloud);
    const auto scaled_query = query(scaled, p, identity, Vec3D::Zero(), 0.5);
    require(scaled_query.valid, "alternative sensor scale");
    near((q.J - scaled_query.J).norm(), 0, 1e-12, "fixed scale invariance");
}

void testJacobian() {
    GaussianVoxelMap map(config());
    map.insertPoints(planeCloud());
    const Mat3D rotation = Eigen::AngleAxisd(0.3, Vec3D(1, 2, 3).normalized()).toRotationMatrix();
    const Vec3D position(0.1, -0.2, 0.3), world(0.52, 0.48, 1.25);
    const Vec3D body = rotation.transpose() * (world - position);
    const double raw = 255 * (0.5 + 0.4 * (world.x() - 0.5));
    const auto base = query(map, body, rotation, position, raw);
    require(base.valid, "Jacobian correspondence");
    const double eps = 1e-6;
    double max_error = 0;
    for (int axis = 0; axis < 6; ++axis) {
        Mat3D plus_r = rotation, minus_r = rotation;
        Vec3D plus_t = position, minus_t = position;
        if (axis < 3) {
            plus_r = rotation * Eigen::AngleAxisd(eps, Vec3D::Unit(axis)).toRotationMatrix();
            minus_r = rotation * Eigen::AngleAxisd(-eps, Vec3D::Unit(axis)).toRotationMatrix();
        } else {
            plus_t(axis - 3) += eps;
            minus_t(axis - 3) -= eps;
        }
        const auto plus = query(map, body, plus_r, plus_t, raw);
        const auto minus = query(map, body, minus_r, minus_t, raw);
        require(plus.valid && minus.valid, "finite difference correspondence remains valid");
        const double numerical = -(plus.r(0) - minus.r(0)) / (2 * eps);
        max_error = std::max(max_error, std::abs(numerical - base.J(0, axis)));
    }
    require(max_error < 1e-6, "right perturbation Jacobian and innovation sign");
    std::cout << "jacobian_max_error=" << max_error << '\n';
}

struct RegistrationResult {
    double translation, rotation, min_eigenvalue;
    size_t intensity_count;
};
RegistrationResult registerPlane(bool enabled, bool flat, bool one_direction, double noise, unsigned seed) {
    GaussianVoxelMap map(config(enabled));
    auto scan = planeCloud(4, flat, one_direction);
    map.insertPoints(scan);
    std::mt19937 generator(seed);
    std::normal_distribution<double> gaussian(0, noise);
    for (auto& p : scan) p.intensity += 255 * gaussian(generator);
    ESKF filter(EskfProcessNoise{});
    filter.cov() = StateCov::Identity() * 0.2;
    filter.state().rot_ = Eigen::AngleAxisd(0.025, Vec3D::UnitZ()).toRotationMatrix();
    filter.state().pos_ = Vec3D(0.08, -0.06, 0.02);
    const State prior = filter.state();
    RegistrationResult result{};
    for (int iteration = 0; iteration < 10; ++iteration) {
        std::vector<KNearestRes<1>> rows;
        result.intensity_count = 0;
        for (const auto& p : scan) {
            const Vec3D world = filter.getRot() * p.pt + filter.getPos();
            const Mat3D world_cov = Mat3D::Identity() * 0.01;
            const Mat3D rot = filter.getRot();
            KNearestInput input(&p.pt, &p.cov, &world, &world_cov, &rot, p.intensity);
            KNearestRes<1> geo, photo;
            if (map.buildPoint2PlaneResidual(input, geo, {1.0, 1e-3, 10.0, 1e-2})) {
                geo.R(0, 0) = 1.0 / CauchyKernel(3.0).weight(geo.r.squaredNorm());
                rows.push_back(geo);
                if (map.buildIntensityResidual(input, photo)) {
                    rows.push_back(photo);
                    ++result.intensity_count;
                }
            }
        }
        require(!rows.empty(), "synthetic observation rows");
        ObsShared obs;
        obs.pt_h.resize(rows.size(), 6);
        obs.pt_z.resize(rows.size());
        obs.pt_R.resize(rows.size());
        for (size_t i = 0; i < rows.size(); ++i) {
            obs.pt_h.row(i) = rows[i].J;
            obs.pt_z(i) = rows[i].r(0);
            obs.pt_R(i) = rows[i].R(0, 0);
        }
        const Mat6D information = obs.pt_h.transpose() * obs.pt_R.cwiseInverse().asDiagonal() * obs.pt_h;
        result.min_eigenvalue = Eigen::SelfAdjointEigenSolver<Mat6D>(information).eigenvalues().minCoeff();
        if (filter.updateByCloud(obs, prior, 10, iteration)) break;
    }
    require(filter.cov().allFinite() && filter.getPos().allFinite(), "finite ESKF state and covariance");
    result.translation = filter.getPos().norm();
    result.rotation = Eigen::AngleAxisd(filter.getRot()).angle() * 180 / M_PI;
    return result;
}
void testRegistration() {
    std::cout << "scenario,translation_error_m,rotation_error_deg,min_information_eigenvalue,intensity_rows\n";
    const auto print = [](const char* name, const RegistrationResult& r) {
        std::cout << name << ',' << r.translation << ',' << r.rotation << ',' << r.min_eigenvalue << ',' << r.intensity_count << '\n';
    };
    const auto baseline = registerPlane(false, false, false, 0, 42);
    const auto fused = registerPlane(true, false, false, 0, 42);
    const auto flat_off = registerPlane(false, true, false, 0, 42);
    const auto flat_on = registerPlane(true, true, false, 0, 42);
    const auto one_direction = registerPlane(true, false, true, 0, 42);
    print("geometry_only", baseline);
    print("geometry_intensity", fused);
    print("flat_intensity_off", flat_off);
    print("flat_intensity_on", flat_on);
    print("one_gradient_direction", one_direction);
    require(baseline.translation > 0.09 && baseline.rotation > 1.0, "plane has tangent and yaw nullspace");
    require(fused.translation < 0.002 && fused.rotation < 0.02, "textured plane recovers pose");
    require(fused.min_eigenvalue > 1 && std::abs(baseline.min_eigenvalue) < 1e-7, "information rank restored");
    near(flat_off.translation, flat_on.translation, 1e-12, "flat texture matches disabled baseline");
    near(flat_off.rotation, flat_on.rotation, 1e-12, "flat rotational fallback");
    require(flat_on.intensity_count == 0 && one_direction.translation > 0.05, "no invented information without texture");
    double mean = 0, max_error = 0;
    for (unsigned seed = 0; seed < 10; ++seed) {
        const auto noisy = registerPlane(true, false, false, 0.01, seed);
        require(noisy.translation < 0.02 && noisy.rotation < 0.3, "noisy textured plane converges");
        mean += noisy.translation / 10;
        max_error = std::max(max_error, noisy.translation);
        print(("noise_seed_" + std::to_string(seed)).c_str(), noisy);
    }
    std::cout << "noisy_translation_mean_m=" << mean << ", max_m=" << max_error << '\n';
}

void testFrontend() {
    const auto tmp = std::filesystem::temp_directory_path() / ("legkilo_intensity_" + std::to_string(getpid()));
    std::filesystem::create_directories(tmp);
    const std::string result_folder = "intensity_test_" + std::to_string(getpid());
    const auto diagnostic_dir = std::filesystem::path(ROOT_DIR) / "result" / result_folder;
    for (bool two_step : {false, true}) {
        for (bool enabled : {false, true}) {
            for (bool flat : {false, true}) {
              for (bool ndt_only : {false, true}) {
                const auto yaml = tmp / "config.yaml";
                std::ofstream out(yaml);
                out << "sensor_type: LO\nlo_vel_process_cov: 20\nlo_imu_gyr_process_cov: 20\n"
                       "imu_acc_meas_noise: 0.1\nimu_gyr_meas_noise: 0.01\n"
                       "frontend_diagnostic_print: false\nfrontend_diagnostic_csv: true\n"
                       "init_type: 1\nextrinsic_T: [0, 0, 0]\nextrinsic_R: [1, 0, 0, 0, 1, 0, 0, 0, 1]\n"
                       "dept_err: 0.01\nbeam_err: 0.01\nvoxel_size: 1.0\nnearby_type: 2\n"
                       "voxel_grid_resolution: 0.001\nvoxel_grid_target_num: 10000\n"
                       "ieskf_max_iterations: 5\n"
                    << "temp_result_save_folder: " << result_folder << '\n'
                    << "p2plane_enable: " << (ndt_only ? "false" : "true") << '\n'
                    << "ndt_enable: " << (ndt_only ? "true" : "false") << '\n'
                    << "two_step_lidar_eskf: " << (two_step ? "true" : "false") << '\n'
                    << "intensity_enable: " << (enabled ? "true" : "false") << '\n';
                out.close();
                KILO frontend(yaml.string());
                size_t total_rows = 0;
                for (int frame = 0; frame < 4; ++frame) {
                    const auto scan = planeCloud(3, flat);
                    common::MeasGroup measurement;
                    measurement.lidar_scan_.cloud_.reset(new PointCloudType());
                    pcl_utils::GaussCloudToPclCloud(scan, measurement.lidar_scan_.cloud_);
                    for (auto& p : measurement.lidar_scan_.cloud_->points) {
                        p.curvature = 0.0;
                        p.x -= 0.005f * frame;  // small inter-frame motion
                    }
                    measurement.lidar_scan_.lidar_begin_time_ = 1.0 + 0.1 * frame;
                    measurement.lidar_scan_.lidar_end_time_ = 1.0 + 0.1 * frame;
                    const auto result = frontend.process(measurement);
                    require(result.valid && result.cloud_world, "full frontend frame processed");
                    require(frontend.getPosImu().allFinite(), "full frontend finite pose");
                    total_rows += result.intensity_count;
                    for (const auto& p : result.cloud_world->points) require(std::isfinite(p.intensity) && p.intensity > 80, "frontend output retains intensity");
                }
                require((total_rows > 0) == (enabled && !flat), "frontend residual scheduling and fallback");
                std::ifstream csv(diagnostic_dir / "frontend_diagnostics.csv");
                std::string line;
                std::getline(csv, line);
                const auto split = [](const std::string& text) {
                    std::vector<std::string> fields;
                    std::istringstream stream(text);
                    std::string field;
                    while (std::getline(stream, field, ',')) fields.push_back(field);
                    if (!text.empty() && text.back() == ',') fields.emplace_back();
                    return fields;
                };
                const auto header = split(line);
                std::map<std::string, size_t> columns;
                for (size_t i = 0; i < header.size(); ++i) columns[header[i]] = i;
                require(columns.count("intensity_count") && columns.count("geometry_information_min_eigenvalue"), "CSV includes new diagnostics");
                size_t csv_rows = 0, csv_intensity_count = 0;
                while (std::getline(csv, line)) {
                    const auto fields = split(line);
                    require(fields.size() == header.size(), "CSV column count matches header");
                    csv_intensity_count += std::stoul(fields[columns.at("intensity_count")]);
                    if (fields[0] != "initialized") {
                        const double geometry = std::stod(fields[columns.at("geometry_information_min_eigenvalue")]);
                        const double fused = std::stod(fields[columns.at("information_min_eigenvalue")]);
                        require(fused + 1e-6 >= geometry, "intensity information is positive semidefinite");
                    }
                    ++csv_rows;
                }
                require(csv_rows == 4 && csv_intensity_count == total_rows, "CSV records actual used observations");
                std::cout << "frontend two_step=" << two_step << " enabled=" << enabled << " flat=" << flat
                          << " ndt_only=" << ndt_only << " intensity_rows=" << total_rows << '\n';
              }
            }
        }
    }
    std::filesystem::remove_all(tmp);
    std::filesystem::remove_all(diagnostic_dir);
}
}  // namespace

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_minloglevel = 2;
    std::cout << std::setprecision(10);
    const std::map<std::string, std::function<void()>> tests{
        {"model", testModel}, {"jacobian", testJacobian}, {"registration", testRegistration}, {"frontend", testFrontend}};
    try {
        for (const auto& test : tests) {
            if (argc > 1 && argv[1] != test.first) continue;
            const auto start = std::chrono::steady_clock::now();
            test.second();
            std::cout << "PASS " << test.first << " ("
                      << std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() << " s)\n";
        }
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
