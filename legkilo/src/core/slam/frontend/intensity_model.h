// SPDX-License-Identifier: MIT
#ifndef LEGKILO_INTENSITY_MODEL_H
#define LEGKILO_INTENSITY_MODEL_H

#include <Eigen/Core>
#include <cstddef>

namespace legkilo {

// All intensity thresholds use I / scale, without per-frame normalization.
struct IntensityConfig {
    bool enable = false;
    double scale = 255.0;
    std::size_t min_points = 12;
    std::size_t max_points = 200;
    double min_spatial_eigenvalue = 1e-4;  // m^2, both tangent directions
    double min_gradient = 0.02;           // normalized intensity / m
    double max_gradient = 5.0;
    double max_fit_rmse = 0.04;
    double min_fit_r2 = 0.3;
    double measurement_sigma = 0.03;
    double max_normal_distance = 0.1;     // m
    double max_mahalanobis = 3.0;         // tangent support radius in stddev
    double residual_gate = 4.0;           // whitened innovation
    double weight = 0.2;                  // information multiplier
    double huber_delta = 1.5;             // whitened innovation

    void validate() const;
    bool normalize(double raw, double& normalized) const;
};

// O(1) centered sufficient statistics. No storage of historical point clouds.
class IntensityModel {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void addPoint(const Eigen::Vector3d& p, double intensity, std::size_t max_points);
    void fit(const Eigen::Vector3d& normal, const IntensityConfig& config);
    bool valid() const { return valid_; }
    std::size_t count() const { return count_; }
    const Eigen::Vector3d& center() const { return mean_p_; }
    const Eigen::Vector3d& gradient() const { return gradient_; }
    double predict(const Eigen::Vector3d& p) const { return mean_i_ + gradient_.dot(p - mean_p_); }
    double supportDistance2(const Eigen::Vector3d& p) const;
    double predictionVariance(const Eigen::Vector3d& p, double measurement_variance) const;

   private:
    std::size_t count_ = 0;
    Eigen::Vector3d mean_p_ = Eigen::Vector3d::Zero();
    double mean_i_ = 0.0;
    Eigen::Matrix3d scatter_p_ = Eigen::Matrix3d::Zero();
    Eigen::Vector3d scatter_pi_ = Eigen::Vector3d::Zero();
    double scatter_i_ = 0.0;
    Eigen::Vector3d gradient_ = Eigen::Vector3d::Zero();
    Eigen::Matrix3d support_inverse_ = Eigen::Matrix3d::Zero();
    double fit_variance_ = 0.0;
    bool valid_ = false;
};
}  // namespace legkilo
#endif
