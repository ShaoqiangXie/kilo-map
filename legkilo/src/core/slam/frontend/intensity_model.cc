// SPDX-License-Identifier: MIT
#include "core/slam/frontend/intensity_model.h"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace legkilo {
void IntensityConfig::validate() const {
    const auto positive = [](double x) { return std::isfinite(x) && x > 0.0; };
    if (!positive(scale) || min_points < 4 || max_points < min_points ||
        !positive(min_spatial_eigenvalue) || !positive(min_gradient) ||
        !positive(max_gradient) || max_gradient < min_gradient || !positive(max_fit_rmse) ||
        !std::isfinite(min_fit_r2) || min_fit_r2 < 0.0 || min_fit_r2 > 1.0 ||
        !positive(measurement_sigma) || !positive(max_normal_distance) ||
        !positive(max_mahalanobis) || !positive(residual_gate) ||
        !positive(weight) || weight > 1.0 || !positive(huber_delta)) {
        throw std::invalid_argument("Invalid intensity residual configuration (see INTENSITY_RESIDUAL_CN.md)");
    }
}

bool IntensityConfig::normalize(double raw, double& normalized) const {
    if (!std::isfinite(raw) || raw < 0.0 || raw > scale) return false;
    normalized = raw / scale;
    return std::isfinite(normalized);
}

void IntensityModel::addPoint(const Eigen::Vector3d& p, double intensity, std::size_t max_points) {
    if (count_ >= max_points || !p.allFinite() || !std::isfinite(intensity)) return;
    ++count_;
    const Eigen::Vector3d dp = p - mean_p_;
    const double di = intensity - mean_i_;
    mean_p_ += dp / static_cast<double>(count_);
    mean_i_ += di / static_cast<double>(count_);
    scatter_p_.noalias() += dp * (p - mean_p_).transpose();
    scatter_pi_.noalias() += dp * (intensity - mean_i_);
    scatter_i_ += di * (intensity - mean_i_);
    valid_ = false;
}

void IntensityModel::fit(const Eigen::Vector3d& normal, const IntensityConfig& config) {
    valid_ = false;
    if (count_ < config.min_points || !normal.allFinite() || normal.norm() < 0.5) return;
    const Eigen::Vector3d n = normal.normalized();
    Eigen::Matrix<double, 3, 2> tangent;
    tangent.col(0) = n.unitOrthogonal();
    tangent.col(1) = n.cross(tangent.col(0));
    const double count = static_cast<double>(count_);
    const Eigen::Matrix2d cov = tangent.transpose() * scatter_p_ * tangent / count;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(0.5 * (cov + cov.transpose()));
    if (eig.info() != Eigen::Success || !eig.eigenvalues().allFinite() ||
        eig.eigenvalues().minCoeff() < config.min_spatial_eigenvalue) return;
    const Eigen::Matrix2d inverse =
        eig.eigenvectors() * eig.eigenvalues().cwiseInverse().asDiagonal() * eig.eigenvectors().transpose();
    support_inverse_ = tangent * inverse * tangent.transpose();
    gradient_ = support_inverse_ * scatter_pi_ / count;
    const double norm = gradient_.norm();
    if (!gradient_.allFinite() || norm < config.min_gradient || norm > config.max_gradient) return;
    const double sse = std::max(0.0, scatter_i_ - 2.0 * gradient_.dot(scatter_pi_) +
                                      gradient_.dot(scatter_p_ * gradient_));
    fit_variance_ = sse / (count - 3.0);  // intercept + two tangent coefficients
    const double r2 = scatter_i_ > 1e-12 ? 1.0 - sse / scatter_i_ : 0.0;
    valid_ = std::isfinite(fit_variance_) && std::sqrt(fit_variance_) <= config.max_fit_rmse &&
             r2 >= config.min_fit_r2;
}

double IntensityModel::supportDistance2(const Eigen::Vector3d& p) const {
    const Eigen::Vector3d d = p - mean_p_;
    return d.dot(support_inverse_ * d);
}

double IntensityModel::predictionVariance(const Eigen::Vector3d& p, double measurement_variance) const {
    const double leverage = (1.0 + supportDistance2(p)) / static_cast<double>(count_);
    return fit_variance_ + std::max(fit_variance_, measurement_variance) * leverage;
}
}  // namespace legkilo
