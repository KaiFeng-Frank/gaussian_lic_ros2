// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace gaussian_lic_tracking
{

struct VisualFrame
{
  int64_t stamp_ns{0};
  size_t width{0};
  size_t height{0};
  std::vector<float> gray;
};

struct VisualResidual
{
  bool valid{false};
  size_t compared_pixels{0};
  double mean_abs_error{0.0};
  double rmse{0.0};
};

struct VisualAlignment
{
  bool valid{false};
  int dx{0};
  int dy{0};
  double subpixel_dx{0.0};
  double subpixel_dy{0.0};
  size_t compared_pixels{0};
  double mean_abs_error{0.0};
  double rmse{0.0};
};

struct VisualPhotometricLinearization
{
  bool valid{false};
  size_t compared_pixels{0};
  double cost{0.0};
  Eigen::Matrix2d hessian{Eigen::Matrix2d::Zero()};
  Eigen::Vector2d rhs{Eigen::Vector2d::Zero()};
  Eigen::Vector2d gauss_newton_step{Eigen::Vector2d::Zero()};
};

struct VisualCameraIntrinsics
{
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
};

struct VisualSe3PhotometricJacobian
{
  bool valid{false};
  Eigen::Matrix<double, 2, 6> pixel_jacobian{Eigen::Matrix<double, 2, 6>::Zero()};
  Eigen::Matrix<double, 1, 6> intensity_jacobian{Eigen::Matrix<double, 1, 6>::Zero()};
};

struct VisualSe3PhotometricSample
{
  Eigen::Vector3d point_camera{Eigen::Vector3d::Zero()};
  Eigen::Vector2d image_gradient{Eigen::Vector2d::Zero()};
  double residual{0.0};
  double weight{1.0};
};

struct VisualSe3PhotometricLinearization
{
  bool valid{false};
  size_t sample_count{0};
  size_t hessian_rank{0};
  double cost{0.0};
  double hessian_min_singular_value{0.0};
  double hessian_max_singular_value{0.0};
  double hessian_condition_number{0.0};
  Eigen::Matrix<double, 6, 6> hessian{Eigen::Matrix<double, 6, 6>::Zero()};
  Eigen::Matrix<double, 6, 1> rhs{Eigen::Matrix<double, 6, 1>::Zero()};
  Eigen::Matrix<double, 6, 1> gauss_newton_step{Eigen::Matrix<double, 6, 1>::Zero()};
};

VisualSe3PhotometricJacobian linearize_se3_photometric_pixel(
  const VisualCameraIntrinsics & intrinsics,
  const Eigen::Vector3d & point_camera,
  const Eigen::Vector2d & image_gradient);

VisualSe3PhotometricLinearization linearize_se3_photometric_samples(
  const VisualCameraIntrinsics & intrinsics,
  const std::vector<VisualSe3PhotometricSample> & samples);

Eigen::Matrix<double, 6, 1> transform_camera_delta_to_body(
  const Eigen::Quaterniond & q_body_camera,
  const Eigen::Vector3d & p_body_camera,
  const Eigen::Matrix<double, 6, 1> & camera_delta);

Eigen::Matrix<double, 6, 6> camera_delta_to_body_adjoint(
  const Eigen::Quaterniond & q_body_camera,
  const Eigen::Vector3d & p_body_camera);

Eigen::Matrix<double, 6, 6> transform_camera_information_to_body(
  const Eigen::Quaterniond & q_body_camera,
  const Eigen::Vector3d & p_body_camera,
  const Eigen::Matrix<double, 6, 6> & camera_hessian);

Eigen::Matrix<double, 6, 6> sqrt_information_from_hessian(
  const Eigen::Matrix<double, 6, 6> & hessian);

class VisualFactor
{
public:
  explicit VisualFactor(size_t max_pixels = 200000);

  void set_max_pixels(size_t max_pixels);
  size_t max_pixels() const { return max_pixels_; }

  VisualResidual evaluate(const VisualFrame & reference, const VisualFrame & candidate) const;
  VisualPhotometricLinearization linearize_translation(
    const VisualFrame & reference,
    const VisualFrame & candidate) const;
  VisualAlignment estimate_translation(
    const VisualFrame & reference,
    const VisualFrame & candidate,
    int max_shift_px) const;

private:
  VisualResidual evaluate_shifted(
    const VisualFrame & reference,
    const VisualFrame & candidate,
    int dx,
    int dy) const;

  size_t max_pixels_{200000};
};

}  // namespace gaussian_lic_tracking
