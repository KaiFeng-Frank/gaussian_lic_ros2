// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_tracking/visual_factor.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

namespace gaussian_lic_tracking
{
namespace
{
bool intrinsics_are_finite(const VisualCameraIntrinsics & intrinsics)
{
  return std::isfinite(intrinsics.fx) && std::isfinite(intrinsics.fy) &&
         std::isfinite(intrinsics.cx) && std::isfinite(intrinsics.cy);
}

Eigen::Matrix3d skew_symmetric(const Eigen::Vector3d & value)
{
  Eigen::Matrix3d skew;
  skew << 0.0, -value.z(), value.y(),
    value.z(), 0.0, -value.x(),
    -value.y(), value.x(), 0.0;
  return skew;
}

double parabolic_subpixel_offset(
  const double minus_cost,
  const double center_cost,
  const double plus_cost)
{
  const double denominator = minus_cost - 2.0 * center_cost + plus_cost;
  if (std::abs(denominator) < 1.0e-12) {
    return 0.0;
  }
  const double offset = 0.5 * (minus_cost - plus_cost) / denominator;
  return std::clamp(offset, -0.5, 0.5);
}
}  // namespace

VisualFactor::VisualFactor(const size_t max_pixels)
{
  set_max_pixels(max_pixels);
}

void VisualFactor::set_max_pixels(const size_t max_pixels)
{
  if (max_pixels == 0U) {
    throw std::runtime_error("visual factor max_pixels must be positive");
  }
  max_pixels_ = max_pixels;
}

VisualResidual VisualFactor::evaluate(const VisualFrame & reference, const VisualFrame & candidate) const
{
  return evaluate_shifted(reference, candidate, 0, 0);
}

VisualSe3PhotometricJacobian linearize_se3_photometric_pixel(
  const VisualCameraIntrinsics & intrinsics,
  const Eigen::Vector3d & point_camera,
  const Eigen::Vector2d & image_gradient)
{
  VisualSe3PhotometricJacobian output;
  if (!point_camera.allFinite() || !image_gradient.allFinite() ||
    !intrinsics_are_finite(intrinsics) ||
    intrinsics.fx <= 0.0 || intrinsics.fy <= 0.0 ||
    std::abs(point_camera.z()) < 1.0e-12)
  {
    return output;
  }

  const double x = point_camera.x();
  const double y = point_camera.y();
  const double z = point_camera.z();
  const double z_inv = 1.0 / z;
  const double z_inv_sq = z_inv * z_inv;

  // Tangent order is [rx, ry, rz, tx, ty, tz] for a left camera-frame SE3 perturbation.
  output.pixel_jacobian(0, 0) = -intrinsics.fx * x * y * z_inv_sq;
  output.pixel_jacobian(0, 1) = intrinsics.fx * (1.0 + x * x * z_inv_sq);
  output.pixel_jacobian(0, 2) = -intrinsics.fx * y * z_inv;
  output.pixel_jacobian(0, 3) = intrinsics.fx * z_inv;
  output.pixel_jacobian(0, 4) = 0.0;
  output.pixel_jacobian(0, 5) = -intrinsics.fx * x * z_inv_sq;

  output.pixel_jacobian(1, 0) = -intrinsics.fy * (1.0 + y * y * z_inv_sq);
  output.pixel_jacobian(1, 1) = intrinsics.fy * x * y * z_inv_sq;
  output.pixel_jacobian(1, 2) = intrinsics.fy * x * z_inv;
  output.pixel_jacobian(1, 3) = 0.0;
  output.pixel_jacobian(1, 4) = intrinsics.fy * z_inv;
  output.pixel_jacobian(1, 5) = -intrinsics.fy * y * z_inv_sq;

  output.intensity_jacobian = image_gradient.transpose() * output.pixel_jacobian;
  output.valid = output.pixel_jacobian.allFinite() && output.intensity_jacobian.allFinite();
  return output;
}

VisualSe3PhotometricLinearization linearize_se3_photometric_samples(
  const VisualCameraIntrinsics & intrinsics,
  const std::vector<VisualSe3PhotometricSample> & samples)
{
  VisualSe3PhotometricLinearization output;
  for (const auto & sample : samples) {
    if (!std::isfinite(sample.weight) || sample.weight <= 0.0 ||
      sample.weight > 1.0 ||
      !std::isfinite(sample.residual))
    {
      continue;
    }
    const auto jacobian = linearize_se3_photometric_pixel(
      intrinsics, sample.point_camera, sample.image_gradient);
    if (!jacobian.valid) {
      continue;
    }
    const double weight = sample.weight;
    output.hessian.noalias() += weight * jacobian.intensity_jacobian.transpose() *
      jacobian.intensity_jacobian;
    output.rhs.noalias() -= weight * jacobian.intensity_jacobian.transpose() * sample.residual;
    output.cost += 0.5 * weight * sample.residual * sample.residual;
    ++output.sample_count;
  }
  if (output.sample_count == 0U || !output.hessian.allFinite() || !output.rhs.allFinite()) {
    return output;
  }
  const Eigen::LDLT<Eigen::Matrix<double, 6, 6>> ldlt(output.hessian);
  if (ldlt.info() != Eigen::Success) {
    return output;
  }
  output.gauss_newton_step = ldlt.solve(output.rhs);
  output.valid = output.gauss_newton_step.allFinite();
  return output;
}

Eigen::Matrix<double, 6, 1> transform_camera_delta_to_body(
  const Eigen::Quaterniond & q_body_camera,
  const Eigen::Vector3d & p_body_camera,
  const Eigen::Matrix<double, 6, 1> & camera_delta)
{
  Eigen::Matrix<double, 6, 1> body_delta = Eigen::Matrix<double, 6, 1>::Zero();
  if (!camera_delta.allFinite()) {
    return body_delta;
  }
  const Eigen::Matrix<double, 6, 6> adjoint =
    camera_delta_to_body_adjoint(q_body_camera, p_body_camera);
  if (!adjoint.allFinite() || adjoint.norm() <= std::numeric_limits<double>::epsilon()) {
    return body_delta;
  }
  body_delta = adjoint * camera_delta;
  return body_delta.allFinite() ? body_delta : Eigen::Matrix<double, 6, 1>::Zero();
}

Eigen::Matrix<double, 6, 6> camera_delta_to_body_adjoint(
  const Eigen::Quaterniond & q_body_camera,
  const Eigen::Vector3d & p_body_camera)
{
  Eigen::Matrix<double, 6, 6> adjoint = Eigen::Matrix<double, 6, 6>::Zero();
  if (!q_body_camera.coeffs().allFinite() ||
    q_body_camera.norm() <= std::numeric_limits<double>::epsilon() ||
    !p_body_camera.allFinite())
  {
    return adjoint;
  }
  const Eigen::Matrix3d r_body_camera = q_body_camera.normalized().toRotationMatrix();
  adjoint.template block<3, 3>(0, 0) = r_body_camera;
  adjoint.template block<3, 3>(3, 0) = skew_symmetric(p_body_camera) * r_body_camera;
  adjoint.template block<3, 3>(3, 3) = r_body_camera;
  return adjoint;
}

Eigen::Matrix<double, 6, 6> transform_camera_information_to_body(
  const Eigen::Quaterniond & q_body_camera,
  const Eigen::Vector3d & p_body_camera,
  const Eigen::Matrix<double, 6, 6> & camera_hessian)
{
  Eigen::Matrix<double, 6, 6> body_hessian = Eigen::Matrix<double, 6, 6>::Zero();
  if (!camera_hessian.allFinite()) {
    return body_hessian;
  }
  const Eigen::Matrix<double, 6, 6> adjoint =
    camera_delta_to_body_adjoint(q_body_camera, p_body_camera);
  if (!adjoint.allFinite() || adjoint.norm() <= std::numeric_limits<double>::epsilon()) {
    return body_hessian;
  }
  const Eigen::Matrix<double, 6, 6> inverse_adjoint = adjoint.inverse();
  body_hessian = inverse_adjoint.transpose() * camera_hessian * inverse_adjoint;
  body_hessian = 0.5 * (body_hessian + body_hessian.transpose());
  return body_hessian.allFinite() ? body_hessian : Eigen::Matrix<double, 6, 6>::Zero();
}

Eigen::Matrix<double, 6, 6> sqrt_information_from_hessian(
  const Eigen::Matrix<double, 6, 6> & hessian)
{
  Eigen::Matrix<double, 6, 6> sqrt_information = Eigen::Matrix<double, 6, 6>::Zero();
  if (!hessian.allFinite()) {
    return sqrt_information;
  }
  const Eigen::Matrix<double, 6, 6> symmetric = 0.5 * (hessian + hessian.transpose());
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(symmetric);
  if (solver.info() != Eigen::Success) {
    return sqrt_information;
  }
  Eigen::Matrix<double, 6, 1> sqrt_values;
  for (Eigen::Index index = 0; index < solver.eigenvalues().size(); ++index) {
    sqrt_values[index] = std::sqrt(std::max(0.0, solver.eigenvalues()[index]));
  }
  sqrt_information = sqrt_values.asDiagonal() * solver.eigenvectors().transpose();
  return sqrt_information.allFinite() ? sqrt_information : Eigen::Matrix<double, 6, 6>::Zero();
}

VisualPhotometricLinearization VisualFactor::linearize_translation(
  const VisualFrame & reference,
  const VisualFrame & candidate) const
{
  VisualPhotometricLinearization output;
  if (reference.width < 3U || reference.height < 3U ||
    reference.width != candidate.width || reference.height != candidate.height)
  {
    return output;
  }
  const size_t pixel_count = reference.width * reference.height;
  if (reference.gray.size() != pixel_count || candidate.gray.size() != pixel_count) {
    return output;
  }

  const size_t stride = pixel_count > max_pixels_
    ? static_cast<size_t>(std::ceil(static_cast<double>(pixel_count) / static_cast<double>(max_pixels_)))
    : 1U;
  for (size_t index = 0; index < pixel_count; index += stride) {
    const size_t x = index % reference.width;
    const size_t y = index / reference.width;
    if (x == 0U || y == 0U || x + 1U >= reference.width || y + 1U >= reference.height) {
      continue;
    }
    const auto at = [&candidate](const size_t px, const size_t py) {
        return static_cast<double>(candidate.gray[py * candidate.width + px]);
      };
    const double residual = static_cast<double>(candidate.gray[index] - reference.gray[index]);
    const double left = at(x - 1U, y);
    const double right = at(x + 1U, y);
    const double up = at(x, y - 1U);
    const double down = at(x, y + 1U);
    if (!std::isfinite(residual) || !std::isfinite(left) || !std::isfinite(right) ||
      !std::isfinite(up) || !std::isfinite(down))
    {
      continue;
    }
    const Eigen::Vector2d jacobian{
      0.5 * (right - left),
      0.5 * (down - up)};
    output.hessian += jacobian * jacobian.transpose();
    output.rhs.noalias() -= jacobian * residual;
    output.cost += 0.5 * residual * residual;
    ++output.compared_pixels;
  }
  if (output.compared_pixels == 0U || !output.hessian.allFinite() || !output.rhs.allFinite()) {
    return output;
  }
  const Eigen::LDLT<Eigen::Matrix2d> ldlt(output.hessian);
  if (ldlt.info() != Eigen::Success) {
    return output;
  }
  output.gauss_newton_step = ldlt.solve(output.rhs);
  output.valid = output.gauss_newton_step.allFinite();
  return output;
}

VisualResidual VisualFactor::evaluate_shifted(
  const VisualFrame & reference,
  const VisualFrame & candidate,
  const int dx,
  const int dy) const
{
  VisualResidual residual;
  if (reference.width == 0U || reference.height == 0U ||
    reference.width != candidate.width || reference.height != candidate.height)
  {
    return residual;
  }
  const int width = static_cast<int>(reference.width);
  const int height = static_cast<int>(reference.height);
  if (std::abs(dx) >= width || std::abs(dy) >= height) {
    return residual;
  }
  const size_t pixel_count = reference.width * reference.height;
  if (reference.gray.size() != pixel_count || candidate.gray.size() != pixel_count) {
    return residual;
  }

  const size_t stride = pixel_count > max_pixels_
    ? static_cast<size_t>(std::ceil(static_cast<double>(pixel_count) / static_cast<double>(max_pixels_)))
    : 1U;
  double abs_sum = 0.0;
  double sq_sum = 0.0;
  size_t compared = 0U;
  for (size_t index = 0; index < pixel_count; index += stride) {
    const int x = static_cast<int>(index % reference.width);
    const int y = static_cast<int>(index / reference.width);
    const int candidate_x = x + dx;
    const int candidate_y = y + dy;
    if (candidate_x < 0 || candidate_x >= width || candidate_y < 0 || candidate_y >= height) {
      continue;
    }
    const size_t candidate_index =
      static_cast<size_t>(candidate_y) * candidate.width + static_cast<size_t>(candidate_x);
    const double diff = static_cast<double>(candidate.gray[candidate_index] - reference.gray[index]);
    if (!std::isfinite(diff)) {
      continue;
    }
    abs_sum += std::abs(diff);
    sq_sum += diff * diff;
    ++compared;
  }
  if (compared == 0U) {
    return residual;
  }
  residual.valid = true;
  residual.compared_pixels = compared;
  residual.mean_abs_error = abs_sum / static_cast<double>(compared);
  residual.rmse = std::sqrt(sq_sum / static_cast<double>(compared));
  return residual;
}

VisualAlignment VisualFactor::estimate_translation(
  const VisualFrame & reference,
  const VisualFrame & candidate,
  const int max_shift_px) const
{
  VisualAlignment best;
  if (max_shift_px < 0) {
    return best;
  }
  for (int dy = -max_shift_px; dy <= max_shift_px; ++dy) {
    for (int dx = -max_shift_px; dx <= max_shift_px; ++dx) {
      const auto residual = evaluate_shifted(reference, candidate, dx, dy);
      if (!residual.valid) {
        continue;
      }
      if (!best.valid || residual.rmse < best.rmse) {
        best.valid = true;
        best.dx = dx;
        best.dy = dy;
        best.subpixel_dx = static_cast<double>(dx);
        best.subpixel_dy = static_cast<double>(dy);
        best.compared_pixels = residual.compared_pixels;
        best.mean_abs_error = residual.mean_abs_error;
        best.rmse = residual.rmse;
      }
    }
  }
  if (best.valid && best.rmse > 1.0e-9) {
    const auto x_minus = evaluate_shifted(reference, candidate, best.dx - 1, best.dy);
    const auto x_plus = evaluate_shifted(reference, candidate, best.dx + 1, best.dy);
    if (x_minus.valid && x_plus.valid) {
      best.subpixel_dx =
        static_cast<double>(best.dx) + parabolic_subpixel_offset(x_minus.rmse, best.rmse, x_plus.rmse);
    }
    const auto y_minus = evaluate_shifted(reference, candidate, best.dx, best.dy - 1);
    const auto y_plus = evaluate_shifted(reference, candidate, best.dx, best.dy + 1);
    if (y_minus.valid && y_plus.valid) {
      best.subpixel_dy =
        static_cast<double>(best.dy) + parabolic_subpixel_offset(y_minus.rmse, best.rmse, y_plus.rmse);
    }
  }
  return best;
}

}  // namespace gaussian_lic_tracking
