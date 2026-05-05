// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_tracking/visual_factor.hpp>

#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

#include <Eigen/Geometry>

namespace
{
Eigen::Vector2d project(
  const gaussian_lic_tracking::VisualCameraIntrinsics & intrinsics,
  const Eigen::Vector3d & point_camera)
{
  return {
    intrinsics.fx * point_camera.x() / point_camera.z() + intrinsics.cx,
    intrinsics.fy * point_camera.y() / point_camera.z() + intrinsics.cy};
}

Eigen::Vector3d perturb_point(const Eigen::Vector3d & point_camera, const Eigen::Matrix<double, 6, 1> & xi)
{
  const Eigen::Vector3d dtheta = xi.template segment<3>(0);
  const Eigen::Vector3d dt = xi.template segment<3>(3);
  const double angle = dtheta.norm();
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  if (angle > 1.0e-14) {
    rotation = Eigen::AngleAxisd(angle, dtheta / angle).toRotationMatrix();
  }
  return rotation * point_camera + dt;
}
}  // namespace

int main()
{
  const gaussian_lic_tracking::VisualCameraIntrinsics intrinsics{
    320.0, 310.0, 160.0, 120.0};
  const Eigen::Vector3d point_camera{0.23, -0.17, 2.4};
  const Eigen::Vector2d image_gradient{0.73, -0.41};
  const auto analytic = gaussian_lic_tracking::linearize_se3_photometric_pixel(
    intrinsics, point_camera, image_gradient);
  if (!analytic.valid) {
    std::cerr << "analytic SE3 photometric Jacobian is invalid\n";
    return 1;
  }

  const Eigen::Vector2d reference_uv = project(intrinsics, point_camera);
  Eigen::Matrix<double, 1, 6> numeric;
  constexpr double epsilon = 1.0e-7;
  for (int column = 0; column < 6; ++column) {
    Eigen::Matrix<double, 6, 1> delta = Eigen::Matrix<double, 6, 1>::Zero();
    delta[column] = epsilon;
    const Eigen::Vector2d plus_uv = project(intrinsics, perturb_point(point_camera, delta));
    const Eigen::Vector2d minus_uv = project(intrinsics, perturb_point(point_camera, -delta));
    const double plus_intensity = image_gradient.dot(plus_uv - reference_uv);
    const double minus_intensity = image_gradient.dot(minus_uv - reference_uv);
    numeric[column] = (plus_intensity - minus_intensity) / (2.0 * epsilon);
  }

  const double max_abs_error = (analytic.intensity_jacobian - numeric).cwiseAbs().maxCoeff();
  std::cout << "visual_se3_jacobian_probe analytic=" << analytic.intensity_jacobian
            << " numeric=" << numeric
            << " max_abs_error=" << max_abs_error << "\n";

  if (max_abs_error > 1.0e-5) {
    std::cerr << "SE3 photometric Jacobian does not match finite differences\n";
    return 1;
  }

  Eigen::Matrix<double, 6, 1> expected_step;
  expected_step << 0.001, -0.002, 0.0015, 0.01, -0.015, 0.02;
  std::vector<gaussian_lic_tracking::VisualSe3PhotometricSample> samples;
  for (int iy = 0; iy < 6; ++iy) {
    for (int ix = 0; ix < 6; ++ix) {
      gaussian_lic_tracking::VisualSe3PhotometricSample sample;
      sample.point_camera = Eigen::Vector3d{
        (static_cast<double>(ix) - 2.5) * 0.08,
        (static_cast<double>(iy) - 2.5) * 0.06,
        1.4 + 0.05 * static_cast<double>((ix + 2 * iy) % 5)};
      sample.image_gradient = Eigen::Vector2d{
        0.2 + 0.05 * static_cast<double>(ix),
        -0.18 + 0.04 * static_cast<double>(iy)};
      sample.weight = 0.5 + 0.1 * static_cast<double>((ix + iy) % 3);
      const auto sample_jacobian = gaussian_lic_tracking::linearize_se3_photometric_pixel(
        intrinsics, sample.point_camera, sample.image_gradient);
      sample.residual = -(sample_jacobian.intensity_jacobian * expected_step)(0);
      samples.push_back(sample);
    }
  }
  const auto linearization =
    gaussian_lic_tracking::linearize_se3_photometric_samples(intrinsics, samples);
  const double step_error = (linearization.gauss_newton_step - expected_step).norm();
  std::cout << " se3_samples=" << linearization.sample_count
            << " se3_step=" << linearization.gauss_newton_step.transpose()
            << " se3_step_error=" << step_error;
  if (!linearization.valid || linearization.sample_count != samples.size() ||
    step_error > 1.0e-6)
  {
    std::cerr << "SE3 photometric sample linearization failed to recover the synthetic step\n";
    return 1;
  }
  auto samples_with_bad_weight = samples;
  samples_with_bad_weight.front().weight = std::numeric_limits<double>::quiet_NaN();
  const auto filtered_linearization =
    gaussian_lic_tracking::linearize_se3_photometric_samples(intrinsics, samples_with_bad_weight);
  if (!filtered_linearization.valid || filtered_linearization.sample_count != samples.size() - 1U) {
    std::cerr << "SE3 photometric linearization failed to skip non-finite sample weights\n";
    return 1;
  }

  constexpr double kPi = 3.14159265358979323846;
  const Eigen::Quaterniond q_i_c(Eigen::AngleAxisd(0.5 * kPi, Eigen::Vector3d::UnitZ()));
  const Eigen::Vector3d p_i_c{1.0, 2.0, -0.5};
  Eigen::Matrix<double, 6, 1> camera_delta;
  camera_delta << 0.1, -0.2, 0.3, 0.4, -0.5, 0.6;
  const auto body_delta =
    gaussian_lic_tracking::transform_camera_delta_to_body(q_i_c, p_i_c, camera_delta);
  Eigen::Matrix<double, 6, 1> expected_body_delta;
  const Eigen::Vector3d expected_omega = q_i_c * camera_delta.segment<3>(0);
  expected_body_delta.segment<3>(0) = expected_omega;
  expected_body_delta.segment<3>(3) =
    p_i_c.cross(expected_omega) + q_i_c * camera_delta.segment<3>(3);
  const double adjoint_error = (body_delta - expected_body_delta).norm();
  std::cout << " camera_to_body_adjoint_error=" << adjoint_error;
  if (adjoint_error > 1.0e-12) {
    std::cerr << "camera-to-body SE3 delta adjoint is incorrect\n";
    return 1;
  }
  const auto invalid_body_delta = gaussian_lic_tracking::transform_camera_delta_to_body(
    Eigen::Quaterniond{0.0, 0.0, 0.0, 0.0}, p_i_c, camera_delta);
  if (invalid_body_delta.norm() != 0.0) {
    std::cerr << "invalid camera-to-body extrinsic should produce a zero delta\n";
    return 1;
  }

  const auto invalid = gaussian_lic_tracking::linearize_se3_photometric_pixel(
    intrinsics, Eigen::Vector3d{0.0, 0.0, 0.0}, image_gradient);
  if (invalid.valid) {
    std::cerr << "zero-depth point should not produce a valid Jacobian\n";
    return 1;
  }
  auto bad_intrinsics = intrinsics;
  bad_intrinsics.fx = std::numeric_limits<double>::quiet_NaN();
  const auto invalid_intrinsics = gaussian_lic_tracking::linearize_se3_photometric_pixel(
    bad_intrinsics, point_camera, image_gradient);
  if (invalid_intrinsics.valid) {
    std::cerr << "non-finite intrinsics should not produce a valid Jacobian\n";
    return 1;
  }

  std::cout << "visual_se3_jacobian_probe OK\n";
  return 0;
}
