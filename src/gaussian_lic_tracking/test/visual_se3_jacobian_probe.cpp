// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_tracking/visual_factor.hpp>

#include <cmath>
#include <iostream>

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
  const auto invalid = gaussian_lic_tracking::linearize_se3_photometric_pixel(
    intrinsics, Eigen::Vector3d{0.0, 0.0, 0.0}, image_gradient);
  if (invalid.valid) {
    std::cerr << "zero-depth point should not produce a valid Jacobian\n";
    return 1;
  }

  std::cout << "visual_se3_jacobian_probe OK\n";
  return 0;
}
