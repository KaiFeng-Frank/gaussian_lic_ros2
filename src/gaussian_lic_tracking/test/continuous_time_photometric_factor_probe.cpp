// SPDX-License-Identifier: GPL-3.0-or-later
//
// Verifies the continuous-time photometric factor projects a known world
// point through the spline-evaluated pose and computes a per-pixel residual
// that matches a closed-form synthetic image.

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gaussian_lic_tracking/spline/continuous_time_photometric_factor.hpp>

using gaussian_lic_tracking::spline::CameraExtrinsics;
using gaussian_lic_tracking::spline::CameraIntrinsics;
using gaussian_lic_tracking::spline::ContinuousTimePhotometricFactor;
using gaussian_lic_tracking::spline::IntensitySampler;
using gaussian_lic_tracking::spline::PhotometricFactorState;

namespace
{

constexpr int N = ContinuousTimePhotometricFactor::N;

PhotometricFactorState build_identity_state(const Eigen::Vector3d & translation)
{
  PhotometricFactorState state;
  for (int i = 0; i < N; ++i) {
    state.rotation_knots[i] = Eigen::Quaterniond::Identity();
    state.position_knots[i] = translation;
  }
  return state;
}

void check_projection_matches_pinhole()
{
  // Identity pose at the origin, identity camera->IMU. A world point at
  // (0, 0, 1) projects to the principal point. The synthetic image returns
  // its (u, v) coordinate so we can verify projection numerically.
  CameraIntrinsics intrinsics{500.0, 500.0, 320.0, 240.0};
  CameraExtrinsics extrinsics;
  PhotometricFactorState state = build_identity_state(Eigen::Vector3d::Zero());

  const Eigen::Vector3d point_world(0.0, 0.0, 1.0);
  IntensitySampler sampler =
    [](double u, double v, bool & valid) -> double {
      valid = true;
      return u + 1000.0 * v;
    };

  std::vector<double> reference_patch(4, 0.0);
  ContinuousTimePhotometricFactor factor(
    0.5, 1.0 / 0.05, point_world,
    /* patch_size_half = */ 1, reference_patch, sampler,
    intrinsics, extrinsics, 1.0);

  double u = 0.0;
  double v = 0.0;
  Eigen::Vector3d p_camera;
  if (!factor.project(state, u, v, p_camera)) {
    std::fprintf(stderr, "projection failed for in-front-of-camera point\n");
    std::exit(1);
  }
  if (std::abs(u - 320.0) > 1.0e-6 || std::abs(v - 240.0) > 1.0e-6) {
    std::fprintf(stderr, "projection mismatch: u=%.6f v=%.6f\n", u, v);
    std::exit(1);
  }
}

void check_projection_rejects_behind_camera()
{
  CameraIntrinsics intrinsics{500.0, 500.0, 320.0, 240.0};
  CameraExtrinsics extrinsics;
  PhotometricFactorState state = build_identity_state(Eigen::Vector3d::Zero());

  const Eigen::Vector3d point_world(0.0, 0.0, -1.0);
  IntensitySampler sampler =
    [](double, double, bool & valid) -> double {
      valid = true;
      return 0.0;
    };

  ContinuousTimePhotometricFactor factor(
    0.5, 1.0 / 0.05, point_world,
    /* patch_size_half = */ 1,
    /* reference = */ std::vector<double>(4, 0.0),
    sampler, intrinsics, extrinsics, 1.0);

  double u = 0.0;
  double v = 0.0;
  Eigen::Vector3d p_camera;
  if (factor.project(state, u, v, p_camera)) {
    std::fprintf(stderr, "projection accepted behind-camera point\n");
    std::exit(1);
  }
}

void check_residual_zero_when_patches_match()
{
  CameraIntrinsics intrinsics{500.0, 500.0, 320.0, 240.0};
  CameraExtrinsics extrinsics;
  PhotometricFactorState state = build_identity_state(Eigen::Vector3d::Zero());

  const Eigen::Vector3d point_world(0.0, 0.0, 1.0);
  // Constant image returns 42.0 everywhere.
  IntensitySampler sampler =
    [](double, double, bool & valid) -> double {
      valid = true;
      return 42.0;
    };

  const int patch_half = 2;
  const int patch_count = (2 * patch_half) * (2 * patch_half);
  std::vector<double> reference_patch(patch_count, 42.0);
  ContinuousTimePhotometricFactor factor(
    0.5, 1.0 / 0.05, point_world, patch_half, reference_patch, sampler,
    intrinsics, extrinsics, 1.0);

  bool valid = false;
  const auto r = factor.residual(state, valid);
  if (!valid) {
    std::fprintf(stderr, "residual marked invalid for valid synthetic projection\n");
    std::exit(1);
  }
  for (const double value : r) {
    if (std::abs(value) > 1.0e-9) {
      std::fprintf(stderr, "non-zero residual for matched patch: %.6g\n", value);
      std::exit(1);
    }
  }
}

void check_residual_scales_with_weight()
{
  CameraIntrinsics intrinsics{500.0, 500.0, 320.0, 240.0};
  CameraExtrinsics extrinsics;
  PhotometricFactorState state = build_identity_state(Eigen::Vector3d::Zero());

  const Eigen::Vector3d point_world(0.0, 0.0, 1.0);
  IntensitySampler sampler =
    [](double, double, bool & valid) -> double {
      valid = true;
      return 50.0;
    };

  const int patch_half = 1;
  const int patch_count = 4;
  std::vector<double> reference_patch(patch_count, 40.0);
  ContinuousTimePhotometricFactor factor(
    0.5, 1.0 / 0.05, point_world, patch_half, reference_patch, sampler,
    intrinsics, extrinsics, 0.5);

  bool valid = false;
  const auto r = factor.residual(state, valid);
  if (!valid) {
    std::fprintf(stderr, "residual invalid in scaling check\n");
    std::exit(1);
  }
  for (const double value : r) {
    if (std::abs(value - 5.0) > 1.0e-9) {
      std::fprintf(stderr, "residual scaling mismatch: %.6g vs expected 5.0\n", value);
      std::exit(1);
    }
  }

  bool _;
  const double squared = factor.total_squared_residual(state, _);
  if (std::abs(squared - 100.0) > 1.0e-9) {
    std::fprintf(stderr,
      "summed squared residual mismatch: %.6g vs expected 100.0\n", squared);
    std::exit(1);
  }
}

}  // namespace

int main()
{
  try {
    check_projection_matches_pinhole();
    check_projection_rejects_behind_camera();
    check_residual_zero_when_patches_match();
    check_residual_scales_with_weight();
  } catch (const std::exception & exception) {
    std::fprintf(stderr,
      "continuous_time_photometric_factor_probe exception: %s\n", exception.what());
    return 1;
  }
  std::printf("continuous_time_photometric_factor_probe ok\n");
  return 0;
}
