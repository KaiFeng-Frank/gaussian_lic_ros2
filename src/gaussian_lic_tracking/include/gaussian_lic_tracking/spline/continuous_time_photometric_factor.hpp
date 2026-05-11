// SPDX-License-Identifier: GPL-3.0-or-later
//
// ROS2-native port of Coco-LIC's PhotometricFactorNURBS continuous-time
// photometric residual
// (external/Coco-LIC/src/odom/factor/analytic_diff/image_feature_factor.h).
//
// The factor reprojects a 3D world point through the spline-evaluated camera
// pose into the current image and compares a small (P x P) intensity patch
// against a reference patch.
//
//   p_C = T_w_c(t)^{-1} * v_point_world
//   uv  = K * (p_C / p_C.z())
//   residual[k] = current_intensity[k] - reference_patch[k]
//
// This file is image-library-independent: callers provide an
// `IntensitySampler` callable that returns a bilinearly-interpolated intensity
// for a given (u, v). Downstream code that already uses OpenCV mats can wrap
// `cv::Mat::at<float>(...)` in such a lambda.

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <functional>
#include <vector>

#include <gaussian_lic_tracking/spline/ceres_spline_helper.hpp>
#include <gaussian_lic_tracking/spline/so3_ops.hpp>
#include <gaussian_lic_tracking/spline/split_spline_view.hpp>

namespace gaussian_lic_tracking
{
namespace spline
{

using IntensitySampler = std::function<double(double u, double v, bool & valid)>;

struct CameraExtrinsics
{
  Eigen::Quaterniond q_camera_to_imu{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d p_camera_in_imu{Eigen::Vector3d::Zero()};
};

struct CameraIntrinsics
{
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
};

struct PhotometricFactorState
{
  std::array<Eigen::Quaterniond, kPositionSplineOrder> rotation_knots;
  std::array<Eigen::Vector3d, kPositionSplineOrder> position_knots;
};

class ContinuousTimePhotometricFactor
{
public:
  static constexpr int N = kPositionSplineOrder;

  ContinuousTimePhotometricFactor(
    double normalized_time,
    double inv_dt_s,
    const Eigen::Vector3d & point_world,
    int patch_size_half,
    const std::vector<double> & reference_patch,
    IntensitySampler sampler,
    const CameraIntrinsics & intrinsics,
    const CameraExtrinsics & extrinsics,
    double weight = 1.0)
  : u_(normalized_time),
    inv_dt_s_(inv_dt_s),
    point_world_(point_world),
    patch_size_half_(patch_size_half),
    patch_size_(2 * patch_size_half),
    reference_patch_(reference_patch),
    sampler_(std::move(sampler)),
    intrinsics_(intrinsics),
    extrinsics_(extrinsics),
    weight_(weight)
  {
  }

  bool project(
    const PhotometricFactorState & state,
    double & u_out,
    double & v_out,
    Eigen::Vector3d & point_camera_out) const
  {
    const SplitSplineView<N> view(state.rotation_knots, state.position_knots, u_, inv_dt_s_);
    const Eigen::Quaterniond q_b_w = view.rotation();
    const Eigen::Vector3d p_b_w = view.position_world();

    // T_w_c = T_w_b * T_b_c, so T_c_w = T_b_c^{-1} * T_w_b^{-1}.
    const Eigen::Quaterniond q_w_c = q_b_w * extrinsics_.q_camera_to_imu;
    const Eigen::Vector3d p_w_c = p_b_w + q_b_w * extrinsics_.p_camera_in_imu;
    point_camera_out = q_w_c.inverse() * (point_world_ - p_w_c);

    if (point_camera_out.z() <= 1.0e-6) {
      return false;
    }

    u_out = intrinsics_.fx * point_camera_out.x() / point_camera_out.z() + intrinsics_.cx;
    v_out = intrinsics_.fy * point_camera_out.y() / point_camera_out.z() + intrinsics_.cy;
    return true;
  }

  // Returns the per-pixel residual vector (length = patch_size_^2). Each
  // entry is `weight * (current_intensity - reference_intensity)`. If the
  // projection is invalid or any patch pixel falls outside the image, the
  // residual is zero and `valid_out` is set to false.
  std::vector<double> residual(
    const PhotometricFactorState & state,
    bool & valid_out) const
  {
    std::vector<double> residuals(patch_size_ * patch_size_, 0.0);
    valid_out = false;

    double u_center = 0.0;
    double v_center = 0.0;
    Eigen::Vector3d p_camera;
    if (!project(state, u_center, v_center, p_camera)) {
      return residuals;
    }

    valid_out = true;
    int residual_index = 0;
    for (int dy = -patch_size_half_; dy < patch_size_half_; ++dy) {
      for (int dx = -patch_size_half_; dx < patch_size_half_; ++dx) {
        const double u = u_center + dx;
        const double v = v_center + dy;
        bool pixel_valid = false;
        const double intensity = sampler_(u, v, pixel_valid);
        if (!pixel_valid) {
          valid_out = false;
        }
        const double reference =
          residual_index < static_cast<int>(reference_patch_.size())
          ? reference_patch_[residual_index]
          : 0.0;
        residuals[residual_index] = weight_ * (intensity - reference);
        ++residual_index;
      }
    }
    return residuals;
  }

  // Sum of squared per-pixel residuals — a scalar useful for sliding-window
  // optimizers that pack photometric error into a single residual block.
  double total_squared_residual(
    const PhotometricFactorState & state,
    bool & valid_out) const
  {
    const auto values = residual(state, valid_out);
    double accum = 0.0;
    for (const double r : values) {
      accum += r * r;
    }
    return accum;
  }

  int patch_size_half() const { return patch_size_half_; }
  int patch_size() const { return patch_size_; }
  double weight() const { return weight_; }
  const Eigen::Vector3d & point_world() const { return point_world_; }
  const CameraIntrinsics & intrinsics() const { return intrinsics_; }
  const CameraExtrinsics & extrinsics() const { return extrinsics_; }

private:
  double u_{0.0};
  double inv_dt_s_{1.0};
  Eigen::Vector3d point_world_{Eigen::Vector3d::Zero()};
  int patch_size_half_{1};
  int patch_size_{2};
  std::vector<double> reference_patch_{};
  IntensitySampler sampler_{};
  CameraIntrinsics intrinsics_{};
  CameraExtrinsics extrinsics_{};
  double weight_{1.0};
};

}  // namespace spline
}  // namespace gaussian_lic_tracking
