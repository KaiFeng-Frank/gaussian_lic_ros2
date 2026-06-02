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

  // Analytic Jacobian of the per-pixel photometric residual w.r.t. the spline
  // rotation + position control knots. This is what makes the factor a real
  // tightly-coupled constraint (vs the old weak SE(3) prior). The image
  // gradient is obtained by central differences on the sampler (autodiff cannot
  // see through the image), then chained analytically:
  //   d_res[k]/d_knot = weight * gradI_k * d_uv_d_pC * d_pC_d_pose * d_pose_d_knot
  // with d_pC/dR(u) = R_ci^T [m]_x (m = R(u)^T (p_world - p(u))), d_pC/dp(u) =
  // -R_wc^T, and d_pose/d_knot the validated SplitSplineView knot Jacobians
  // (right-perturbation convention). Mirrors Coco-LIC PhotometricFactorNURBS.
  struct PhotometricKnotJacobian
  {
    bool valid{false};
    std::vector<double> residuals;
    std::array<Eigen::Matrix<double, Eigen::Dynamic, 3>, N> d_res_d_rot_knot;
    std::array<Eigen::Matrix<double, Eigen::Dynamic, 3>, N> d_res_d_pos_knot;
  };

  bool evaluate_with_knot_jacobian(
    const PhotometricFactorState & state,
    PhotometricKnotJacobian & out,
    double image_gradient_step_px = 1.0) const
  {
    const int m_residuals = patch_size_ * patch_size_;
    out.valid = false;
    out.residuals.assign(m_residuals, 0.0);
    for (int i = 0; i < N; ++i) {
      out.d_res_d_rot_knot[i] = Eigen::Matrix<double, Eigen::Dynamic, 3>::Zero(m_residuals, 3);
      out.d_res_d_pos_knot[i] = Eigen::Matrix<double, Eigen::Dynamic, 3>::Zero(m_residuals, 3);
    }

    SplitSplineView<N> view(state.rotation_knots, state.position_knots, u_, inv_dt_s_);
    typename SplitSplineView<N>::So3KnotJacobian rot_jac;
    typename SplitSplineView<N>::R3KnotJacobian pos_jac;
    const Eigen::Quaterniond q_b_w = view.rotation_with_knot_jacobian(rot_jac);
    const Eigen::Vector3d p_b_w = view.position_with_knot_jacobian(pos_jac);

    const Eigen::Quaterniond q_c_i = extrinsics_.q_camera_to_imu;
    const Eigen::Vector3d p_c_i = extrinsics_.p_camera_in_imu;
    const Eigen::Matrix3d r_c_i = q_c_i.toRotationMatrix();
    const Eigen::Quaterniond q_w_c = (q_b_w * q_c_i).normalized();
    const Eigen::Matrix3d r_w_c = q_w_c.toRotationMatrix();

    const Eigen::Vector3d m_body = q_b_w.inverse() * (point_world_ - p_b_w);
    const Eigen::Vector3d p_camera = q_c_i.inverse() * (m_body - p_c_i);
    if (p_camera.z() <= 1.0e-6) {
      return false;
    }
    const double z_inv = 1.0 / p_camera.z();
    const double u_center = intrinsics_.fx * p_camera.x() * z_inv + intrinsics_.cx;
    const double v_center = intrinsics_.fy * p_camera.y() * z_inv + intrinsics_.cy;

    Eigen::Matrix<double, 2, 3> d_uv_d_pc;
    d_uv_d_pc <<
      intrinsics_.fx * z_inv, 0.0, -intrinsics_.fx * p_camera.x() * z_inv * z_inv,
      0.0, intrinsics_.fy * z_inv, -intrinsics_.fy * p_camera.y() * z_inv * z_inv;

    const Eigen::Matrix3d d_pc_d_drot = r_c_i.transpose() * skew(m_body);
    const Eigen::Matrix3d d_pc_d_dpos = -r_w_c.transpose();
    const Eigen::Matrix<double, 2, 3> d_uv_d_drot = d_uv_d_pc * d_pc_d_drot;
    const Eigen::Matrix<double, 2, 3> d_uv_d_dpos = d_uv_d_pc * d_pc_d_dpos;

    std::array<Eigen::Matrix<double, 2, 3>, N> d_uv_d_rot_knot;
    std::array<Eigen::Matrix<double, 2, 3>, N> d_uv_d_pos_knot;
    for (int i = 0; i < N; ++i) {
      d_uv_d_rot_knot[i] = d_uv_d_drot * rot_jac.d_val_d_knot[i];
      d_uv_d_pos_knot[i] = pos_jac.d_val_d_knot[i] * d_uv_d_dpos;
    }

    out.valid = true;
    const double h = image_gradient_step_px;
    int idx = 0;
    for (int dy = -patch_size_half_; dy < patch_size_half_; ++dy) {
      for (int dx = -patch_size_half_; dx < patch_size_half_; ++dx) {
        const double u = u_center + dx;
        const double v = v_center + dy;
        bool valid_pix = false;
        bool vu1 = false, vu2 = false, vv1 = false, vv2 = false;
        const double intensity = sampler_(u, v, valid_pix);
        const double reference =
          idx < static_cast<int>(reference_patch_.size()) ? reference_patch_[idx] : 0.0;
        out.residuals[idx] = weight_ * (intensity - reference);

        const double iu1 = sampler_(u + h, v, vu1);
        const double iu2 = sampler_(u - h, v, vu2);
        const double iv1 = sampler_(u, v + h, vv1);
        const double iv2 = sampler_(u, v - h, vv2);
        if (!valid_pix || !vu1 || !vu2 || !vv1 || !vv2) {
          out.valid = false;
        }
        Eigen::RowVector2d grad_i((iu1 - iu2) / (2.0 * h), (iv1 - iv2) / (2.0 * h));
        const Eigen::RowVector2d w_grad = weight_ * grad_i;
        for (int i = 0; i < N; ++i) {
          out.d_res_d_rot_knot[i].row(idx) = w_grad * d_uv_d_rot_knot[i];
          out.d_res_d_pos_knot[i].row(idx) = w_grad * d_uv_d_pos_knot[i];
        }
        ++idx;
      }
    }
    return out.valid;
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
