// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_tracking/sliding_window_optimizer.hpp>

#include <cmath>
#include <exception>
#include <iostream>
#include <limits>

namespace
{
Eigen::Matrix<double, 15, 15> expected_state_delta_jacobian(
  const Eigen::Quaterniond & reference_q)
{
  Eigen::Matrix<double, 15, 15> jacobian = Eigen::Matrix<double, 15, 15>::Zero();
  jacobian.block<3, 3>(0, 0) = reference_q.normalized().inverse().toRotationMatrix();
  jacobian.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
  jacobian.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity();
  jacobian.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity();
  jacobian.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity();
  return jacobian;
}

template<typename Function>
bool expect_throw(const char * name, Function && function)
{
  try {
    function();
  } catch (const std::exception &) {
    return true;
  }
  std::cerr << name << " was not rejected\n";
  return false;
}
}  // namespace

int main()
{
  gaussian_lic_tracking::SlidingWindowOptimizer optimizer;

  gaussian_lic_tracking::SlidingWindowState state;
  state.stamp_ns = 100;
  state.q_w_i = Eigen::Quaterniond(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()));
  state.v_w_i = Eigen::Vector3d{0.2, -0.1, 0.05};
  state.p_w_i = Eigen::Vector3d{1.0, -2.0, 0.5};
  state.gyro_bias = Eigen::Vector3d{0.01, -0.02, 0.03};
  state.accel_bias = Eigen::Vector3d{-0.1, 0.05, 0.2};
  optimizer.add_or_update_state(state);

  gaussian_lic_tracking::SlidingWindowPosePrior pose_prior;
  pose_prior.stamp_ns = state.stamp_ns;
  pose_prior.q_w_i = state.q_w_i;
  pose_prior.p_w_i = state.p_w_i;
  pose_prior.rotation_weight = 4.0;
  pose_prior.translation_weight = 9.0;
  optimizer.add_pose_prior(pose_prior);

  gaussian_lic_tracking::SlidingWindowStatePrior state_prior;
  state_prior.stamp_ns = state.stamp_ns;
  state_prior.q_w_i = state.q_w_i;
  state_prior.v_w_i = state.v_w_i;
  state_prior.p_w_i = state.p_w_i;
  state_prior.gyro_bias = state.gyro_bias;
  state_prior.accel_bias = state.accel_bias;
  state_prior.sqrt_information = Eigen::Matrix<double, 15, 15>::Identity();
  state_prior.rotation_weight = 16.0;
  state_prior.velocity_weight = 25.0;
  state_prior.position_weight = 36.0;
  state_prior.gyro_bias_weight = 49.0;
  state_prior.accel_bias_weight = 64.0;
  optimizer.add_state_prior(state_prior);

  gaussian_lic_tracking::SlidingWindowDensePrior dense_prior;
  dense_prior.stamp_ns.push_back(state.stamp_ns);
  dense_prior.reference_states.push_back(state);
  dense_prior.sqrt_information = Eigen::MatrixXd::Zero(15, 15);
  for (Eigen::Index i = 0; i < 15; ++i) {
    dense_prior.sqrt_information(i, i) = 0.1 * static_cast<double>(i + 1);
  }
  dense_prior.target_delta = Eigen::VectorXd::Zero(15);
  optimizer.add_dense_prior(dense_prior);

  gaussian_lic_tracking::SlidingWindowSe3PhotometricFactor se3_factor;
  se3_factor.stamp_ns = state.stamp_ns;
  se3_factor.reference_q_w_i = state.q_w_i;
  se3_factor.reference_p_w_i = state.p_w_i;
  se3_factor.weight = 4.0;
  se3_factor.sqrt_information = Eigen::Matrix<double, 6, 6>::Zero();
  for (Eigen::Index i = 0; i < 6; ++i) {
    se3_factor.sqrt_information(i, i) = 0.2 * static_cast<double>(i + 1);
  }
  optimizer.add_se3_photometric_factor(se3_factor);

  const auto normal = optimizer.build_normal_equation(0.0);
  if (!normal.valid || normal.jacobian.rows() != 42 || normal.jacobian.cols() != 15) {
    std::cerr << "prior Jacobian normal equation has wrong dimensions\n";
    return 1;
  }

  Eigen::MatrixXd expected = Eigen::MatrixXd::Zero(42, 15);
  const Eigen::Matrix3d rotation_jacobian =
    state.q_w_i.normalized().inverse().toRotationMatrix();
  expected.block<3, 3>(0, 0) = 2.0 * rotation_jacobian;
  expected.block<3, 3>(3, 6) = 3.0 * Eigen::Matrix3d::Identity();

  Eigen::Matrix<double, 15, 15> state_sqrt = Eigen::Matrix<double, 15, 15>::Zero();
  state_sqrt.block<3, 3>(0, 0) = 4.0 * Eigen::Matrix3d::Identity();
  state_sqrt.block<3, 3>(3, 3) = 5.0 * Eigen::Matrix3d::Identity();
  state_sqrt.block<3, 3>(6, 6) = 6.0 * Eigen::Matrix3d::Identity();
  state_sqrt.block<3, 3>(9, 9) = 7.0 * Eigen::Matrix3d::Identity();
  state_sqrt.block<3, 3>(12, 12) = 8.0 * Eigen::Matrix3d::Identity();
  expected.block<15, 15>(6, 0) =
    state_sqrt * expected_state_delta_jacobian(state.q_w_i);

  expected.block(21, 0, 15, 15) =
    dense_prior.sqrt_information * expected_state_delta_jacobian(state.q_w_i);

  Eigen::Matrix<double, 6, 15> se3_delta = Eigen::Matrix<double, 6, 15>::Zero();
  se3_delta.block<3, 3>(0, 0) = rotation_jacobian;
  se3_delta.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity();
  expected.block(36, 0, 6, 15) = 2.0 * se3_factor.sqrt_information * se3_delta;

  const double max_abs_error = (normal.jacobian - expected).cwiseAbs().maxCoeff();
  std::cout << "sliding_window_prior_jacobian_probe rows=" << normal.jacobian.rows()
            << " cols=" << normal.jacobian.cols()
            << " max_abs_error=" << max_abs_error << "\n";
  if (max_abs_error > 1.0e-12) {
    std::cerr << "prior/dense/SE3 analytic Jacobian blocks are wrong\n";
    return 1;
  }

  const double nan = std::numeric_limits<double>::quiet_NaN();
  const Eigen::Quaterniond zero_quaternion(0.0, 0.0, 0.0, 0.0);

  bool validation_ok = true;
  validation_ok &= expect_throw("non-finite sliding-window config", [nan]() {
    gaussian_lic_tracking::SlidingWindowConfig config;
    config.damping = nan;
    gaussian_lic_tracking::SlidingWindowOptimizer invalid_optimizer(config);
  });
  validation_ok &= expect_throw("zero sliding-window state quaternion", [state, zero_quaternion]() {
    gaussian_lic_tracking::SlidingWindowOptimizer invalid_optimizer;
    auto invalid_state = state;
    invalid_state.q_w_i = zero_quaternion;
    invalid_optimizer.add_or_update_state(invalid_state);
  });
  validation_ok &= expect_throw("non-finite sliding-window state", [state, nan]() {
    gaussian_lic_tracking::SlidingWindowOptimizer invalid_optimizer;
    auto invalid_state = state;
    invalid_state.p_w_i.x() = nan;
    invalid_optimizer.add_or_update_state(invalid_state);
  });
  validation_ok &= expect_throw("non-finite IMU factor weight", [nan]() {
    gaussian_lic_tracking::SlidingWindowOptimizer invalid_optimizer;
    gaussian_lic_tracking::SlidingWindowImuFactor factor;
    factor.from_stamp_ns = 0;
    factor.to_stamp_ns = 1;
    factor.weight = nan;
    invalid_optimizer.add_imu_factor(factor);
  });
  validation_ok &= expect_throw("zero pose-prior quaternion", [pose_prior, zero_quaternion]() {
    gaussian_lic_tracking::SlidingWindowOptimizer invalid_optimizer;
    auto invalid_prior = pose_prior;
    invalid_prior.q_w_i = zero_quaternion;
    invalid_optimizer.add_pose_prior(invalid_prior);
  });
  validation_ok &= expect_throw("non-finite pose-prior weight", [pose_prior, nan]() {
    gaussian_lic_tracking::SlidingWindowOptimizer invalid_optimizer;
    auto invalid_prior = pose_prior;
    invalid_prior.translation_weight = nan;
    invalid_optimizer.add_pose_prior(invalid_prior);
  });
  validation_ok &= expect_throw("non-finite state-prior information", [state_prior, nan]() {
    gaussian_lic_tracking::SlidingWindowOptimizer invalid_optimizer;
    auto invalid_prior = state_prior;
    invalid_prior.sqrt_information(0, 0) = nan;
    invalid_optimizer.add_state_prior(invalid_prior);
  });
  validation_ok &= expect_throw("invalid dense-prior reference state", [dense_prior, zero_quaternion]() {
    gaussian_lic_tracking::SlidingWindowOptimizer invalid_optimizer;
    auto invalid_prior = dense_prior;
    invalid_prior.reference_states[0].q_w_i = zero_quaternion;
    invalid_optimizer.add_dense_prior(invalid_prior);
  });
  validation_ok &= expect_throw("non-finite point-factor weight", [nan]() {
    gaussian_lic_tracking::SlidingWindowOptimizer invalid_optimizer;
    gaussian_lic_tracking::SlidingWindowPointToPointFactor factor;
    factor.weight = nan;
    factor.frame_points_i.emplace_back(0.0, 0.0, 0.0);
    factor.target_points_w.emplace_back(0.0, 0.0, 0.0);
    invalid_optimizer.add_point_to_point_factor(factor);
  });
  validation_ok &= expect_throw("out-of-range point robust weight", []() {
    gaussian_lic_tracking::SlidingWindowOptimizer invalid_optimizer;
    gaussian_lic_tracking::SlidingWindowPointToPointFactor factor;
    factor.frame_points_i.emplace_back(0.0, 0.0, 0.0);
    factor.target_points_w.emplace_back(0.0, 0.0, 0.0);
    factor.point_weights.push_back(1.5);
    invalid_optimizer.add_point_to_point_factor(factor);
  });
  validation_ok &= expect_throw("non-finite plane-factor weight", [nan]() {
    gaussian_lic_tracking::SlidingWindowOptimizer invalid_optimizer;
    gaussian_lic_tracking::SlidingWindowPointToPlaneFactor factor;
    factor.weight = nan;
    factor.frame_points_i.emplace_back(0.0, 0.0, 0.0);
    factor.target_points_w.emplace_back(0.0, 0.0, 0.0);
    factor.target_normals_w.emplace_back(0.0, 0.0, 1.0);
    invalid_optimizer.add_point_to_plane_factor(factor);
  });
  validation_ok &= expect_throw("out-of-range plane robust weight", []() {
    gaussian_lic_tracking::SlidingWindowOptimizer invalid_optimizer;
    gaussian_lic_tracking::SlidingWindowPointToPlaneFactor factor;
    factor.frame_points_i.emplace_back(0.0, 0.0, 0.0);
    factor.target_points_w.emplace_back(0.0, 0.0, 0.0);
    factor.target_normals_w.emplace_back(0.0, 0.0, 1.0);
    factor.point_weights.push_back(0.0);
    invalid_optimizer.add_point_to_plane_factor(factor);
  });
  validation_ok &= expect_throw("non-finite visual alignment scale", [state, nan]() {
    gaussian_lic_tracking::SlidingWindowOptimizer invalid_optimizer;
    gaussian_lic_tracking::SlidingWindowVisualAlignmentFactor factor;
    factor.stamp_ns = state.stamp_ns;
    factor.reference_p_w_i = state.p_w_i;
    factor.meters_per_pixel = nan;
    invalid_optimizer.add_visual_alignment_factor(factor);
  });
  validation_ok &= expect_throw(
    "invalid SE3 photometric reference quaternion", [se3_factor, zero_quaternion]() {
      gaussian_lic_tracking::SlidingWindowOptimizer invalid_optimizer;
      auto invalid_factor = se3_factor;
      invalid_factor.reference_q_w_i = zero_quaternion;
      invalid_optimizer.add_se3_photometric_factor(invalid_factor);
    });
  validation_ok &= expect_throw("non-finite SE3 photometric weight", [se3_factor, nan]() {
    gaussian_lic_tracking::SlidingWindowOptimizer invalid_optimizer;
    auto invalid_factor = se3_factor;
    invalid_factor.weight = nan;
    invalid_optimizer.add_se3_photometric_factor(invalid_factor);
  });
  validation_ok &= expect_throw("non-finite smoothness weight", [nan]() {
    gaussian_lic_tracking::SlidingWindowOptimizer invalid_optimizer;
    gaussian_lic_tracking::SlidingWindowTrajectorySmoothnessFactor factor;
    factor.previous_stamp_ns = 0;
    factor.current_stamp_ns = 1;
    factor.next_stamp_ns = 2;
    factor.rotation_rate_weight = nan;
    invalid_optimizer.add_trajectory_smoothness_factor(factor);
  });
  if (!validation_ok) {
    std::cerr << "sliding-window optimizer validation probes failed\n";
    return 1;
  }

  std::cout << "sliding_window_prior_jacobian_probe OK\n";
  return 0;
}
