// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gaussian_lic_tracking/imu_preintegrator.hpp>

namespace gaussian_lic_tracking
{

struct SlidingWindowConfig
{
  size_t max_states{12};
  size_t max_iterations{4};
  double damping{1.0e-6};
  double step_tolerance{1.0e-8};
  double numeric_epsilon{1.0e-6};
  double marginalization_prior_weight{1.0};
};

struct SlidingWindowState
{
  int64_t stamp_ns{0};
  Eigen::Vector3d p_w_i{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q_w_i{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d v_w_i{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gyro_bias{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accel_bias{Eigen::Vector3d::Zero()};
  bool fixed{false};
};

struct SlidingWindowImuFactor
{
  int64_t from_stamp_ns{0};
  int64_t to_stamp_ns{0};
  ImuPreintegrator preintegration;
  Eigen::Vector3d gravity_w{Eigen::Vector3d::Zero()};
  double weight{1.0};
  double bias_weight{1.0};
};

struct SlidingWindowPosePrior
{
  int64_t stamp_ns{0};
  Eigen::Vector3d p_w_i{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q_w_i{Eigen::Quaterniond::Identity()};
  double translation_weight{1.0};
  double rotation_weight{1.0};
};

struct SlidingWindowStatePrior
{
  int64_t stamp_ns{0};
  Eigen::Vector3d p_w_i{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q_w_i{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d v_w_i{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gyro_bias{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accel_bias{Eigen::Vector3d::Zero()};
  Eigen::Matrix<double, 15, 15> sqrt_information{Eigen::Matrix<double, 15, 15>::Identity()};
  double rotation_weight{1.0};
  double velocity_weight{1.0};
  double position_weight{1.0};
  double gyro_bias_weight{1.0};
  double accel_bias_weight{1.0};
};

struct SlidingWindowPointToPointFactor
{
  int64_t stamp_ns{0};
  std::vector<Eigen::Vector3d> frame_points_i;
  std::vector<Eigen::Vector3d> target_points_w;
  double weight{1.0};
};

struct SlidingWindowVisualAlignmentFactor
{
  int64_t stamp_ns{0};
  Eigen::Vector2d measured_shift_px{Eigen::Vector2d::Zero()};
  Eigen::Vector3d reference_p_w_i{Eigen::Vector3d::Zero()};
  double meters_per_pixel{0.01};
  double weight{1.0};
};

struct SlidingWindowDensePrior
{
  std::vector<int64_t> stamp_ns;
  std::vector<SlidingWindowState> reference_states;
  Eigen::MatrixXd sqrt_information;
  Eigen::VectorXd target_delta;
};

struct SlidingWindowSummary
{
  size_t state_count{0};
  size_t imu_factor_count{0};
  size_t pose_prior_count{0};
  size_t state_prior_count{0};
  size_t dense_prior_count{0};
  size_t point_factor_count{0};
  size_t visual_factor_count{0};
  size_t marginalized_state_count{0};
  size_t iterations{0};
  double initial_cost{0.0};
  double final_cost{0.0};
  bool converged{false};
};

struct SchurComplementResult
{
  bool valid{false};
  Eigen::MatrixXd hessian;
  Eigen::VectorXd rhs;
};

struct SlidingWindowNormalEquation
{
  bool valid{false};
  size_t variable_count{0};
  Eigen::VectorXd residual;
  Eigen::MatrixXd jacobian;
  Eigen::MatrixXd hessian;
  Eigen::VectorXd rhs;
  double cost{0.0};
};

SchurComplementResult compute_schur_complement(
  const Eigen::MatrixXd & h_mm,
  const Eigen::MatrixXd & h_mr,
  const Eigen::MatrixXd & h_rr,
  const Eigen::VectorXd & rhs_m,
  const Eigen::VectorXd & rhs_r,
  double damping);

class SlidingWindowOptimizer
{
public:
  explicit SlidingWindowOptimizer(SlidingWindowConfig config = {});

  void set_config(const SlidingWindowConfig & config);
  const SlidingWindowConfig & config() const { return config_; }

  void clear();
  void add_or_update_state(const SlidingWindowState & state);
  bool get_state(int64_t stamp_ns, SlidingWindowState & state) const;
  const std::vector<SlidingWindowState> & states() const { return states_; }

  void add_imu_factor(const SlidingWindowImuFactor & factor);
  void add_pose_prior(const SlidingWindowPosePrior & prior);
  void add_state_prior(const SlidingWindowStatePrior & prior);
  void add_dense_prior(const SlidingWindowDensePrior & prior);
  void add_point_to_point_factor(const SlidingWindowPointToPointFactor & factor);
  void add_visual_alignment_factor(const SlidingWindowVisualAlignmentFactor & factor);

  SlidingWindowNormalEquation build_normal_equation(double damping = 0.0) const;
  SlidingWindowSummary optimize();

private:
  struct VariableBlock
  {
    size_t state_index{0};
    size_t offset{0};
  };

  static Eigen::Vector3d rotation_residual(
    const Eigen::Quaterniond & measured_q,
    const Eigen::Quaterniond & predicted_q);
  static Eigen::Matrix<double, 15, 1> state_delta(
    const SlidingWindowState & reference,
    const SlidingWindowState & state);
  static void apply_delta(
    std::vector<SlidingWindowState> & states,
    const std::vector<VariableBlock> & variables,
    const Eigen::VectorXd & delta,
    double scale = 1.0);

  int find_state_index(int64_t stamp_ns) const;
  SlidingWindowStatePrior make_state_prior(const SlidingWindowState & state) const;
  std::vector<VariableBlock> variable_layout() const;
  Eigen::VectorXd build_residual(const std::vector<SlidingWindowState> & states) const;
  SlidingWindowNormalEquation linearize(
    const std::vector<SlidingWindowState> & states,
    const std::vector<VariableBlock> & variables,
    double damping) const;
  double compute_cost(const Eigen::VectorXd & residual) const;
  bool add_schur_marginalization_prior_for_front();
  size_t enforce_window_size();

  SlidingWindowConfig config_;
  std::vector<SlidingWindowState> states_;
  std::vector<SlidingWindowImuFactor> imu_factors_;
  std::vector<SlidingWindowPosePrior> pose_priors_;
  std::vector<SlidingWindowStatePrior> state_priors_;
  std::vector<SlidingWindowDensePrior> dense_priors_;
  std::vector<SlidingWindowPointToPointFactor> point_factors_;
  std::vector<SlidingWindowVisualAlignmentFactor> visual_factors_;
  size_t marginalized_state_count_{0};
};

}  // namespace gaussian_lic_tracking
