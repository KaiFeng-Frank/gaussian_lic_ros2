// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>
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
  double max_rotation_step_rad{0.0};
  double max_velocity_step_mps{0.0};
  double max_translation_step_m{0.0};
  double max_bias_step{0.0};
  double max_normal_equation_condition{1.0e13};
  double min_normal_equation_rank_ratio{0.0};
  double max_state_gap_s{1.0};
  bool visual_marginalization_prior_zero_bias_columns{false};
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
  double rotation_weight{1.0};
  double velocity_weight{1.0};
  double position_weight{1.0};
  double bias_weight{1.0};
  double gyro_bias_weight{1.0};
  double accel_bias_weight{1.0};
  double bias_random_walk_reference_dt_s{0.0};
  double gyro_bias_random_walk_sigma{0.0};
  double accel_bias_random_walk_sigma{0.0};
  Eigen::Matrix<double, 9, 9> sqrt_information{Eigen::Matrix<double, 9, 9>::Zero()};
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
  uint64_t source_id{0};
  std::vector<Eigen::Vector3d> frame_points_i;
  std::vector<Eigen::Vector3d> target_points_w;
  std::vector<double> point_weights;
  double weight{1.0};
  double huber_delta_m{0.0};
};

struct SlidingWindowPointToPlaneFactor
{
  int64_t stamp_ns{0};
  uint64_t source_id{0};
  std::vector<Eigen::Vector3d> frame_points_i;
  std::vector<Eigen::Vector3d> target_points_w;
  std::vector<Eigen::Vector3d> target_normals_w;
  std::vector<double> point_weights;
  double weight{1.0};
  double huber_delta_m{0.0};
};

struct SlidingWindowVisualAlignmentFactor
{
  int64_t stamp_ns{0};
  uint64_t source_id{0};
  std::vector<int64_t> support_stamp_ns;
  std::vector<double> support_weights;
  bool has_reference_pose{false};
  Eigen::Vector2d measured_shift_px{Eigen::Vector2d::Zero()};
  Eigen::Vector3d reference_p_w_i{Eigen::Vector3d::Zero()};
  double meters_per_pixel{0.01};
  double weight{1.0};
  double huber_delta_m{0.0};
};

struct SlidingWindowSe3PhotometricFactor
{
  int64_t stamp_ns{0};
  uint64_t source_id{0};
  std::vector<int64_t> support_stamp_ns;
  std::vector<double> support_weights;
  bool has_reference_pose{false};
  Eigen::Vector3d reference_p_w_i{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond reference_q_w_i{Eigen::Quaterniond::Identity()};
  Eigen::Matrix<double, 6, 1> target_delta{Eigen::Matrix<double, 6, 1>::Zero()};
  Eigen::Matrix<double, 6, 6> sqrt_information{Eigen::Matrix<double, 6, 6>::Identity()};
  double weight{1.0};
  double huber_delta{0.0};
};

struct SlidingWindowRelativeTranslationFactor
{
  int64_t from_stamp_ns{0};
  int64_t to_stamp_ns{0};
  uint64_t source_id{0};
  Eigen::Vector3d delta_p_w{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond delta_q_from_to{Eigen::Quaterniond::Identity()};
  bool translation_in_from_frame{false};
  double weight{1.0};
  double huber_delta_m{0.0};
  double rotation_weight{0.0};
  double rotation_huber_delta_rad{0.0};
};

struct SlidingWindowRelativeDistanceFactor
{
  int64_t from_stamp_ns{0};
  int64_t to_stamp_ns{0};
  uint64_t source_id{0};
  double distance_m{0.0};
  double weight{1.0};
  double huber_delta_m{0.0};
};

struct SlidingWindowTrajectorySmoothnessFactor
{
  int64_t previous_stamp_ns{0};
  int64_t current_stamp_ns{0};
  int64_t next_stamp_ns{0};
  Eigen::Vector3d target_rotation_rate_delta{Eigen::Vector3d::Zero()};
  Eigen::Vector3d target_position_rate_delta{Eigen::Vector3d::Zero()};
  Eigen::Vector3d target_velocity_acceleration_delta{Eigen::Vector3d::Zero()};
  Eigen::Vector3d target_gyro_bias_rate_delta{Eigen::Vector3d::Zero()};
  Eigen::Vector3d target_accel_bias_rate_delta{Eigen::Vector3d::Zero()};
  double rotation_rate_weight{1.0};
  double position_rate_weight{1.0};
  double velocity_acceleration_weight{1.0};
  double position_velocity_consistency_weight{0.0};
  double gyro_bias_rate_weight{1.0};
  double accel_bias_rate_weight{1.0};
};

struct SlidingWindowDensePrior
{
  std::vector<int64_t> stamp_ns;
  std::vector<SlidingWindowState> reference_states;
  Eigen::MatrixXd sqrt_information;
  Eigen::VectorXd target_delta;
};

struct SlidingWindowCostBreakdown
{
  double imu_cost{0.0};
  double pose_prior_cost{0.0};
  double state_prior_cost{0.0};
  double dense_prior_cost{0.0};
  double point_factor_cost{0.0};
  double plane_factor_cost{0.0};
  double visual_factor_cost{0.0};
  double se3_photometric_factor_cost{0.0};
  double relative_translation_factor_cost{0.0};
  double relative_distance_factor_cost{0.0};
  double smoothness_factor_cost{0.0};
};

struct SlidingWindowSummary
{
  size_t state_count{0};
  size_t imu_factor_count{0};
  size_t pose_prior_count{0};
  size_t state_prior_count{0};
  size_t dense_prior_count{0};
  size_t point_factor_count{0};
  size_t plane_factor_count{0};
  size_t visual_factor_count{0};
  size_t se3_photometric_factor_count{0};
  size_t relative_translation_factor_count{0};
  size_t relative_distance_factor_count{0};
  size_t smoothness_factor_count{0};
  size_t imu_factor_replacement_count{0};
  size_t point_factor_replacement_count{0};
  size_t plane_factor_replacement_count{0};
  size_t visual_factor_replacement_count{0};
  size_t se3_photometric_factor_replacement_count{0};
  size_t relative_translation_factor_replacement_count{0};
  size_t relative_distance_factor_replacement_count{0};
  size_t smoothness_factor_replacement_count{0};
  size_t orphan_factor_count{0};
  size_t marginalized_state_count{0};
  size_t schur_marginalization_count{0};
  size_t fallback_marginalization_prior_count{0};
  size_t marginalized_backsubstitution_count{0};
  size_t marginalized_backsubstitution_chain_update_count{0};
  size_t marginalized_backsubstitution_interpolation_count{0};
  size_t visual_marginalization_prior_count{0};
  size_t se3_photometric_marginalization_prior_count{0};
  size_t dense_prior_rows{0};
  size_t dense_prior_cols{0};
  size_t dense_prior_rank{0};
  size_t normal_equation_rows{0};
  size_t normal_equation_cols{0};
  size_t normal_equation_rank{0};
  size_t numeric_jacobian_block_count{0};
  size_t numeric_jacobian_column_count{0};
  size_t iterations{0};
  size_t accepted_steps{0};
  size_t rejected_steps{0};
  size_t limited_steps{0};
  size_t invalid_candidate_steps{0};
  size_t linearization_failure_count{0};
  size_t linear_solve_failure_count{0};
  double initial_cost{0.0};
  double final_cost{0.0};
  double imu_cost{0.0};
  double pose_prior_cost{0.0};
  double state_prior_cost{0.0};
  double dense_prior_cost{0.0};
  double point_factor_cost{0.0};
  double plane_factor_cost{0.0};
  double visual_factor_cost{0.0};
  double se3_photometric_factor_cost{0.0};
  double relative_translation_factor_cost{0.0};
  double relative_distance_factor_cost{0.0};
  double smoothness_factor_cost{0.0};
  double last_step_norm{0.0};
  double last_step_scale{0.0};
  double last_damping{0.0};
  double dense_prior_min_singular_value{0.0};
  double dense_prior_max_singular_value{0.0};
  double dense_prior_gyro_bias_min_singular_value{0.0};
  double dense_prior_gyro_bias_max_singular_value{0.0};
  double dense_prior_accel_bias_min_singular_value{0.0};
  double dense_prior_accel_bias_max_singular_value{0.0};
  double min_state_dt_s{0.0};
  double max_state_dt_s{0.0};
  double normal_equation_min_singular_value{0.0};
  double normal_equation_max_singular_value{0.0};
  double normal_equation_condition_number{0.0};
  double gyro_bias_norm{0.0};
  double accel_bias_norm{0.0};
  double gyro_bias_random_walk_sqrt_info_mean{0.0};
  double accel_bias_random_walk_sqrt_info_mean{0.0};
  double gyro_bias_random_walk_sqrt_info_max{0.0};
  double accel_bias_random_walk_sqrt_info_max{0.0};
  double gyro_bias_observability{0.0};
  double accel_bias_observability{0.0};
  bool converged{false};
  bool normal_equation_degenerate{false};
  bool state_gap_degenerate{false};
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
  size_t numeric_jacobian_block_count{0};
  size_t numeric_jacobian_column_count{0};
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
  void add_point_to_plane_factor(const SlidingWindowPointToPlaneFactor & factor);
  void add_visual_alignment_factor(const SlidingWindowVisualAlignmentFactor & factor);
  void add_se3_photometric_factor(const SlidingWindowSe3PhotometricFactor & factor);
  bool add_marginalized_visual_alignment_prior(const SlidingWindowVisualAlignmentFactor & factor);
  bool add_marginalized_se3_photometric_prior(const SlidingWindowSe3PhotometricFactor & factor);
  bool add_marginalized_relative_translation_prior(
    const SlidingWindowRelativeTranslationFactor & factor);
  void add_relative_translation_factor(const SlidingWindowRelativeTranslationFactor & factor);
  void add_relative_distance_factor(const SlidingWindowRelativeDistanceFactor & factor);
  void add_trajectory_smoothness_factor(const SlidingWindowTrajectorySmoothnessFactor & factor);

  SlidingWindowNormalEquation build_normal_equation(double damping = 0.0) const;
  SlidingWindowSummary optimize();

private:
  struct VariableBlock
  {
    size_t state_index{0};
    size_t offset{0};
  };
  struct NumericJacobianBlock
  {
    Eigen::Index row_start{0};
    Eigen::Index row_size{0};
    Eigen::Index column_start{0};
    Eigen::Index column_size{0};
  };
  struct MarginalizedStateBacksubstitution
  {
    int64_t marginalized_stamp_ns{0};
    SlidingWindowState marginalized_reference_state;
    std::vector<int64_t> retained_stamp_ns;
    std::vector<SlidingWindowState> retained_reference_states;
    Eigen::Matrix<double, 15, 1> marginalized_target_delta{
      Eigen::Matrix<double, 15, 1>::Zero()};
    Eigen::MatrixXd marginalized_delta_from_retained;
  };
  struct MarginalizedMeasurementBacksubstitution
  {
    SlidingWindowState marginalized_reference_state;
    std::vector<int64_t> retained_stamp_ns;
    std::vector<SlidingWindowState> retained_reference_states;
    Eigen::Matrix<double, 15, 1> marginalized_target_delta{
      Eigen::Matrix<double, 15, 1>::Zero()};
    Eigen::MatrixXd marginalized_delta_from_retained;
    bool interpolated{false};
  };
  struct MeasurementEndpointBacksubstitution
  {
    SlidingWindowState reference_state;
    std::vector<int64_t> retained_stamp_ns;
    std::vector<SlidingWindowState> retained_reference_states;
    Eigen::Matrix<double, 15, 1> target_delta{Eigen::Matrix<double, 15, 1>::Zero()};
    Eigen::MatrixXd delta_from_retained;
    bool active{false};
    bool interpolated{false};
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
  Eigen::VectorXd build_residual(
    const std::vector<SlidingWindowState> & states,
    SlidingWindowCostBreakdown * breakdown) const;
  std::vector<NumericJacobianBlock> fill_analytic_jacobian(
    const std::vector<SlidingWindowState> & states,
    const std::vector<VariableBlock> & variables,
    Eigen::MatrixXd & jacobian) const;
  SlidingWindowNormalEquation linearize(
    const std::vector<SlidingWindowState> & states,
    const std::vector<VariableBlock> & variables,
    double damping) const;
  double compute_cost(const Eigen::VectorXd & residual) const;
  bool add_schur_marginalization_prior_for_front();
  bool add_marginalized_measurement_prior(
    int64_t factor_stamp_ns,
    const Eigen::VectorXd & residual,
    const Eigen::MatrixXd & marginalized_jacobian);
  bool add_marginalized_support_measurement_prior(
    const std::vector<MeasurementEndpointBacksubstitution> & endpoints,
    const Eigen::VectorXd & residual,
    const std::vector<Eigen::MatrixXd> & endpoint_jacobians);
  bool add_dense_measurement_prior(
    const std::vector<int64_t> & retained_stamp_ns,
    const std::vector<SlidingWindowState> & retained_reference_states,
    const Eigen::VectorXd & residual,
    const Eigen::MatrixXd & jacobian);
  std::optional<MeasurementEndpointBacksubstitution> build_measurement_endpoint_backsubstitution(
    int64_t factor_stamp_ns) const;
  const MarginalizedStateBacksubstitution * select_marginalized_backsubstitution(
    int64_t factor_stamp_ns) const;
  std::optional<MarginalizedMeasurementBacksubstitution> build_marginalized_measurement_backsubstitution(
    int64_t factor_stamp_ns) const;
  size_t propagate_marginalized_backsubstitutions(
    const MarginalizedStateBacksubstitution & substitution);
  size_t prune_marginalized_backsubstitutions();
  size_t prune_marginalized_factor_references();
  size_t count_orphan_factors() const;
  size_t enforce_window_size();

  SlidingWindowConfig config_;
  std::vector<SlidingWindowState> states_;
  std::vector<SlidingWindowImuFactor> imu_factors_;
  std::vector<SlidingWindowPosePrior> pose_priors_;
  std::vector<SlidingWindowStatePrior> state_priors_;
  std::vector<SlidingWindowDensePrior> dense_priors_;
  std::vector<MarginalizedStateBacksubstitution> marginalized_backsubstitutions_;
  std::vector<SlidingWindowPointToPointFactor> point_factors_;
  std::vector<SlidingWindowPointToPlaneFactor> plane_factors_;
  std::vector<SlidingWindowVisualAlignmentFactor> visual_factors_;
  std::vector<SlidingWindowSe3PhotometricFactor> se3_photometric_factors_;
  std::vector<SlidingWindowRelativeTranslationFactor> relative_translation_factors_;
  std::vector<SlidingWindowRelativeDistanceFactor> relative_distance_factors_;
  std::vector<SlidingWindowTrajectorySmoothnessFactor> smoothness_factors_;
  size_t imu_factor_replacement_count_{0};
  size_t point_factor_replacement_count_{0};
  size_t plane_factor_replacement_count_{0};
  size_t visual_factor_replacement_count_{0};
  size_t se3_photometric_factor_replacement_count_{0};
  size_t relative_translation_factor_replacement_count_{0};
  size_t relative_distance_factor_replacement_count_{0};
  size_t smoothness_factor_replacement_count_{0};
  size_t marginalized_state_count_{0};
  size_t schur_marginalization_count_{0};
  size_t fallback_marginalization_prior_count_{0};
  size_t marginalized_backsubstitution_chain_update_count_{0};
  size_t marginalized_backsubstitution_interpolation_count_{0};
  size_t visual_marginalization_prior_count_{0};
  size_t se3_photometric_marginalization_prior_count_{0};
};

}  // namespace gaussian_lic_tracking
