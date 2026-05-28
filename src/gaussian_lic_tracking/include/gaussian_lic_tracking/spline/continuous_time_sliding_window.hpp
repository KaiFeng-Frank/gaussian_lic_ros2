// SPDX-License-Identifier: GPL-3.0-or-later
//
// ROS2-native port of Coco-LIC's odometry sliding-window driver. Combines
// the ported `TrajectoryEstimator` with `SplineMarginalizationInfo` into a
// streaming online estimator:
//
//   1. The window holds a fixed number of consecutive cumulative B-spline
//      knots. The first solve treats every knot as free.
//   2. Incoming IMU / LiDAR samples are bucketed against the segment they
//      fall in. Samples outside the current window are buffered until the
//      window slides forward to cover them.
//   3. `step()` extends the window by one knot (carrying world pose
//      forward), absorbs all newly buffered samples, optionally marginalizes
//      the oldest knot(s), and runs Ceres on the active window.
//
// Note: marginalization is performed at the linearized-residual level by
// snapshotting Jacobians of every factor that touches the to-be-dropped
// knots, running Schur complement through `SplineMarginalizationInfo`, and
// re-injecting the resulting square-root prior as a regular residual block
// on the kept knots.

#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gaussian_lic_tracking/spline/continuous_time_imu_factor.hpp>
#include <gaussian_lic_tracking/spline/continuous_time_lidar_factor.hpp>
#include <gaussian_lic_tracking/spline/spline_marginalization.hpp>
#include <gaussian_lic_tracking/spline/trajectory_estimator.hpp>

namespace gaussian_lic_tracking
{
namespace spline
{

struct ContinuousTimeSlidingWindowOptions
{
  double dt_s{0.05};
  int window_knot_count{8};
  int marginalize_oldest_count{1};
  int max_iterations_per_step{20};
  Eigen::Vector3d gravity_world{Eigen::Vector3d(0.0, 0.0, -9.81)};
  Eigen::Vector3d initial_gyro_bias{Eigen::Vector3d::Zero()};
  Eigen::Vector3d initial_accel_bias{Eigen::Vector3d::Zero()};
  // Per-axis IMU information weights. Higher = tighter constraint.
  // Defaults reflect realistic Coco-LIC / VINS-Mono settings for a
  // commodity 200 Hz IMU: gyro variance ~1e-2 rad²/s² → info ~10²,
  // accel variance ~1e-1 m²/s⁴ → info ~10¹.
  double imu_info_gyro{100.0};
  double imu_info_accel{10.0};
  // Optional Ceres trust-region controls for online windows. Leave <= 0 to
  // use Ceres defaults.
  double ceres_initial_trust_region_radius{0.0};
  double ceres_max_trust_region_radius{0.0};
  double position_smoothness_weight{0.0};
  double position_smoothness_huber_delta_m{0.0};
  double rotation_smoothness_weight{0.0};
  double rotation_smoothness_huber_delta_rad{0.0};
  // Soft priors on the first N retained knots in each local solve. This
  // approximates Coco-LIC's marginalized fixed-prefix anchor without forcing a
  // hard SetFixedIndex path in experiments where the dense prior is incomplete.
  int retained_knot_prior_count{0};
  double retained_knot_position_prior_weight{0.0};
  double retained_knot_position_prior_huber_delta_m{0.0};
  double retained_knot_orientation_prior_weight{0.0};
  double retained_knot_orientation_prior_huber_delta_rad{0.0};
  // Soft random-walk priors on the IMU biases. Each online solve is biased
  // toward the accepted bias from the previous solve, but Ceres can still
  // move the bias when IMU evidence is strong enough.
  double gyro_bias_prior_weight{0.0};
  double gyro_bias_prior_huber_delta_radps{0.0};
  double accel_bias_prior_weight{0.0};
  double accel_bias_prior_huber_delta_mps2{0.0};
  double lidar_huber_delta_m{0.10};
  bool hold_gyro_bias_constant{false};
  bool hold_accel_bias_constant{false};
  bool hold_gravity_constant{true};
  // Optional Coco-LIC TrajectoryEstimator::SetFixedIndex-style gauge anchor.
  // Full upstream mode uses 3 together with marginalization priors; keep the
  // standalone sliding-window default disabled to avoid over-constraining the
  // current simplified online prior.
  int fixed_control_point_index{-1};
  // Reject a Ceres update instead of publishing / carrying it forward when
  // the online solve proposes an implausibly large single-step knot change.
  // This is a production guard for real rosbag replay: bad initialization or
  // a malformed geometric factor should drop that solve, not explode odometry.
  double max_position_update_m{2.0};
  double max_rotation_update_rad{0.50};
  // Exclude this many guard knots on each side when computing the update
  // acceptance magnitude. Edge knots are finite-checked but do not veto the
  // whole interior update when this is positive.
  int update_gate_edge_knot_margin{0};
  // Multiplier for the constant-velocity position prediction used when a new
  // knot is appended before optimization. Keep the library default at 1.0 for
  // synthetic tests; real-bag launch paths can lower this when rejected online
  // solves would otherwise dead-reckon a stale velocity for hundreds of steps.
  double position_extrapolation_damping{1.0};
  // Online real-bag solves can occasionally propose a poorly-observed SO(3)
  // update while the position spline update remains small and useful. Keep
  // the old rotations in that case but let the bounded position correction
  // feed the next window instead of turning the whole step into dead
  // reckoning.
  bool apply_position_update_on_rotation_reject{false};
  // When the SO(3) proposal exceeds `max_rotation_update_rad`, apply a
  // log-map scaled version of the rotation delta instead of rejecting the
  // rotation entirely. This is an online trust-region guard for real bags.
  bool apply_limited_rotation_update{false};
  // When the translation proposal exceeds `max_position_update_m`, apply the
  // largest safe fraction of the position delta instead of discarding the
  // whole step. Keep this default-off until a bag-specific strict parity run
  // proves it improves long-window accuracy.
  bool apply_limited_position_update{false};
  // When limited rotation is active, scale the accompanying position knot
  // delta by the same trust-region ratio. This keeps a rejected SE(3)
  // proposal from applying full translation with only partial rotation.
  bool scale_position_with_limited_rotation{true};
};

struct ContinuousTimeSlidingWindowDiagnostics
{
  std::size_t steps_run{0};
  std::size_t total_imu_factors{0};
  std::size_t total_lidar_factors{0};
  std::size_t total_lidar_point_factors{0};
  std::size_t total_lidar_normal_factors{0};
  std::size_t total_position_prior_factors{0};
  std::size_t total_velocity_prior_factors{0};
  std::size_t total_angular_velocity_prior_factors{0};
  std::size_t total_orientation_prior_factors{0};
  std::size_t total_gyro_bias_prior_factors{0};
  std::size_t total_accel_bias_prior_factors{0};
  std::size_t total_position_smoothness_factors{0};
  std::size_t total_rotation_smoothness_factors{0};
  std::size_t total_retained_knot_position_prior_factors{0};
  std::size_t total_retained_knot_orientation_prior_factors{0};
  std::size_t total_spline_marginalization_priors{0};
  std::size_t total_spline_marginalization_prior_rows{0};
  std::size_t total_spline_orientation_marginalization_priors{0};
  std::size_t total_spline_orientation_marginalization_prior_rows{0};
  std::size_t total_marginalized_knots{0};
  std::size_t accepted_solver_steps{0};
  std::size_t last_step_imu_factors{0};
  std::size_t last_step_lidar_factors{0};
  std::size_t last_step_lidar_point_factors{0};
  std::size_t last_step_lidar_normal_factors{0};
  std::size_t last_step_position_prior_factors{0};
  std::size_t last_step_velocity_prior_factors{0};
  std::size_t last_step_angular_velocity_prior_factors{0};
  std::size_t last_step_orientation_prior_factors{0};
  std::size_t last_step_gyro_bias_prior_factors{0};
  std::size_t last_step_accel_bias_prior_factors{0};
  std::size_t last_step_position_smoothness_factors{0};
  std::size_t last_step_rotation_smoothness_factors{0};
  std::size_t last_step_retained_knot_position_prior_factors{0};
  std::size_t last_step_retained_knot_orientation_prior_factors{0};
  std::size_t last_step_spline_marginalization_prior_factors{0};
  std::size_t last_step_spline_marginalization_prior_rows{0};
  std::size_t last_step_spline_orientation_marginalization_prior_factors{0};
  std::size_t last_step_spline_orientation_marginalization_prior_rows{0};
  bool last_step_update_accepted{false};
  bool last_step_update_rejected{false};
  bool last_step_rotation_limited{false};
  bool last_step_position_limited{false};
  double last_step_initial_cost{0.0};
  double last_step_final_cost{0.0};
  double last_step_initial_imu_cost{0.0};
  double last_step_final_imu_cost{0.0};
  double last_step_initial_lidar_cost{0.0};
  double last_step_final_lidar_cost{0.0};
  double last_step_initial_position_prior_cost{0.0};
  double last_step_final_position_prior_cost{0.0};
  double last_step_initial_velocity_prior_cost{0.0};
  double last_step_final_velocity_prior_cost{0.0};
  double last_step_initial_orientation_prior_cost{0.0};
  double last_step_final_orientation_prior_cost{0.0};
  double last_step_initial_bias_prior_cost{0.0};
  double last_step_final_bias_prior_cost{0.0};
  double last_step_initial_smoothness_cost{0.0};
  double last_step_final_smoothness_cost{0.0};
  std::size_t rejected_solver_steps{0};
  std::size_t invalid_update_rejections{0};
  std::size_t position_update_rejections{0};
  std::size_t rotation_update_rejections{0};
  std::size_t rotation_limited_solver_steps{0};
  std::size_t position_limited_solver_steps{0};
  double last_step_max_position_update_m{0.0};
  double last_step_max_rotation_update_rad{0.0};
  double last_rejected_position_update_m{0.0};
  double last_rejected_rotation_update_rad{0.0};
  double last_rotation_limited_position_update_m{0.0};
  double last_rotation_limited_rotation_update_rad{0.0};
  double last_position_limited_position_update_m{0.0};
  double last_position_limited_rotation_update_rad{0.0};
};

class ContinuousTimeSlidingWindowEstimator
{
public:
  static constexpr int N = TrajectoryEstimator::N;

  explicit ContinuousTimeSlidingWindowEstimator(
    const ContinuousTimeSlidingWindowOptions & options);

  ~ContinuousTimeSlidingWindowEstimator();
  ContinuousTimeSlidingWindowEstimator(
    const ContinuousTimeSlidingWindowEstimator &) = delete;
  ContinuousTimeSlidingWindowEstimator & operator=(
    const ContinuousTimeSlidingWindowEstimator &) = delete;

  void initialize(
    int64_t start_stamp_ns,
    const std::vector<Eigen::Quaterniond> & rotation_knots,
    const std::vector<Eigen::Vector3d> & position_knots);

  // Adds a streaming IMU sample. The sample is buffered when its stamp
  // exceeds the current window's coverage and consumed on the next `step()`.
  void add_imu_sample(int64_t stamp_ns, const ImuSample & sample);

  // Adds a streaming LiDAR plane / edge correspondence (one per call).
  void add_lidar_correspondence(
    int64_t stamp_ns,
    const LidarPointCorrespondence & correspondence,
    const LidarExtrinsics & extrinsics,
    double weight = 1.0,
    double huber_delta_m = -1.0);

  void add_lidar_point_to_point_correspondence(
    int64_t stamp_ns,
    const Eigen::Vector3d & point_lidar,
    const Eigen::Vector3d & target_point_map,
    const LidarExtrinsics & extrinsics,
    double weight = 1.0,
    double huber_delta_m = -1.0,
    double scale = 1.0);

  void add_lidar_plane_normal_correspondence(
    int64_t stamp_ns,
    const Eigen::Vector3d & normal_lidar,
    const Eigen::Vector3d & normal_world,
    const LidarExtrinsics & extrinsics,
    double weight = 1.0,
    double huber_delta_rad = -1.0);

  // Adds a timestamped position-only prior. The prior uses the same
  // signed-nanosecond buffering and activation semantics as IMU/LiDAR
  // factors, but does not introduce a synthetic LiDAR geometry residual.
  void add_position_prior(
    int64_t stamp_ns,
    const Eigen::Vector3d & position_world,
    double weight = 1.0,
    double huber_delta_m = -1.0);

  // Adds a timestamped velocity prior in world coordinates. This constrains
  // local motion scale without anchoring global position.
  void add_velocity_prior(
    int64_t stamp_ns,
    const Eigen::Vector3d & velocity_world,
    double weight = 1.0,
    double huber_delta_mps = -1.0);

  void add_angular_velocity_prior(
    int64_t stamp_ns,
    const Eigen::Vector3d & angular_velocity_body,
    double weight = 1.0,
    double huber_delta_radps = -1.0);

  void add_relative_position_prior(
    int64_t start_stamp_ns,
    int64_t end_stamp_ns,
    const Eigen::Vector3d & target_p_body0,
    double weight = 1.0,
    double huber_delta_m = -1.0);

  void add_relative_orientation_prior(
    int64_t start_stamp_ns,
    int64_t end_stamp_ns,
    const Eigen::Quaterniond & target_q_body0_body1,
    double weight = 1.0,
    double huber_delta_rad = -1.0);

  // Adds a timestamped SO(3) orientation prior with the same buffering
  // semantics as the position prior.
  void add_orientation_prior(
    int64_t stamp_ns,
    const Eigen::Quaterniond & q_world_body,
    double weight = 1.0,
    double huber_delta_rad = -1.0);

  // Applies a bounded, time-tapered SE(3) pose hint to the active spline
  // window so frontend odometry can seed the next online solve without
  // rigidly dragging older knots. Gains are clamped to [0, 1]. Returns false
  // when the stamp is outside the queryable window or the target pose is
  // invalid.
  bool apply_pose_hint(
    int64_t stamp_ns,
    const Eigen::Quaterniond & q_world_body,
    const Eigen::Vector3d & position_world,
    double position_gain = 1.0,
    double rotation_gain = 1.0);

  // Advances the window by one knot, absorbs all buffered samples that fit
  // inside the new window, optionally marginalizes the oldest knot(s), and
  // runs Ceres on the active window. Returns false when the window cannot
  // be extended (no buffered samples to motivate the new knot).
  bool step();

  // Look up the body pose at a stamp inside the active window.
  bool query_pose(int64_t stamp_ns, Eigen::Quaterniond & q_w_b, Eigen::Vector3d & p_w_b) const;

  int64_t newest_knot_stamp_ns() const;
  int64_t oldest_active_knot_stamp_ns() const;
  std::size_t active_knot_count() const;
  std::size_t buffered_imu_count() const;
  std::size_t buffered_lidar_count() const;

  const Eigen::Vector3d & gyro_bias() const;
  const Eigen::Vector3d & accel_bias() const;
  const Eigen::Vector3d & gravity_world() const;
  void set_gyro_bias(const Eigen::Vector3d & gyro_bias);
  void set_accel_bias(const Eigen::Vector3d & accel_bias);

  const ContinuousTimeSlidingWindowDiagnostics & diagnostics() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace spline
}  // namespace gaussian_lic_tracking
