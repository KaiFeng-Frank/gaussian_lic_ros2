// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_tracking/lidar_factor.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <Eigen/Eigenvalues>
#include <Eigen/SVD>

namespace gaussian_lic_tracking
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

bool pose_is_finite(const TrajectoryPose & pose)
{
  return pose.p_w_i.allFinite() && pose.q_w_i.coeffs().allFinite() &&
         pose.q_w_i.norm() > std::numeric_limits<double>::epsilon();
}

double correspondence_confidence(const double residual_norm, const double robust_kernel)
{
  if (!std::isfinite(residual_norm) || residual_norm < 0.0) {
    return 0.0;
  }
  if (residual_norm <= std::numeric_limits<double>::epsilon()) {
    return 1.0;
  }
  return std::clamp(robust_kernel / residual_norm, 0.0, 1.0);
}
}

LidarFactor::LidarFactor(LidarFactorConfig config)
{
  set_config(config);
}

void LidarFactor::set_config(const LidarFactorConfig & config)
{
  if (!std::isfinite(config.nearest_distance_m) || !std::isfinite(config.correction_gain) ||
    !std::isfinite(config.max_correction_m) || !std::isfinite(config.max_rotation_rad) ||
    !std::isfinite(config.robust_kernel_m) || !std::isfinite(config.plane_max_condition))
  {
    throw std::runtime_error("LiDAR factor numeric config values must be finite");
  }
  if (config.min_points == 0U) {
    throw std::runtime_error("LiDAR factor min_points must be positive");
  }
  if (config.nearest_distance_m <= 0.0) {
    throw std::runtime_error("LiDAR factor nearest distance must be positive");
  }
  if (config.correction_gain < 0.0 || config.correction_gain > 1.0) {
    throw std::runtime_error("LiDAR factor correction_gain must be in [0, 1]");
  }
  if (config.max_correction_m <= 0.0) {
    throw std::runtime_error("LiDAR factor max correction must be positive");
  }
  if (config.max_rotation_rad <= 0.0) {
    throw std::runtime_error("LiDAR factor max rotation correction must be positive");
  }
  if (config.robust_kernel_m <= 0.0) {
    throw std::runtime_error("LiDAR factor robust kernel must be positive");
  }
  if (config.plane_min_neighbors < 3U) {
    throw std::runtime_error("LiDAR plane factor needs at least three neighbors");
  }
  if (config.plane_max_condition <= 0.0) {
    throw std::runtime_error("LiDAR plane condition threshold must be positive");
  }
  config_ = config;
}

void LidarFactor::clear()
{
  map_points_w_.clear();
}

std::vector<Eigen::Vector3d> LidarFactor::sample_points(
  const std::vector<Eigen::Vector3d> & points,
  const size_t max_points)
{
  std::vector<Eigen::Vector3d> sampled;
  sampled.reserve(max_points == 0U ? points.size() : std::min(points.size(), max_points));
  const size_t stride = max_points == 0U || points.size() <= max_points
    ? 1U
    : static_cast<size_t>(
      std::ceil(static_cast<double>(points.size()) / static_cast<double>(max_points)));
  for (size_t index = 0; index < points.size() && sampled.size() < max_points; index += stride) {
    if (points[index].allFinite()) {
      sampled.push_back(points[index]);
    }
  }
  if (max_points == 0U) {
    for (size_t index = sampled.size(); index < points.size(); ++index) {
      if (points[index].allFinite()) {
        sampled.push_back(points[index]);
      }
    }
  }
  return sampled;
}

LidarCorrection LidarFactor::compute_translation_correction(
  const std::vector<Eigen::Vector3d> & frame_points_i,
  const TrajectoryPose & predicted_pose) const
{
  LidarCorrection correction;
  if (!pose_is_finite(predicted_pose)) {
    return correction;
  }

  const auto sampled = sample_points(frame_points_i, config_.max_frame_points);
  if (sampled.size() < config_.min_points || map_points_w_.size() < config_.min_points) {
    return correction;
  }
  const double max_distance_sq = config_.nearest_distance_m * config_.nearest_distance_m;
  Eigen::Vector3d residual_sum = Eigen::Vector3d::Zero();
  double residual_norm_sum = 0.0;
  size_t matches = 0U;

  for (const auto & point_i : sampled) {
    const Eigen::Vector3d point_w = predicted_pose.q_w_i * point_i + predicted_pose.p_w_i;
    double best_distance_sq = std::numeric_limits<double>::infinity();
    Eigen::Vector3d best_point_w = Eigen::Vector3d::Zero();
    for (const auto & map_point_w : map_points_w_) {
      const double distance_sq = (map_point_w - point_w).squaredNorm();
      if (distance_sq < best_distance_sq) {
        best_distance_sq = distance_sq;
        best_point_w = map_point_w;
      }
    }
    if (best_distance_sq <= max_distance_sq) {
      const Eigen::Vector3d residual = best_point_w - point_w;
      residual_sum += residual;
      residual_norm_sum += std::sqrt(best_distance_sq);
      ++matches;
    }
  }

  if (matches < config_.min_points) {
    return correction;
  }

  correction.applied = true;
  correction.matched_points = matches;
  correction.mean_residual_m = residual_norm_sum / static_cast<double>(matches);
  correction.delta_p_w = config_.correction_gain * residual_sum / static_cast<double>(matches);
  const double norm = correction.delta_p_w.norm();
  if (norm > config_.max_correction_m) {
    correction.delta_p_w *= config_.max_correction_m / norm;
  }
  return correction;
}

LidarPoseCorrection LidarFactor::compute_pose_correction(
  const std::vector<Eigen::Vector3d> & frame_points_i,
  const TrajectoryPose & predicted_pose) const
{
  LidarPoseCorrection correction;
  if (!pose_is_finite(predicted_pose)) {
    return correction;
  }

  const auto sampled = sample_points(frame_points_i, config_.max_frame_points);
  if (sampled.size() < config_.min_points || map_points_w_.size() < config_.min_points) {
    return correction;
  }
  const double max_distance_sq = config_.nearest_distance_m * config_.nearest_distance_m;
  std::vector<Eigen::Vector3d> source_w;
  std::vector<Eigen::Vector3d> target_w;
  std::vector<double> match_weights;
  source_w.reserve(sampled.size());
  target_w.reserve(sampled.size());
  match_weights.reserve(sampled.size());
  double residual_norm_sum = 0.0;
  double match_weight_sum = 0.0;

  for (const auto & point_i : sampled) {
    const Eigen::Vector3d point_w = predicted_pose.q_w_i * point_i + predicted_pose.p_w_i;
    double best_distance_sq = std::numeric_limits<double>::infinity();
    Eigen::Vector3d best_point_w = Eigen::Vector3d::Zero();
    for (const auto & map_point_w : map_points_w_) {
      const double distance_sq = (map_point_w - point_w).squaredNorm();
      if (distance_sq < best_distance_sq) {
        best_distance_sq = distance_sq;
        best_point_w = map_point_w;
      }
    }
    if (best_distance_sq <= max_distance_sq) {
      const double residual_norm = std::sqrt(best_distance_sq);
      const double match_weight =
        std::min(1.0, config_.robust_kernel_m / std::max(residual_norm, 1.0e-12));
      source_w.push_back(point_w);
      target_w.push_back(best_point_w);
      match_weights.push_back(match_weight);
      residual_norm_sum += match_weight * residual_norm;
      match_weight_sum += match_weight;
    }
  }

  if (source_w.size() < config_.min_points ||
    match_weight_sum <= std::numeric_limits<double>::epsilon())
  {
    return correction;
  }

  Eigen::Vector3d source_centroid = Eigen::Vector3d::Zero();
  Eigen::Vector3d target_centroid = Eigen::Vector3d::Zero();
  for (size_t index = 0; index < source_w.size(); ++index) {
    source_centroid += match_weights[index] * source_w[index];
    target_centroid += match_weights[index] * target_w[index];
  }
  source_centroid /= match_weight_sum;
  target_centroid /= match_weight_sum;

  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (size_t index = 0; index < source_w.size(); ++index) {
    covariance += match_weights[index] *
      (target_w[index] - target_centroid) * (source_w[index] - source_centroid).transpose();
  }

  const Eigen::JacobiSVD<Eigen::Matrix3d> svd(
    covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d u = svd.matrixU();
  const Eigen::Matrix3d v = svd.matrixV();
  Eigen::Matrix3d rotation = u * v.transpose();
  if (rotation.determinant() < 0.0) {
    u.col(2) *= -1.0;
    rotation = u * v.transpose();
  }

  Eigen::Quaterniond delta_q(rotation);
  delta_q.normalize();
  Eigen::AngleAxisd delta_aa(delta_q);
  double angle = delta_aa.angle();
  if (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  if (std::abs(angle) > config_.max_rotation_rad) {
    const double clamped = std::copysign(config_.max_rotation_rad, angle);
    delta_q = Eigen::Quaterniond(Eigen::AngleAxisd(clamped, delta_aa.axis())).normalized();
    rotation = delta_q.toRotationMatrix();
  }

  Eigen::Vector3d correction_translation = target_centroid - rotation * source_centroid;
  Eigen::Vector3d delta_p =
    (rotation * predicted_pose.p_w_i + correction_translation) - predicted_pose.p_w_i;
  delta_p *= config_.correction_gain;
  const double delta_norm = delta_p.norm();
  if (delta_norm > config_.max_correction_m) {
    delta_p *= config_.max_correction_m / delta_norm;
  }
  if (config_.correction_gain < 1.0) {
    Eigen::AngleAxisd gained_aa(delta_q);
    if (gained_aa.angle() > 1.0e-12) {
      delta_q = Eigen::Quaterniond(
        Eigen::AngleAxisd(config_.correction_gain * gained_aa.angle(), gained_aa.axis())).normalized();
    }
  }

  correction.applied = true;
  correction.matched_points = source_w.size();
  correction.mean_residual_m = residual_norm_sum / match_weight_sum;
  correction.delta_p_w = delta_p;
  correction.delta_q = delta_q;
  return correction;
}

SlidingWindowPointToPointFactor LidarFactor::build_point_to_point_factor(
  const std::vector<Eigen::Vector3d> & frame_points_i,
  const TrajectoryPose & predicted_pose) const
{
  SlidingWindowPointToPointFactor factor;
  factor.stamp_ns = predicted_pose.stamp_ns;
  factor.source_id = 1U;
  if (!pose_is_finite(predicted_pose)) {
    return factor;
  }

  const auto sampled = sample_points(frame_points_i, config_.max_frame_points);
  if (sampled.size() < config_.min_points || map_points_w_.size() < config_.min_points) {
    return factor;
  }
  const double max_distance_sq = config_.nearest_distance_m * config_.nearest_distance_m;
  factor.frame_points_i.reserve(sampled.size());
  factor.target_points_w.reserve(sampled.size());
  factor.point_weights.reserve(sampled.size());
  factor.weight = 1.0 / std::max(config_.nearest_distance_m * config_.nearest_distance_m, 1.0e-12);
  factor.huber_delta_m = config_.robust_kernel_m;

  for (const auto & point_i : sampled) {
    const Eigen::Vector3d point_w = predicted_pose.q_w_i * point_i + predicted_pose.p_w_i;
    double best_distance_sq = std::numeric_limits<double>::infinity();
    Eigen::Vector3d best_point_w = Eigen::Vector3d::Zero();
    for (const auto & map_point_w : map_points_w_) {
      const double distance_sq = (map_point_w - point_w).squaredNorm();
      if (distance_sq < best_distance_sq) {
        best_distance_sq = distance_sq;
        best_point_w = map_point_w;
      }
    }
    if (best_distance_sq <= max_distance_sq) {
      const double residual_norm = std::sqrt(best_distance_sq);
      const double confidence = correspondence_confidence(residual_norm, config_.robust_kernel_m);
      if (confidence <= 0.0) {
        continue;
      }
      factor.frame_points_i.push_back(point_i);
      factor.target_points_w.push_back(best_point_w);
      factor.point_weights.push_back(confidence);
    }
  }
  if (factor.frame_points_i.size() < config_.min_points) {
    factor.frame_points_i.clear();
    factor.target_points_w.clear();
    factor.point_weights.clear();
  }
  return factor;
}

SlidingWindowPointToPlaneFactor LidarFactor::build_point_to_plane_factor(
  const std::vector<Eigen::Vector3d> & frame_points_i,
  const TrajectoryPose & predicted_pose) const
{
  SlidingWindowPointToPlaneFactor factor;
  factor.stamp_ns = predicted_pose.stamp_ns;
  factor.source_id = 1U;
  if (!pose_is_finite(predicted_pose)) {
    return factor;
  }

  const auto sampled = sample_points(frame_points_i, config_.max_frame_points);
  if (sampled.size() < config_.min_points || map_points_w_.size() < config_.plane_min_neighbors) {
    return factor;
  }
  const double max_distance_sq = config_.nearest_distance_m * config_.nearest_distance_m;
  factor.frame_points_i.reserve(sampled.size());
  factor.target_points_w.reserve(sampled.size());
  factor.target_normals_w.reserve(sampled.size());
  factor.point_weights.reserve(sampled.size());
  factor.weight = 1.0 / std::max(config_.nearest_distance_m * config_.nearest_distance_m, 1.0e-12);
  factor.huber_delta_m = config_.robust_kernel_m;

  for (const auto & point_i : sampled) {
    const Eigen::Vector3d point_w = predicted_pose.q_w_i * point_i + predicted_pose.p_w_i;
    std::vector<std::pair<double, Eigen::Vector3d>> neighbors;
    neighbors.reserve(config_.plane_min_neighbors);
    for (const auto & map_point_w : map_points_w_) {
      const double distance_sq = (map_point_w - point_w).squaredNorm();
      if (distance_sq > max_distance_sq) {
        continue;
      }
      neighbors.emplace_back(distance_sq, map_point_w);
      std::sort(
        neighbors.begin(), neighbors.end(),
        [](const auto & lhs, const auto & rhs) {
          return lhs.first < rhs.first;
        });
      if (neighbors.size() > config_.plane_min_neighbors) {
        neighbors.pop_back();
      }
    }
    if (neighbors.size() < config_.plane_min_neighbors) {
      continue;
    }

    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const auto & neighbor : neighbors) {
      centroid += neighbor.second;
    }
    centroid /= static_cast<double>(neighbors.size());
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const auto & neighbor : neighbors) {
      const Eigen::Vector3d centered = neighbor.second - centroid;
      covariance += centered * centered.transpose();
    }
    covariance /= static_cast<double>(neighbors.size());
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success) {
      continue;
    }
    const auto eigenvalues = solver.eigenvalues();
    const double condition = eigenvalues[1] > 1.0e-12
      ? eigenvalues[0] / eigenvalues[1]
      : std::numeric_limits<double>::infinity();
    if (condition > config_.plane_max_condition) {
      continue;
    }
    Eigen::Vector3d normal = solver.eigenvectors().col(0).normalized();
    const double signed_distance = normal.dot(point_w - centroid);
    if (std::abs(signed_distance) > config_.nearest_distance_m) {
      continue;
    }
    const double confidence = correspondence_confidence(std::abs(signed_distance), config_.robust_kernel_m);
    if (confidence <= 0.0) {
      continue;
    }
    if (signed_distance > 0.0) {
      normal *= -1.0;
    }
    factor.frame_points_i.push_back(point_i);
    factor.target_points_w.push_back(centroid);
    factor.target_normals_w.push_back(normal);
    factor.point_weights.push_back(confidence);
  }
  if (factor.frame_points_i.size() < config_.min_points) {
    factor.frame_points_i.clear();
    factor.target_points_w.clear();
    factor.target_normals_w.clear();
    factor.point_weights.clear();
  }
  return factor;
}

void LidarFactor::insert_keyframe(
  const std::vector<Eigen::Vector3d> & frame_points_i,
  const TrajectoryPose & pose)
{
  if (!pose_is_finite(pose)) {
    return;
  }
  const auto sampled = sample_points(frame_points_i, config_.max_frame_points);
  if (sampled.size() < config_.min_points) {
    return;
  }
  map_points_w_.reserve(std::min(config_.max_map_points, map_points_w_.size() + sampled.size()));
  for (const auto & point_i : sampled) {
    map_points_w_.push_back(pose.q_w_i * point_i + pose.p_w_i);
  }
  if (config_.max_map_points > 0U && map_points_w_.size() > config_.max_map_points) {
    const size_t overflow = map_points_w_.size() - config_.max_map_points;
    map_points_w_.erase(map_points_w_.begin(), map_points_w_.begin() + static_cast<std::ptrdiff_t>(overflow));
  }
}

}  // namespace gaussian_lic_tracking
