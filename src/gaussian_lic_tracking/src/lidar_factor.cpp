// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_tracking/lidar_factor.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <Eigen/Cholesky>
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

double plane_condition_confidence(const double condition, const double max_condition)
{
  if (!std::isfinite(condition) || !std::isfinite(max_condition) || max_condition <= 0.0 ||
    condition < 0.0 || condition >= max_condition)
  {
    return 0.0;
  }
  return std::clamp(1.0 - condition / max_condition, 0.0, 1.0);
}

Eigen::Matrix3d skew_matrix(const Eigen::Vector3d & value)
{
  Eigen::Matrix3d skew;
  skew << 0.0, -value.z(), value.y(),
    value.z(), 0.0, -value.x(),
    -value.y(), value.x(), 0.0;
  return skew;
}

Eigen::Quaterniond quaternion_from_tangent(const Eigen::Vector3d & tangent)
{
  const double angle = tangent.norm();
  if (!std::isfinite(angle) || angle <= 1.0e-12) {
    return Eigen::Quaterniond::Identity();
  }
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, tangent / angle)).normalized();
}

bool fit_local_plane(
  const std::vector<std::pair<double, Eigen::Vector3d>> & neighbors,
  const Eigen::Vector3d & point_w,
  const LidarFactorConfig & config,
  Eigen::Vector3d & centroid,
  Eigen::Vector3d & normal,
  double & signed_distance,
  double & confidence)
{
  if (neighbors.size() < config.plane_min_neighbors) {
    return false;
  }
  centroid.setZero();
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
    return false;
  }
  const auto eigenvalues = solver.eigenvalues();
  const double condition = eigenvalues[1] > 1.0e-12
    ? eigenvalues[0] / eigenvalues[1]
    : std::numeric_limits<double>::infinity();
  if (condition > config.plane_max_condition) {
    return false;
  }

  normal = solver.eigenvectors().col(0).normalized();
  signed_distance = normal.dot(point_w - centroid);
  if (std::abs(signed_distance) > config.nearest_distance_m) {
    return false;
  }
  confidence =
    correspondence_confidence(std::abs(signed_distance), config.robust_kernel_m) *
    plane_condition_confidence(condition, config.plane_max_condition);
  return confidence > 0.0;
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
  if (config.pose_iterations == 0U) {
    throw std::runtime_error("LiDAR pose correction iterations must be positive");
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
  if (!map_points_w_.empty()) {
    rebuild_spatial_index();
  }
}

void LidarFactor::clear()
{
  map_points_w_.clear();
  map_voxels_.clear();
  voxel_size_m_ = 0.0;
}

size_t LidarFactor::VoxelKeyHash::operator()(const VoxelKey & key) const
{
  size_t seed = 1469598103934665603ULL;
  const auto mix = [&seed](const int64_t value) {
    const size_t hashed = std::hash<int64_t>{}(value);
    seed ^= hashed + 0x9e3779b97f4a7c15ULL + (seed << 6U) + (seed >> 2U);
  };
  mix(key.x);
  mix(key.y);
  mix(key.z);
  return seed;
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

LidarFactor::VoxelKey LidarFactor::voxel_key_for_point(
  const Eigen::Vector3d & point,
  const double voxel_size)
{
  return VoxelKey{
    static_cast<int64_t>(std::floor(point.x() / voxel_size)),
    static_cast<int64_t>(std::floor(point.y() / voxel_size)),
    static_cast<int64_t>(std::floor(point.z() / voxel_size))};
}

void LidarFactor::rebuild_spatial_index()
{
  map_voxels_.clear();
  voxel_size_m_ = std::max(config_.nearest_distance_m, 1.0e-6);
  map_voxels_.reserve(map_points_w_.size());
  for (size_t index = 0U; index < map_points_w_.size(); ++index) {
    if (!map_points_w_[index].allFinite()) {
      continue;
    }
    map_voxels_[voxel_key_for_point(map_points_w_[index], voxel_size_m_)].push_back(index);
  }
}

bool LidarFactor::find_nearest_map_point(
  const Eigen::Vector3d & point_w,
  const double max_distance_sq,
  Eigen::Vector3d & best_point_w,
  double & best_distance_sq) const
{
  if (map_voxels_.empty() || voxel_size_m_ <= 0.0 || !point_w.allFinite()) {
    return false;
  }
  const VoxelKey center = voxel_key_for_point(point_w, voxel_size_m_);
  best_distance_sq = std::numeric_limits<double>::infinity();
  for (int64_t dx = -1; dx <= 1; ++dx) {
    for (int64_t dy = -1; dy <= 1; ++dy) {
      for (int64_t dz = -1; dz <= 1; ++dz) {
        const auto bucket = map_voxels_.find(VoxelKey{center.x + dx, center.y + dy, center.z + dz});
        if (bucket == map_voxels_.end()) {
          continue;
        }
        for (const size_t index : bucket->second) {
          const double distance_sq = (map_points_w_[index] - point_w).squaredNorm();
          if (distance_sq < best_distance_sq) {
            best_distance_sq = distance_sq;
            best_point_w = map_points_w_[index];
          }
        }
      }
    }
  }
  return best_distance_sq <= max_distance_sq;
}

std::vector<std::pair<double, Eigen::Vector3d>> LidarFactor::find_nearest_map_neighbors(
  const Eigen::Vector3d & point_w,
  const double max_distance_sq,
  const size_t max_neighbors) const
{
  std::vector<std::pair<double, Eigen::Vector3d>> neighbors;
  if (map_voxels_.empty() || voxel_size_m_ <= 0.0 || max_neighbors == 0U || !point_w.allFinite()) {
    return neighbors;
  }
  const VoxelKey center = voxel_key_for_point(point_w, voxel_size_m_);
  neighbors.reserve(max_neighbors);
  for (int64_t dx = -1; dx <= 1; ++dx) {
    for (int64_t dy = -1; dy <= 1; ++dy) {
      for (int64_t dz = -1; dz <= 1; ++dz) {
        const auto bucket = map_voxels_.find(VoxelKey{center.x + dx, center.y + dy, center.z + dz});
        if (bucket == map_voxels_.end()) {
          continue;
        }
        for (const size_t index : bucket->second) {
          const double distance_sq = (map_points_w_[index] - point_w).squaredNorm();
          if (distance_sq > max_distance_sq) {
            continue;
          }
          neighbors.emplace_back(distance_sq, map_points_w_[index]);
          std::sort(
            neighbors.begin(), neighbors.end(),
            [](const auto & lhs, const auto & rhs) {
              return lhs.first < rhs.first;
            });
          if (neighbors.size() > max_neighbors) {
            neighbors.pop_back();
          }
        }
      }
    }
  }
  return neighbors;
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
    if (find_nearest_map_point(point_w, max_distance_sq, best_point_w, best_distance_sq)) {
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
  Eigen::Quaterniond q_est = predicted_pose.q_w_i.normalized();
  Eigen::Vector3d p_est = predicted_pose.p_w_i;
  size_t last_matches = 0U;
  double last_mean_residual = 0.0;
  const auto estimate_mean_residual =
    [&](const Eigen::Quaterniond & q_w_i, const Eigen::Vector3d & p_w_i,
      size_t & matches, double & mean_residual) -> bool
    {
      matches = 0U;
      mean_residual = 0.0;
      double residual_sum = 0.0;
      double weight_sum = 0.0;
      for (const auto & point_i : sampled) {
        const Eigen::Vector3d point_w = q_w_i * point_i + p_w_i;
        double best_distance_sq = std::numeric_limits<double>::infinity();
        Eigen::Vector3d best_point_w = Eigen::Vector3d::Zero();
        if (!find_nearest_map_point(point_w, max_distance_sq, best_point_w, best_distance_sq)) {
          continue;
        }
        const double residual_norm = std::sqrt(best_distance_sq);
        const double match_weight =
          std::min(1.0, config_.robust_kernel_m / std::max(residual_norm, 1.0e-12));
        residual_sum += match_weight * residual_norm;
        weight_sum += match_weight;
        ++matches;
      }
      if (matches < config_.min_points || weight_sum <= std::numeric_limits<double>::epsilon()) {
        return false;
      }
      mean_residual = residual_sum / weight_sum;
      return true;
    };

  for (size_t iteration = 0U; iteration < config_.pose_iterations; ++iteration) {
    std::vector<Eigen::Vector3d> source_w;
    std::vector<Eigen::Vector3d> target_w;
    std::vector<double> match_weights;
    source_w.reserve(sampled.size());
    target_w.reserve(sampled.size());
    match_weights.reserve(sampled.size());
    double residual_norm_sum = 0.0;
    double match_weight_sum = 0.0;

    for (const auto & point_i : sampled) {
      const Eigen::Vector3d point_w = q_est * point_i + p_est;
      double best_distance_sq = std::numeric_limits<double>::infinity();
      Eigen::Vector3d best_point_w = Eigen::Vector3d::Zero();
      if (find_nearest_map_point(point_w, max_distance_sq, best_point_w, best_distance_sq)) {
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
      break;
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

    Eigen::Quaterniond step_q(rotation);
    step_q.normalize();
    Eigen::AngleAxisd step_aa(step_q);
    double angle = step_aa.angle();
    if (angle > kPi) {
      angle -= 2.0 * kPi;
    }
    if (std::abs(angle) > config_.max_rotation_rad) {
      const double clamped = std::copysign(config_.max_rotation_rad, angle);
      step_q = Eigen::Quaterniond(Eigen::AngleAxisd(clamped, step_aa.axis())).normalized();
      rotation = step_q.toRotationMatrix();
    }

    const Eigen::Vector3d correction_translation = target_centroid - rotation * source_centroid;
    Eigen::Vector3d step_p = (rotation * p_est + correction_translation) - p_est;
    step_p *= config_.correction_gain;
    const double step_p_norm = step_p.norm();
    if (step_p_norm > config_.max_correction_m) {
      step_p *= config_.max_correction_m / step_p_norm;
    }
    if (config_.correction_gain < 1.0) {
      Eigen::AngleAxisd gained_aa(step_q);
      if (gained_aa.angle() > 1.0e-12) {
        step_q = Eigen::Quaterniond(
          Eigen::AngleAxisd(config_.correction_gain * gained_aa.angle(), gained_aa.axis())).normalized();
      }
    }

    Eigen::Quaterniond next_q = (step_q * q_est).normalized();
    Eigen::Vector3d next_p = p_est + step_p;
    const Eigen::Vector3d total_p = next_p - predicted_pose.p_w_i;
    const double total_p_norm = total_p.norm();
    if (total_p_norm > config_.max_correction_m) {
      next_p = predicted_pose.p_w_i + total_p * (config_.max_correction_m / total_p_norm);
    }
    Eigen::Quaterniond total_q =
      (next_q * predicted_pose.q_w_i.normalized().inverse()).normalized();
    Eigen::AngleAxisd total_aa(total_q);
    double total_angle = total_aa.angle();
    if (total_angle > kPi) {
      total_angle -= 2.0 * kPi;
    }
    if (std::abs(total_angle) > config_.max_rotation_rad) {
      const double clamped = std::copysign(config_.max_rotation_rad, total_angle);
      total_q = Eigen::Quaterniond(Eigen::AngleAxisd(clamped, total_aa.axis())).normalized();
      next_q = (total_q * predicted_pose.q_w_i.normalized()).normalized();
    }
    const double current_mean_residual = residual_norm_sum / match_weight_sum;
    if (iteration > 0U) {
      size_t candidate_matches = 0U;
      double candidate_mean_residual = 0.0;
      if (!estimate_mean_residual(next_q, next_p, candidate_matches, candidate_mean_residual) ||
        candidate_mean_residual > current_mean_residual * 1.05)
      {
        break;
      }
    }
    q_est = next_q;
    p_est = next_p;
    last_matches = source_w.size();
    last_mean_residual = current_mean_residual;

    const double step_angle = std::abs(Eigen::AngleAxisd(step_q).angle());
    if (step_p.norm() < 1.0e-6 && step_angle < 1.0e-6) {
      break;
    }
  }

  if (last_matches < config_.min_points) {
    return correction;
  }

  correction.applied = true;
  correction.matched_points = last_matches;
  correction.mean_residual_m = last_mean_residual;
  correction.delta_p_w = p_est - predicted_pose.p_w_i;
  correction.delta_q = (q_est * predicted_pose.q_w_i.normalized().inverse()).normalized();
  return correction;
}

LidarPoseCorrection LidarFactor::compute_point_to_plane_pose_correction(
  const std::vector<Eigen::Vector3d> & frame_points_i,
  const TrajectoryPose & predicted_pose) const
{
  LidarPoseCorrection correction;
  if (!pose_is_finite(predicted_pose)) {
    return correction;
  }

  const auto sampled = sample_points(frame_points_i, config_.max_frame_points);
  if (sampled.size() < config_.min_points || map_points_w_.size() < config_.plane_min_neighbors) {
    return correction;
  }

  const double max_distance_sq = config_.nearest_distance_m * config_.nearest_distance_m;
  Eigen::Quaterniond q_est = predicted_pose.q_w_i.normalized();
  Eigen::Vector3d p_est = predicted_pose.p_w_i;
  size_t last_matches = 0U;
  double last_mean_abs_residual = 0.0;

  const auto evaluate_pose =
    [&](const Eigen::Quaterniond & q_w_i, const Eigen::Vector3d & p_w_i,
      size_t & matches, double & mean_abs_residual) -> bool
    {
      matches = 0U;
      mean_abs_residual = 0.0;
      double weighted_residual_sum = 0.0;
      double weight_sum = 0.0;
      for (const auto & point_i : sampled) {
        const Eigen::Vector3d point_w = q_w_i * point_i + p_w_i;
        const auto neighbors =
          find_nearest_map_neighbors(point_w, max_distance_sq, config_.plane_min_neighbors);
        Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
        Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
        double signed_distance = 0.0;
        double confidence = 0.0;
        if (!fit_local_plane(
            neighbors, point_w, config_, centroid, normal, signed_distance, confidence))
        {
          continue;
        }
        weighted_residual_sum += confidence * std::abs(signed_distance);
        weight_sum += confidence;
        ++matches;
      }
      if (matches < config_.min_points || weight_sum <= std::numeric_limits<double>::epsilon()) {
        return false;
      }
      mean_abs_residual = weighted_residual_sum / weight_sum;
      return true;
    };

  for (size_t iteration = 0U; iteration < config_.pose_iterations; ++iteration) {
    Eigen::Matrix<double, 6, 6> hessian = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> gradient = Eigen::Matrix<double, 6, 1>::Zero();
    double weighted_residual_sum = 0.0;
    double weight_sum = 0.0;
    size_t matches = 0U;

    for (const auto & point_i : sampled) {
      const Eigen::Vector3d point_i_rotated = q_est * point_i;
      const Eigen::Vector3d point_w = point_i_rotated + p_est;
      const auto neighbors =
        find_nearest_map_neighbors(point_w, max_distance_sq, config_.plane_min_neighbors);
      Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
      Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
      double signed_distance = 0.0;
      double confidence = 0.0;
      if (!fit_local_plane(
          neighbors, point_w, config_, centroid, normal, signed_distance, confidence))
      {
        continue;
      }

      Eigen::Matrix<double, 1, 6> jacobian;
      jacobian.template block<1, 3>(0, 0) = -normal.transpose() * skew_matrix(point_i_rotated);
      jacobian.template block<1, 3>(0, 3) = normal.transpose();
      const double weight = std::max(confidence, 1.0e-6);
      hessian.noalias() += weight * jacobian.transpose() * jacobian;
      gradient.noalias() += weight * jacobian.transpose() * signed_distance;
      weighted_residual_sum += weight * std::abs(signed_distance);
      weight_sum += weight;
      ++matches;
    }

    if (matches < config_.min_points || weight_sum <= std::numeric_limits<double>::epsilon()) {
      break;
    }

    const double current_mean_residual = weighted_residual_sum / weight_sum;
    hessian.diagonal().array() += 1.0e-6;
    const Eigen::LDLT<Eigen::Matrix<double, 6, 6>> ldlt(hessian);
    if (ldlt.info() != Eigen::Success) {
      break;
    }
    Eigen::Matrix<double, 6, 1> step = ldlt.solve(-gradient);
    if (!step.allFinite()) {
      break;
    }
    step *= config_.correction_gain;

    Eigen::Vector3d rotation_step = step.template head<3>();
    Eigen::Vector3d translation_step = step.template tail<3>();
    const double rotation_norm = rotation_step.norm();
    if (rotation_norm > config_.max_rotation_rad && rotation_norm > 1.0e-12) {
      rotation_step *= config_.max_rotation_rad / rotation_norm;
    }
    const double translation_norm = translation_step.norm();
    if (translation_norm > config_.max_correction_m && translation_norm > 1.0e-12) {
      translation_step *= config_.max_correction_m / translation_norm;
    }

    Eigen::Quaterniond next_q =
      (quaternion_from_tangent(rotation_step) * q_est).normalized();
    Eigen::Vector3d next_p = p_est + translation_step;
    const Eigen::Vector3d total_p = next_p - predicted_pose.p_w_i;
    const double total_p_norm = total_p.norm();
    if (total_p_norm > config_.max_correction_m && total_p_norm > 1.0e-12) {
      next_p = predicted_pose.p_w_i + total_p * (config_.max_correction_m / total_p_norm);
    }
    Eigen::Quaterniond total_q =
      (next_q * predicted_pose.q_w_i.normalized().inverse()).normalized();
    Eigen::AngleAxisd total_aa(total_q);
    double total_angle = total_aa.angle();
    if (total_angle > kPi) {
      total_angle -= 2.0 * kPi;
    }
    if (std::abs(total_angle) > config_.max_rotation_rad) {
      const double clamped = std::copysign(config_.max_rotation_rad, total_angle);
      total_q = Eigen::Quaterniond(Eigen::AngleAxisd(clamped, total_aa.axis())).normalized();
      next_q = (total_q * predicted_pose.q_w_i.normalized()).normalized();
    }

    size_t candidate_matches = 0U;
    double candidate_mean_residual = 0.0;
    if (!evaluate_pose(next_q, next_p, candidate_matches, candidate_mean_residual) ||
      candidate_mean_residual > current_mean_residual * 1.05)
    {
      break;
    }

    q_est = next_q;
    p_est = next_p;
    last_matches = candidate_matches;
    last_mean_abs_residual = candidate_mean_residual;

    if (translation_step.norm() < 1.0e-6 && rotation_step.norm() < 1.0e-6) {
      break;
    }
  }

  if (last_matches < config_.min_points) {
    return correction;
  }

  correction.applied = true;
  correction.matched_points = last_matches;
  correction.mean_residual_m = last_mean_abs_residual;
  correction.delta_p_w = p_est - predicted_pose.p_w_i;
  correction.delta_q = (q_est * predicted_pose.q_w_i.normalized().inverse()).normalized();
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
    if (find_nearest_map_point(point_w, max_distance_sq, best_point_w, best_distance_sq)) {
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
    const auto neighbors =
      find_nearest_map_neighbors(point_w, max_distance_sq, config_.plane_min_neighbors);
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
    const double confidence =
      correspondence_confidence(std::abs(signed_distance), config_.robust_kernel_m) *
      plane_condition_confidence(condition, config_.plane_max_condition);
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

SlidingWindowPointToLineFactor LidarFactor::build_point_to_line_factor(
  const std::vector<Eigen::Vector3d> & frame_points_i,
  const TrajectoryPose & predicted_pose) const
{
  SlidingWindowPointToLineFactor factor;
  factor.stamp_ns = predicted_pose.stamp_ns;
  factor.source_id = 2U;  // distinct from the plane factor (source_id 1U)
  if (!config_.enable_line_factor || !pose_is_finite(predicted_pose)) {
    return factor;
  }
  const auto sampled = sample_points(frame_points_i, config_.max_frame_points);
  if (sampled.size() < config_.min_points || map_points_w_.size() < config_.plane_min_neighbors) {
    return factor;
  }
  const double max_distance_sq = config_.nearest_distance_m * config_.nearest_distance_m;
  factor.frame_points_i.reserve(sampled.size());
  factor.line_points_w.reserve(sampled.size());
  factor.line_dirs_w.reserve(sampled.size());
  factor.point_weights.reserve(sampled.size());
  factor.weight = 1.0 / std::max(config_.nearest_distance_m * config_.nearest_distance_m, 1.0e-12);
  factor.huber_delta_m = config_.robust_kernel_m;

  for (const auto & point_i : sampled) {
    const Eigen::Vector3d point_w = predicted_pose.q_w_i * point_i + predicted_pose.p_w_i;
    const auto neighbors =
      find_nearest_map_neighbors(point_w, max_distance_sq, config_.plane_min_neighbors);
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
    const auto eigenvalues = solver.eigenvalues();  // ascending: l0 <= l1 <= l2
    // LINE/EDGE: the two smaller eigenvalues are both << the largest (points
    // elongated along ONE direction). Discriminate by lambda1/lambda2.
    const double linearity = eigenvalues[2] > 1.0e-12
      ? eigenvalues[1] / eigenvalues[2]
      : std::numeric_limits<double>::infinity();
    if (linearity > config_.line_max_condition) {
      continue;
    }
    const Eigen::Vector3d dir = solver.eigenvectors().col(2).normalized();
    const Eigen::Vector3d offset = point_w - centroid;
    const Eigen::Vector3d perp = offset - offset.dot(dir) * dir;
    const double perp_distance = perp.norm();
    if (perp_distance > config_.nearest_distance_m) {
      continue;
    }
    const double confidence =
      correspondence_confidence(perp_distance, config_.robust_kernel_m) *
      plane_condition_confidence(linearity, config_.line_max_condition);
    if (confidence <= 0.0) {
      continue;
    }
    factor.frame_points_i.push_back(point_i);
    factor.line_points_w.push_back(centroid);
    factor.line_dirs_w.push_back(dir);
    factor.point_weights.push_back(confidence);
  }
  if (factor.frame_points_i.size() < config_.min_points) {
    factor.frame_points_i.clear();
    factor.line_points_w.clear();
    factor.line_dirs_w.clear();
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
  rebuild_spatial_index();
}

}  // namespace gaussian_lic_tracking
