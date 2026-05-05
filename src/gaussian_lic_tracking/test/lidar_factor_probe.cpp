// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_tracking/lidar_factor.hpp>

#include <cmath>
#include <exception>
#include <iostream>
#include <limits>
#include <vector>

int main()
{
  gaussian_lic_tracking::LidarFactorConfig config;
  config.min_points = 16;
  config.max_frame_points = 256;
  config.max_map_points = 1024;
  config.nearest_distance_m = 0.5;
  config.correction_gain = 1.0;
  config.max_correction_m = 1.0;
  config.max_rotation_rad = 0.2;
  config.robust_kernel_m = 0.02;
  bool rejected_bad_config = false;
  try {
    auto bad_config = config;
    bad_config.nearest_distance_m = std::numeric_limits<double>::quiet_NaN();
    gaussian_lic_tracking::LidarFactor bad_factor(bad_config);
  } catch (const std::exception &) {
    rejected_bad_config = true;
  }
  if (!rejected_bad_config) {
    std::cerr << "LiDAR factor failed to reject non-finite config\n";
    return 1;
  }
  gaussian_lic_tracking::LidarFactor factor(config);

  std::vector<Eigen::Vector3d> scan;
  for (int y = 0; y < 8; ++y) {
    for (int x = 0; x < 8; ++x) {
      scan.emplace_back(0.2 * static_cast<double>(x), 0.2 * static_cast<double>(y), 1.0);
    }
  }

  gaussian_lic_tracking::TrajectoryPose identity;
  identity.stamp_ns = 0;
  identity.q_w_i = Eigen::Quaterniond::Identity();
  factor.insert_keyframe(scan, identity);
  if (factor.map_size() != scan.size()) {
    std::cerr << "LiDAR map insert size mismatch\n";
    return 1;
  }

  std::vector<Eigen::Vector3d> scan_with_nan = scan;
  scan_with_nan.front().x() = std::numeric_limits<double>::quiet_NaN();
  factor.clear();
  factor.insert_keyframe(scan_with_nan, identity);
  if (factor.map_size() != scan.size() - 1U) {
    std::cerr << "LiDAR factor failed to filter non-finite keyframe points\n";
    return 1;
  }
  gaussian_lic_tracking::TrajectoryPose bad_pose = identity;
  bad_pose.q_w_i.coeffs().setZero();
  factor.insert_keyframe(scan, bad_pose);
  if (factor.map_size() != scan.size() - 1U) {
    std::cerr << "LiDAR factor accepted an invalid-pose keyframe\n";
    return 1;
  }
  factor.clear();
  factor.insert_keyframe(scan, identity);

  gaussian_lic_tracking::TrajectoryPose predicted;
  predicted.stamp_ns = 100000000LL;
  predicted.q_w_i = Eigen::Quaterniond::Identity();
  predicted.p_w_i = Eigen::Vector3d(0.05, 0.0, 0.0);
  if (factor.compute_translation_correction(scan, bad_pose).applied) {
    std::cerr << "LiDAR factor corrected with an invalid pose\n";
    return 1;
  }
  const auto correction = factor.compute_translation_correction(scan, predicted);
  const double x_error = std::abs(correction.delta_p_w.x() + 0.05);
  const double yz_error = std::abs(correction.delta_p_w.y()) + std::abs(correction.delta_p_w.z());
  std::cout << "lidar_factor_probe matched=" << correction.matched_points
            << " mean_residual_m=" << correction.mean_residual_m
            << " delta=" << correction.delta_p_w.transpose()
            << " map_size=" << factor.map_size() << "\n";
  if (!correction.applied || correction.matched_points < config.min_points) {
    std::cerr << "LiDAR factor did not produce a correction\n";
    return 1;
  }
  if (x_error > 1.0e-12 || yz_error > 1.0e-12) {
    std::cerr << "LiDAR translation correction is wrong\n";
    return 1;
  }

  std::vector<Eigen::Vector3d> structured_scan;
  for (int z = 0; z < 3; ++z) {
    for (int y = 0; y < 4; ++y) {
      for (int x = 0; x < 5; ++x) {
        structured_scan.emplace_back(
          0.2 * static_cast<double>(x),
          0.15 * static_cast<double>(y),
          0.1 * static_cast<double>(z));
      }
    }
  }
  factor.clear();
  factor.insert_keyframe(structured_scan, identity);

  gaussian_lic_tracking::TrajectoryPose rotated_prediction;
  rotated_prediction.stamp_ns = 200000000LL;
  rotated_prediction.p_w_i = Eigen::Vector3d(0.04, -0.03, 0.02);
  rotated_prediction.q_w_i =
    Eigen::Quaterniond(Eigen::AngleAxisd(0.04, Eigen::Vector3d::UnitZ()));
  const auto pose_correction = factor.compute_pose_correction(structured_scan, rotated_prediction);
  const Eigen::Quaterniond corrected_q =
    (pose_correction.delta_q * rotated_prediction.q_w_i).normalized();
  const Eigen::Vector3d corrected_p = rotated_prediction.p_w_i + pose_correction.delta_p_w;
  const double pose_translation_error = corrected_p.norm();
  const double pose_rotation_error =
    Eigen::AngleAxisd(corrected_q).angle();
  std::cout << "lidar_pose_correction matched=" << pose_correction.matched_points
            << " mean_residual_m=" << pose_correction.mean_residual_m
            << " delta_p=" << pose_correction.delta_p_w.transpose()
            << " corrected_translation_error=" << pose_translation_error
            << " corrected_rotation_error=" << pose_rotation_error << "\n";
  if (!pose_correction.applied || pose_correction.matched_points < config.min_points) {
    std::cerr << "LiDAR pose correction did not apply\n";
    return 1;
  }
  if (pose_translation_error > 1.0e-6 || pose_rotation_error > 1.0e-6) {
    std::cerr << "LiDAR 6-DoF pose correction is wrong\n";
    return 1;
  }

  std::vector<Eigen::Vector3d> contaminated_scan = structured_scan;
  for (size_t index = 0; index < 24U && index < contaminated_scan.size(); ++index) {
    contaminated_scan[index] = Eigen::Vector3d(
      1.05 + 0.02 * static_cast<double>(index % 6U),
      0.65 + 0.01 * static_cast<double>(index / 6U),
      0.28);
  }
  auto robust_config = config;
  robust_config.robust_kernel_m = 0.01;
  auto unweighted_config = config;
  unweighted_config.robust_kernel_m = 100.0;
  gaussian_lic_tracking::LidarFactor robust_factor(robust_config);
  gaussian_lic_tracking::LidarFactor unweighted_factor(unweighted_config);
  robust_factor.insert_keyframe(structured_scan, identity);
  unweighted_factor.insert_keyframe(structured_scan, identity);
  const auto robust_pose_correction =
    robust_factor.compute_pose_correction(contaminated_scan, rotated_prediction);
  const auto unweighted_pose_correction =
    unweighted_factor.compute_pose_correction(contaminated_scan, rotated_prediction);
  const Eigen::Quaterniond robust_corrected_q =
    (robust_pose_correction.delta_q * rotated_prediction.q_w_i).normalized();
  const Eigen::Quaterniond unweighted_corrected_q =
    (unweighted_pose_correction.delta_q * rotated_prediction.q_w_i).normalized();
  const double robust_error =
    (rotated_prediction.p_w_i + robust_pose_correction.delta_p_w).norm() +
    Eigen::AngleAxisd(robust_corrected_q).angle();
  const double unweighted_error =
    (rotated_prediction.p_w_i + unweighted_pose_correction.delta_p_w).norm() +
    Eigen::AngleAxisd(unweighted_corrected_q).angle();
  std::cout << "lidar_pose_robust_kabsch robust_error=" << robust_error
            << " unweighted_error=" << unweighted_error
            << " robust_mean_residual_m=" << robust_pose_correction.mean_residual_m
            << " unweighted_mean_residual_m=" << unweighted_pose_correction.mean_residual_m
            << "\n";
  if (!robust_pose_correction.applied || !unweighted_pose_correction.applied) {
    std::cerr << "LiDAR robust pose correction did not apply\n";
    return 1;
  }
  if (robust_error + 1.0e-6 >= unweighted_error) {
    std::cerr << "LiDAR robust Kabsch weighting did not improve contaminated pose correction\n";
    return 1;
  }

  const auto point_factor = factor.build_point_to_point_factor(structured_scan, rotated_prediction);
  std::cout << "lidar_point_factor correspondences=" << point_factor.frame_points_i.size()
            << " weight=" << point_factor.weight
            << " robust_weights=" << point_factor.point_weights.size() << "\n";
  if (point_factor.stamp_ns != rotated_prediction.stamp_ns ||
    point_factor.frame_points_i.size() < config.min_points ||
    point_factor.frame_points_i.size() != point_factor.target_points_w.size() ||
    point_factor.point_weights.size() != point_factor.frame_points_i.size())
  {
    std::cerr << "LiDAR point-to-point window factor is invalid\n";
    return 1;
  }
  bool saw_downweighted_match = false;
  for (const double weight : point_factor.point_weights) {
    if (weight <= 0.0 || weight > 1.0) {
      std::cerr << "LiDAR robust point weight is outside (0, 1]\n";
      return 1;
    }
    saw_downweighted_match = saw_downweighted_match || weight < 1.0;
  }
  if (!saw_downweighted_match) {
    std::cerr << "LiDAR robust kernel did not downweight any nonzero residual\n";
    return 1;
  }

  std::vector<Eigen::Vector3d> plane_scan;
  for (int y = 0; y < 6; ++y) {
    for (int x = 0; x < 6; ++x) {
      plane_scan.emplace_back(0.1 * static_cast<double>(x), 0.1 * static_cast<double>(y), 0.0);
    }
  }
  factor.clear();
  factor.insert_keyframe(plane_scan, identity);
  gaussian_lic_tracking::TrajectoryPose plane_prediction;
  plane_prediction.stamp_ns = 300000000LL;
  plane_prediction.q_w_i = Eigen::Quaterniond::Identity();
  plane_prediction.p_w_i = Eigen::Vector3d{0.0, 0.0, 0.05};
  const auto plane_factor = factor.build_point_to_plane_factor(plane_scan, plane_prediction);
  std::cout << " lidar_plane_factor correspondences=" << plane_factor.frame_points_i.size()
            << " normals=" << plane_factor.target_normals_w.size() << "\n";
  if (plane_factor.frame_points_i.size() < config.min_points ||
    plane_factor.frame_points_i.size() != plane_factor.target_points_w.size() ||
    plane_factor.frame_points_i.size() != plane_factor.target_normals_w.size() ||
    plane_factor.point_weights.size() != plane_factor.frame_points_i.size())
  {
    std::cerr << "LiDAR point-to-plane window factor is invalid\n";
    return 1;
  }
  for (const auto & normal : plane_factor.target_normals_w) {
    if (std::abs(std::abs(normal.z()) - 1.0) > 1.0e-6) {
      std::cerr << "LiDAR plane normal is not aligned with the fitted plane\n";
      return 1;
    }
  }

  factor.clear();
  const auto empty_correction = factor.compute_translation_correction(scan, predicted);
  if (empty_correction.applied) {
    std::cerr << "LiDAR factor corrected without a map\n";
    return 1;
  }

  std::cout << "lidar_factor_probe OK\n";
  return 0;
}
