// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_tracking/sliding_window_optimizer.hpp>

#include <cmath>
#include <iostream>

namespace
{
Eigen::Matrix3d skew(const Eigen::Vector3d & value)
{
  Eigen::Matrix3d matrix;
  matrix << 0.0, -value.z(), value.y(),
    value.z(), 0.0, -value.x(),
    -value.y(), value.x(), 0.0;
  return matrix;
}
}  // namespace

int main()
{
  gaussian_lic_tracking::SlidingWindowOptimizer optimizer;
  gaussian_lic_tracking::SlidingWindowState state;
  state.stamp_ns = 100;
  state.q_w_i = Eigen::Quaterniond::Identity();
  state.p_w_i = Eigen::Vector3d{0.1, -0.2, 0.3};
  optimizer.add_or_update_state(state);

  gaussian_lic_tracking::SlidingWindowPointToPointFactor point_factor;
  point_factor.stamp_ns = state.stamp_ns;
  point_factor.weight = 4.0;
  point_factor.frame_points_i.emplace_back(1.0, 2.0, 3.0);
  point_factor.target_points_w.emplace_back(1.2, 1.7, 3.4);
  point_factor.point_weights.push_back(0.25);
  optimizer.add_point_to_point_factor(point_factor);

  gaussian_lic_tracking::SlidingWindowPointToPlaneFactor plane_factor;
  plane_factor.stamp_ns = state.stamp_ns;
  plane_factor.weight = 4.0;
  plane_factor.frame_points_i.emplace_back(1.0, 2.0, 3.0);
  plane_factor.target_points_w.emplace_back(1.0, 2.0, 0.0);
  plane_factor.target_normals_w.emplace_back(0.0, 0.0, 1.0);
  plane_factor.point_weights.push_back(0.25);
  optimizer.add_point_to_plane_factor(plane_factor);

  gaussian_lic_tracking::SlidingWindowVisualAlignmentFactor visual_factor;
  visual_factor.stamp_ns = state.stamp_ns;
  visual_factor.reference_p_w_i = state.p_w_i;
  visual_factor.measured_shift_px = Eigen::Vector2d{2.0, -3.0};
  visual_factor.meters_per_pixel = 0.01;
  visual_factor.weight = 9.0;
  optimizer.add_visual_alignment_factor(visual_factor);

  const auto normal = optimizer.build_normal_equation(0.0);
  if (!normal.valid || normal.jacobian.rows() != 6 || normal.jacobian.cols() != 15) {
    std::cerr << "geometric Jacobian normal equation has wrong dimensions\n";
    return 1;
  }

  Eigen::MatrixXd expected = Eigen::MatrixXd::Zero(6, 15);
  const Eigen::Vector3d rotated_point{1.0, 2.0, 3.0};
  expected.block<3, 3>(0, 0) = -skew(rotated_point);
  expected.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity();
  expected.block<1, 3>(3, 0) =
    -Eigen::RowVector3d{0.0, 0.0, 1.0} * skew(rotated_point);
  expected.block<1, 3>(3, 6) = Eigen::RowVector3d{0.0, 0.0, 1.0};
  expected(4, 6) = 3.0;
  expected(5, 7) = 3.0;

  const double max_abs_error = (normal.jacobian - expected).cwiseAbs().maxCoeff();
  std::cout << "sliding_window_geometric_jacobian_probe rows=" << normal.jacobian.rows()
            << " cols=" << normal.jacobian.cols()
            << " max_abs_error=" << max_abs_error << "\n";
  if (max_abs_error > 1.0e-12) {
    std::cerr << "analytic geometric Jacobian does not match expected SE3 blocks\n";
    return 1;
  }

  std::cout << "sliding_window_geometric_jacobian_probe OK\n";
  return 0;
}
