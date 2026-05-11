// SPDX-License-Identifier: GPL-3.0-or-later
//
// ROS2-native port of Coco-LIC's MarginalizationInfo / MarginalizationFactor
// (external/Coco-LIC/src/odom/factor/analytic_diff/marginalization_factor.{h,cpp}).
//
// `SplineMarginalizationInfo` accumulates linearized residual blocks that
// touch a subset of spline control knots flagged for marginalization. After
// `marginalize()` runs the Schur complement reduction, the resulting square
// root J and residual r form a linearized prior on the *kept* knots, exposed
// through `linearized_jacobian()` / `linearized_residual()`.
//
// This file is independent of Ceres so the math can be unit-tested alongside
// the rest of the spline foundation.

#pragma once

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

namespace gaussian_lic_tracking
{
namespace spline
{

// One linearized residual block: Jacobian rows stacked horizontally across
// parameter blocks (each block is `parameter_block_sizes[i]` columns wide).
struct LinearizedResidualBlock
{
  std::vector<std::int64_t> parameter_block_ids;     // user-chosen identifiers
  std::vector<int> parameter_block_sizes;             // size of each block
  std::vector<Eigen::MatrixXd> jacobians;             // one block per parameter
  Eigen::VectorXd residual;
};

struct MarginalizationResult
{
  // Square-root information J such that the linearized prior cost reads
  //   c = 0.5 * |J * Δx_keep + r|^2
  Eigen::MatrixXd jacobian;
  Eigen::VectorXd residual;
  // Order in which the kept parameter blocks were stacked into the columns
  // of `jacobian` (and rows of `Δx_keep`).
  std::vector<std::int64_t> kept_block_ids;
  std::vector<int> kept_block_sizes;
  int margin_rows{0};
  int keep_rows{0};
};

class SplineMarginalizationInfo
{
public:
  void mark_block_to_marginalize(std::int64_t id);
  void add_residual_block(const LinearizedResidualBlock & block);

  // Build the normal equation, run Schur complement against the marginalized
  // blocks, and decompose the reduced prior. Returns true on success.
  bool marginalize(MarginalizationResult & out) const;

  std::size_t residual_block_count() const { return blocks_.size(); }
  std::size_t marginalized_block_count() const { return marginalize_ids_.size(); }

private:
  std::vector<LinearizedResidualBlock> blocks_;
  // Use vector + linear search; we expect O(10) marginalized blocks.
  std::vector<std::int64_t> marginalize_ids_;
};

}  // namespace spline
}  // namespace gaussian_lic_tracking
