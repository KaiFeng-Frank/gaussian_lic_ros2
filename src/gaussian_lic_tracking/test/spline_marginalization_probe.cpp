// SPDX-License-Identifier: GPL-3.0-or-later
//
// Verifies the spline marginalization Schur-complement pipeline.
//
// Synthetic scenario:
//   * Two parameter blocks: a "marginalized" block (size 3) and a "kept"
//     block (size 3).
//   * One residual block touching both with random Jacobian entries.
// After marginalization, the reduced prior cost in the kept block must match
// the cost obtained by exactly substituting the optimal Δx_m back into the
// original normal equation. This is the textbook Schur consistency check.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <gaussian_lic_tracking/spline/spline_marginalization.hpp>

using gaussian_lic_tracking::spline::LinearizedResidualBlock;
using gaussian_lic_tracking::spline::MarginalizationResult;
using gaussian_lic_tracking::spline::SplineMarginalizationInfo;

namespace
{

LinearizedResidualBlock make_block(
  std::int64_t marg_id,
  std::int64_t keep_id,
  const Eigen::MatrixXd & jac_marg,
  const Eigen::MatrixXd & jac_keep,
  const Eigen::VectorXd & residual)
{
  LinearizedResidualBlock block;
  block.parameter_block_ids = {marg_id, keep_id};
  block.parameter_block_sizes = {static_cast<int>(jac_marg.cols()),
    static_cast<int>(jac_keep.cols())};
  block.jacobians = {jac_marg, jac_keep};
  block.residual = residual;
  return block;
}

void check_schur_consistency()
{
  // Random-ish but reproducible inputs.
  Eigen::MatrixXd jac_marg(4, 3);
  jac_marg <<
    0.5, 0.0, 0.1,
    0.2, 0.3, 0.0,
    -0.1, 0.4, 0.2,
    0.0, 0.0, 0.5;
  Eigen::MatrixXd jac_keep(4, 3);
  jac_keep <<
    0.1, 0.5, 0.0,
    0.0, 0.2, 0.3,
    0.3, -0.1, 0.4,
    -0.2, 0.0, 0.1;
  Eigen::VectorXd r(4);
  r << 0.1, -0.2, 0.05, 0.15;

  SplineMarginalizationInfo info;
  info.mark_block_to_marginalize(/*marg_id=*/1);
  info.add_residual_block(make_block(1, 2, jac_marg, jac_keep, r));

  MarginalizationResult result;
  if (!info.marginalize(result)) {
    std::fprintf(stderr, "marginalize() returned false\n");
    std::exit(1);
  }

  // Manual Schur computation.
  Eigen::MatrixXd H(6, 6);
  H.setZero();
  Eigen::MatrixXd full_jacobian(4, 6);
  full_jacobian << jac_marg, jac_keep;
  H = full_jacobian.transpose() * full_jacobian;
  Eigen::VectorXd b = -full_jacobian.transpose() * r;

  Eigen::MatrixXd H_mm = H.topLeftCorner(3, 3);
  Eigen::MatrixXd H_mk = H.topRightCorner(3, 3);
  Eigen::MatrixXd H_kk = H.bottomRightCorner(3, 3);
  Eigen::VectorXd b_m = b.head(3);
  Eigen::VectorXd b_k = b.tail(3);

  Eigen::MatrixXd H_mm_inv = H_mm.inverse();
  Eigen::MatrixXd reduced_H = H_kk - H_mk.transpose() * H_mm_inv * H_mk;
  Eigen::VectorXd reduced_b = b_k - H_mk.transpose() * H_mm_inv * b_m;

  // Test: the prior recovered from result reproduces J^T J = reduced_H and
  // J^T r = -reduced_b (within numeric tolerance).
  const Eigen::MatrixXd jtj = result.jacobian.transpose() * result.jacobian;
  if ((jtj - reduced_H).cwiseAbs().maxCoeff() > 1.0e-8) {
    std::fprintf(stderr,
      "Schur H mismatch: max |J^T J - reduced_H| = %.6g\n",
      (jtj - reduced_H).cwiseAbs().maxCoeff());
    std::exit(1);
  }
  const Eigen::VectorXd jtr = result.jacobian.transpose() * result.residual;
  if ((jtr + reduced_b).cwiseAbs().maxCoeff() > 1.0e-8) {
    std::fprintf(stderr,
      "Schur b mismatch: max |J^T r + reduced_b| = %.6g\n",
      (jtr + reduced_b).cwiseAbs().maxCoeff());
    std::exit(1);
  }

  // Layout sanity: one kept block of size 3.
  if (result.kept_block_ids.size() != 1 || result.kept_block_ids[0] != 2) {
    std::fprintf(stderr, "kept block id mismatch\n");
    std::exit(1);
  }
  if (result.kept_block_sizes.size() != 1 || result.kept_block_sizes[0] != 3) {
    std::fprintf(stderr, "kept block size mismatch\n");
    std::exit(1);
  }
  if (result.margin_rows != 3 || result.keep_rows != 3) {
    std::fprintf(stderr,
      "margin/keep row count mismatch: margin=%d keep=%d\n",
      result.margin_rows, result.keep_rows);
    std::exit(1);
  }
}

void check_two_residual_blocks_compose()
{
  // Marginalize a position knot (id=0, size=3) constrained by two
  // independent residual blocks; the Schur reduction should commute with
  // accumulation.
  SplineMarginalizationInfo info;
  info.mark_block_to_marginalize(0);

  Eigen::MatrixXd jacA_m(3, 3);
  jacA_m << 1, 0, 0, 0, 1, 0, 0, 0, 1;
  Eigen::MatrixXd jacA_k(3, 3);
  jacA_k << 0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1;
  Eigen::VectorXd rA(3);
  rA << 0.01, 0.02, 0.03;
  info.add_residual_block(make_block(0, 1, jacA_m, jacA_k, rA));

  Eigen::MatrixXd jacB_m(3, 3);
  jacB_m << 0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5;
  Eigen::MatrixXd jacB_k(3, 3);
  jacB_k << 0.0, 0.2, 0.0, 0.0, 0.0, 0.2, 0.2, 0.0, 0.0;
  Eigen::VectorXd rB(3);
  rB << -0.01, -0.02, 0.0;
  info.add_residual_block(make_block(0, 1, jacB_m, jacB_k, rB));

  MarginalizationResult result;
  if (!info.marginalize(result)) {
    std::fprintf(stderr, "marginalize with two residual blocks failed\n");
    std::exit(1);
  }

  // Combined manual Schur.
  Eigen::MatrixXd full_jac(6, 6);
  full_jac.setZero();
  full_jac.block(0, 0, 3, 3) = jacA_m;
  full_jac.block(0, 3, 3, 3) = jacA_k;
  full_jac.block(3, 0, 3, 3) = jacB_m;
  full_jac.block(3, 3, 3, 3) = jacB_k;
  Eigen::VectorXd full_r(6);
  full_r << rA, rB;
  Eigen::MatrixXd H = full_jac.transpose() * full_jac;
  Eigen::VectorXd b = -full_jac.transpose() * full_r;

  Eigen::MatrixXd H_mm_inv = H.topLeftCorner(3, 3).inverse();
  Eigen::MatrixXd reduced_H = H.bottomRightCorner(3, 3) -
    H.topRightCorner(3, 3).transpose() * H_mm_inv * H.topRightCorner(3, 3);
  Eigen::VectorXd reduced_b = b.tail(3) -
    H.topRightCorner(3, 3).transpose() * H_mm_inv * b.head(3);

  const Eigen::MatrixXd jtj = result.jacobian.transpose() * result.jacobian;
  if ((jtj - reduced_H).cwiseAbs().maxCoeff() > 1.0e-8) {
    std::fprintf(stderr,
      "two-block Schur H mismatch: max |J^T J - reduced_H|=%.6g\n",
      (jtj - reduced_H).cwiseAbs().maxCoeff());
    std::exit(1);
  }
  const Eigen::VectorXd jtr = result.jacobian.transpose() * result.residual;
  if ((jtr + reduced_b).cwiseAbs().maxCoeff() > 1.0e-8) {
    std::fprintf(stderr,
      "two-block Schur b mismatch: max |J^T r + reduced_b|=%.6g\n",
      (jtr + reduced_b).cwiseAbs().maxCoeff());
    std::exit(1);
  }
}

}  // namespace

int main()
{
  try {
    check_schur_consistency();
    check_two_residual_blocks_compose();
  } catch (const std::exception & exception) {
    std::fprintf(stderr, "spline_marginalization_probe exception: %s\n", exception.what());
    return 1;
  }
  std::printf("spline_marginalization_probe ok\n");
  return 0;
}
