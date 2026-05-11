// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_tracking/spline/spline_marginalization.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace gaussian_lic_tracking
{
namespace spline
{

namespace
{

bool block_marked(
  const std::vector<std::int64_t> & marginalize_ids,
  std::int64_t id)
{
  return std::find(marginalize_ids.begin(), marginalize_ids.end(), id) !=
         marginalize_ids.end();
}

}  // namespace

void SplineMarginalizationInfo::mark_block_to_marginalize(std::int64_t id)
{
  if (!block_marked(marginalize_ids_, id)) {
    marginalize_ids_.push_back(id);
  }
}

void SplineMarginalizationInfo::add_residual_block(
  const LinearizedResidualBlock & block)
{
  if (block.parameter_block_ids.size() != block.parameter_block_sizes.size() ||
    block.parameter_block_ids.size() != block.jacobians.size())
  {
    throw std::runtime_error(
            "LinearizedResidualBlock parameter id/size/jacobian array lengths must match");
  }
  for (std::size_t i = 0; i < block.parameter_block_ids.size(); ++i) {
    if (block.jacobians[i].cols() != block.parameter_block_sizes[i]) {
      throw std::runtime_error("LinearizedResidualBlock jacobian columns do not match block size");
    }
    if (block.jacobians[i].rows() != block.residual.size()) {
      throw std::runtime_error("LinearizedResidualBlock jacobian rows do not match residual size");
    }
  }
  blocks_.push_back(block);
}

bool SplineMarginalizationInfo::marginalize(MarginalizationResult & out) const
{
  if (blocks_.empty()) {
    return false;
  }
  if (marginalize_ids_.empty()) {
    return false;
  }

  // Step 1: enumerate all parameter blocks touched and assign a column offset.
  std::vector<std::int64_t> marg_ids = marginalize_ids_;
  std::vector<int> marg_sizes;
  std::vector<std::int64_t> keep_ids;
  std::vector<int> keep_sizes;

  std::unordered_map<std::int64_t, int> block_size_map;
  for (const auto & block : blocks_) {
    for (std::size_t i = 0; i < block.parameter_block_ids.size(); ++i) {
      const auto id = block.parameter_block_ids[i];
      const int size = block.parameter_block_sizes[i];
      auto it = block_size_map.find(id);
      if (it == block_size_map.end()) {
        block_size_map[id] = size;
      } else if (it->second != size) {
        return false;
      }
    }
  }

  marg_sizes.reserve(marg_ids.size());
  for (auto id : marg_ids) {
    auto it = block_size_map.find(id);
    if (it == block_size_map.end()) {
      return false;
    }
    marg_sizes.push_back(it->second);
  }

  for (const auto & [id, size] : block_size_map) {
    if (!block_marked(marg_ids, id)) {
      keep_ids.push_back(id);
      keep_sizes.push_back(size);
    }
  }

  // Step 2: assign column offsets, marginalized blocks first.
  std::unordered_map<std::int64_t, int> column_offset;
  int offset = 0;
  for (std::size_t i = 0; i < marg_ids.size(); ++i) {
    column_offset[marg_ids[i]] = offset;
    offset += marg_sizes[i];
  }
  const int m = offset;
  for (std::size_t i = 0; i < keep_ids.size(); ++i) {
    column_offset[keep_ids[i]] = offset;
    offset += keep_sizes[i];
  }
  const int n = offset;

  if (m == 0 || n == m) {
    return false;
  }

  // Step 3: accumulate H = J^T J and b = -J^T r.
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(n, n);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(n);
  for (const auto & block : blocks_) {
    // Build the full jacobian row for this block by stitching its sparse
    // contributions over the global parameter layout.
    const int residual_dim = block.residual.size();
    Eigen::MatrixXd block_jacobian = Eigen::MatrixXd::Zero(residual_dim, n);
    for (std::size_t i = 0; i < block.parameter_block_ids.size(); ++i) {
      const int col = column_offset[block.parameter_block_ids[i]];
      block_jacobian.block(0, col, residual_dim, block.parameter_block_sizes[i]) =
        block.jacobians[i];
    }
    H.noalias() += block_jacobian.transpose() * block_jacobian;
    b.noalias() -= block_jacobian.transpose() * block.residual;
  }

  // Step 4: Schur-complement out the marginalized block.
  Eigen::MatrixXd H_mm = H.topLeftCorner(m, m);
  Eigen::MatrixXd H_mk = H.topRightCorner(m, n - m);
  Eigen::MatrixXd H_kk = H.bottomRightCorner(n - m, n - m);
  Eigen::VectorXd b_m = b.head(m);
  Eigen::VectorXd b_k = b.tail(n - m);

  // Symmetrize H_mm before inversion via SVD for numerical robustness.
  Eigen::MatrixXd H_mm_sym = 0.5 * (H_mm + H_mm.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig_mm(H_mm_sym);
  if (eig_mm.info() != Eigen::Success) {
    return false;
  }
  const double eps = 1.0e-12;
  const Eigen::VectorXd & evals = eig_mm.eigenvalues();
  Eigen::VectorXd inv_evals = Eigen::VectorXd::Zero(evals.size());
  for (int i = 0; i < evals.size(); ++i) {
    if (std::abs(evals[i]) > eps) {
      inv_evals[i] = 1.0 / evals[i];
    }
  }
  const Eigen::MatrixXd & V = eig_mm.eigenvectors();
  const Eigen::MatrixXd H_mm_inv = V * inv_evals.asDiagonal() * V.transpose();

  const Eigen::MatrixXd H_km = H_mk.transpose();
  Eigen::MatrixXd reduced_H = H_kk - H_km * H_mm_inv * H_mk;
  Eigen::VectorXd reduced_b = b_k - H_km * H_mm_inv * b_m;

  // Symmetrize before decomposition.
  reduced_H = 0.5 * (reduced_H + reduced_H.transpose());

  // Step 5: J^T J = reduced_H, want J such that the prior cost is
  // 0.5 * |J Δx + r|^2 with gradient -J^T r = -reduced_b and Hessian
  // J^T J = reduced_H. Use eigendecomposition for numerical stability:
  //   reduced_H = V S V^T, J = sqrt(S) V^T, r = -J^{-T} b_k_reduced.
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig_kk(reduced_H);
  if (eig_kk.info() != Eigen::Success) {
    return false;
  }
  const Eigen::VectorXd & ev = eig_kk.eigenvalues();
  Eigen::VectorXd sqrt_ev = Eigen::VectorXd::Zero(ev.size());
  Eigen::VectorXd inv_sqrt_ev = Eigen::VectorXd::Zero(ev.size());
  for (int i = 0; i < ev.size(); ++i) {
    const double v = std::max(0.0, ev[i]);
    sqrt_ev[i] = std::sqrt(v);
    if (sqrt_ev[i] > eps) {
      inv_sqrt_ev[i] = 1.0 / sqrt_ev[i];
    }
  }
  const Eigen::MatrixXd & Vk = eig_kk.eigenvectors();
  out.jacobian = sqrt_ev.asDiagonal() * Vk.transpose();
  // Evaluate the diagonal*matrix product into a concrete vector before
  // negation so unary minus has a typed operand.
  const Eigen::VectorXd residual_signed =
    (inv_sqrt_ev.asDiagonal() * Vk.transpose() * reduced_b).eval();
  out.residual = -residual_signed;
  out.kept_block_ids = keep_ids;
  out.kept_block_sizes = keep_sizes;
  out.margin_rows = m;
  out.keep_rows = n - m;
  return true;
}

}  // namespace spline
}  // namespace gaussian_lic_tracking
