// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_tracking/sliding_window_optimizer.hpp>

#include <iostream>

int main()
{
  Eigen::MatrixXd h_mm(2, 2);
  h_mm << 5.0, 1.0,
    1.0, 4.0;
  Eigen::MatrixXd h_mr(2, 2);
  h_mr << 1.0, 0.5,
    -0.25, 0.75;
  Eigen::MatrixXd h_rr(2, 2);
  h_rr << 3.0, 0.2,
    0.2, 2.5;
  Eigen::VectorXd rhs_m(2);
  rhs_m << 1.0, -2.0;
  Eigen::VectorXd rhs_r(2);
  rhs_r << 0.5, 1.5;

  Eigen::MatrixXd full_hessian(4, 4);
  full_hessian << h_mm, h_mr,
    h_mr.transpose(), h_rr;
  Eigen::VectorXd full_rhs(4);
  full_rhs << rhs_m, rhs_r;
  const Eigen::VectorXd full_solution = full_hessian.ldlt().solve(full_rhs);

  const auto schur = gaussian_lic_tracking::compute_schur_complement(
    h_mm, h_mr, h_rr, rhs_m, rhs_r, 0.0);
  if (!schur.valid) {
    std::cerr << "Schur complement result is invalid\n";
    return 1;
  }
  const Eigen::VectorXd reduced_solution = schur.hessian.ldlt().solve(schur.rhs);
  const double solution_error = (reduced_solution - full_solution.tail<2>()).norm();
  const double symmetry_error = (schur.hessian - schur.hessian.transpose()).norm();
  std::cout << "schur_complement_probe solution_error=" << solution_error
            << " symmetry_error=" << symmetry_error
            << " reduced_dim=" << schur.hessian.rows() << "\n";
  if (solution_error > 1.0e-12 || symmetry_error > 1.0e-12) {
    std::cerr << "Schur complement does not match the full normal-equation solve\n";
    return 1;
  }
  std::cout << "schur_complement_probe OK\n";
  return 0;
}
