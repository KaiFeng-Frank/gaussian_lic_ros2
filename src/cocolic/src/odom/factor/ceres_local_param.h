/**
BSD 3-Clause License

This file is part of the Basalt project.
https://gitlab.com/VladyslavUsenko/basalt-headers.git

Copyright (c) 2019, Vladyslav Usenko and Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

@file
@brief Generic local parametrization for Sophus Lie group types to be used with
ceres.
*/

/**
File adapted from Sophus

Copyright 2011-2017 Hauke Strasdat
          2012-2017 Steven Lovegrove

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to
deal in the Software without restriction, including without limitation the
rights  to use, copy, modify, merge, publish, distribute, sublicense, and/or
sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
IN THE SOFTWARE.
*/

#pragma once

// Ported ROS1->ROS2: Ceres 2.2 removed LocalParameterization; use Manifold.
// GlobalSize->AmbientSize, LocalSize->TangentSize, ComputeJacobian->PlusJacobian,
// + new required Minus / MinusJacobian. Plus is unchanged (T*exp(delta)).
#include <ceres/manifold.h>
#include <sophus_lib/se3.hpp>

namespace cocolic {

/// @brief Manifold for ceres that can be used with Sophus Lie group types.
/// Proper variant (true lifting Jacobian). NOTE: production attaches the
/// analytic variant below; auto_diff param is null in TrajectoryEstimator, so
/// this variant's MinusJacobian (identity-consistent fallback) is off the
/// deployed path.
template <class Groupd>
class LieLocalParameterization : public ceres::Manifold {
 public:
  virtual ~LieLocalParameterization() {}

  using Tangentd = typename Groupd::Tangent;

  ///  T * exp(x)
  virtual bool Plus(double const* T_raw, double const* delta_raw,
                    double* T_plus_delta_raw) const {
    Eigen::Map<Groupd const> const T(T_raw);
    Eigen::Map<Tangentd const> const delta(delta_raw);
    Eigen::Map<Groupd> T_plus_delta(T_plus_delta_raw);
    T_plus_delta = T * Groupd::exp(delta);
    return true;
  }

  /// d(T*exp(x))/dx at x=0  (AmbientSize x TangentSize)
  virtual bool PlusJacobian(double const* T_raw, double* jacobian_raw) const {
    Eigen::Map<Groupd const> T(T_raw);
    Eigen::Map<Eigen::Matrix<double, Groupd::num_parameters, Groupd::DoF,
                             Eigen::RowMajor>>
        jacobian(jacobian_raw);
    jacobian = T.Dx_this_mul_exp_x_at_0();
    return true;
  }

  /// Minus: log(x^{-1} * y) — inverse of Plus.
  virtual bool Minus(double const* y_raw, double const* x_raw,
                     double* y_minus_x_raw) const {
    Eigen::Map<Groupd const> const y(y_raw);
    Eigen::Map<Groupd const> const x(x_raw);
    Eigen::Map<Tangentd> y_minus_x(y_minus_x_raw);
    y_minus_x = (x.inverse() * y).log();
    return true;
  }

  /// d Minus(y,x)/dy at y=x  (TangentSize x AmbientSize). Identity-left block
  /// (left-consistent with the analytic trick; off the production path).
  virtual bool MinusJacobian(double const* /*x_raw*/, double* jacobian_raw) const {
    Eigen::Map<Eigen::Matrix<double, Groupd::DoF, Groupd::num_parameters,
                             Eigen::RowMajor>>
        jacobian(jacobian_raw);
    jacobian.setZero();
    for (int i = 0; i < Groupd::DoF; ++i) jacobian(i, i) = 1.0;
    return true;
  }

  virtual int AmbientSize() const { return Groupd::num_parameters; }
  virtual int TangentSize() const { return Groupd::DoF; }
};

/// Analytic variant: the lifting Jacobian is folded into the analytic cost
/// Jacobians (VINS trick), so PlusJacobian/MinusJacobian are the identity
/// embedding. This is the variant used in production (attached to SO3 knots).
template <class Groupd>
class LieAnalyticLocalParameterization : public ceres::Manifold {
 public:
  virtual ~LieAnalyticLocalParameterization() {}

  using Tangentd = typename Groupd::Tangent;

  ///  T * exp(x)
  virtual bool Plus(double const* T_raw, double const* delta_raw,
                    double* T_plus_delta_raw) const {
    Eigen::Map<Groupd const> const T(T_raw);
    Eigen::Map<Tangentd const> const delta(delta_raw);
    Eigen::Map<Groupd> T_plus_delta(T_plus_delta_raw);
    T_plus_delta = T * Groupd::exp(delta);
    return true;
  }

  /// Identity embedding (AmbientSize x TangentSize): factor Jacobians carry the
  /// lift. Matches upstream ComputeJacobian (diag 1 in the top TangentSize rows).
  virtual bool PlusJacobian(double const* /*T_raw*/, double* jacobian_raw) const {
    Eigen::Map<Eigen::Matrix<double, Groupd::num_parameters, Groupd::DoF,
                             Eigen::RowMajor>>
        jacobian(jacobian_raw);
    jacobian.setZero();
    for (int i = 0; i < Groupd::DoF; ++i) jacobian(i, i) = 1.0;
    return true;
  }

  /// Minus: log(x^{-1} * y).
  virtual bool Minus(double const* y_raw, double const* x_raw,
                     double* y_minus_x_raw) const {
    Eigen::Map<Groupd const> const y(y_raw);
    Eigen::Map<Groupd const> const x(x_raw);
    Eigen::Map<Tangentd> y_minus_x(y_minus_x_raw);
    y_minus_x = (x.inverse() * y).log();
    return true;
  }

  /// Identity embedding (TangentSize x AmbientSize), consistent with PlusJacobian.
  virtual bool MinusJacobian(double const* /*x_raw*/, double* jacobian_raw) const {
    Eigen::Map<Eigen::Matrix<double, Groupd::DoF, Groupd::num_parameters,
                             Eigen::RowMajor>>
        jacobian(jacobian_raw);
    jacobian.setZero();
    for (int i = 0; i < Groupd::DoF; ++i) jacobian(i, i) = 1.0;
    return true;
  }

  virtual int AmbientSize() const { return Groupd::num_parameters; }
  virtual int TangentSize() const { return Groupd::DoF; }
};

}  // namespace cocolic
