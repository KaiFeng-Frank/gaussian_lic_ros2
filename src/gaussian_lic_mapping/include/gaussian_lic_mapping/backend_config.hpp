// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

namespace gaussian_lic_mapping
{

struct GaussianBackendConfig
{
  int width{0};
  int height{0};

  bool depth_completion{false};
  int patch_size{10};
  double max_depth{20.0};

  int sh_degree{3};
  bool white_background{false};
  bool random_background{false};
  bool convert_shs_python{false};
  bool compute_cov3d_python{false};
  double lambda_erank{0.0};
  double scaling_scale{1.0};

  double position_lr{0.00016};
  double feature_lr{0.005};
  double opacity_lr{0.05};
  double scaling_lr{0.005};
  double rotation_lr{0.001};
  double lambda_dssim{0.2};
  bool optimize_depth{true};
  double lambda_depth{0.005};
  bool iteration_decay{false};

  bool apply_exposure{false};
  double exposure_lr{0.001};
  int skybox_points_num{0};
  double skybox_radius{1000.0};

  bool enable_photometric_optimization{false};
  int optimization_steps_per_keyframe{0};
  int optimization_max_samples{4096};

  bool enable_density_control{false};
  double prune_min_opacity{0.005};
  int max_foreground_gaussians{0};
};

}  // namespace gaussian_lic_mapping
