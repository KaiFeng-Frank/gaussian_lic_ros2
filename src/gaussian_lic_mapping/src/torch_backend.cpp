// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_mapping/torch_backend.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

namespace gaussian_lic_mapping
{
namespace
{

struct GaussianTensorPack
{
  torch::Tensor xyz;
  torch::Tensor features_dc;
  torch::Tensor features_rest;
  torch::Tensor scaling;
  torch::Tensor rotation;
  torch::Tensor opacity;
  size_t foreground_count{0};
  size_t skybox_count{0};
};

torch::Tensor eigen_matrix_to_torch_tensor(
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> matrix,
  torch::Device device)
{
  matrix.transposeInPlace();
  return torch::from_blob(
    matrix.data(),
    {matrix.rows(), matrix.cols()},
    torch::TensorOptions().dtype(torch::kFloat32)).clone().to(device);
}

torch::Tensor make_world_view_transform(
  const Eigen::Matrix3d & r_wc,
  const Eigen::Vector3d & t_wc,
  torch::Device device)
{
  Eigen::Matrix4f rt = Eigen::Matrix4f::Zero();
  const Eigen::Matrix3f r_cw = r_wc.transpose().cast<float>();
  const Eigen::Vector3f t_cw = (-r_wc.transpose() * t_wc).cast<float>();
  rt.topLeftCorner<3, 3>() = r_cw;
  rt.topRightCorner<3, 1>() = t_cw;
  rt(3, 3) = 1.0F;

  return eigen_matrix_to_torch_tensor(rt, device).transpose(0, 1);
}

torch::Tensor make_projection_matrix(
  const int width,
  const int height,
  const float fx,
  const float fy,
  const float cx,
  const float cy,
  torch::Device device)
{
  constexpr float z_near = 0.01F;
  constexpr float z_far = 100.0F;
  const float fov_x = 2.0F * std::atan(static_cast<float>(width) / (2.0F * fx));
  const float fov_y = 2.0F * std::atan(static_cast<float>(height) / (2.0F * fy));

  torch::Tensor projection = torch::zeros({4, 4}, torch::TensorOptions().dtype(torch::kFloat32).device(device));
  projection.index_put_({0, 0}, 1.0F / std::tan(fov_x / 2.0F));
  projection.index_put_({1, 1}, 1.0F / std::tan(fov_y / 2.0F));
  projection.index_put_({0, 2}, (2.0F * cx - static_cast<float>(width)) / static_cast<float>(width));
  projection.index_put_({1, 2}, (2.0F * cy - static_cast<float>(height)) / static_cast<float>(height));
  projection.index_put_({3, 2}, 1.0F);
  projection.index_put_({2, 2}, z_far / (z_far - z_near));
  projection.index_put_({2, 3}, -(z_far * z_near) / (z_far - z_near));
  return projection.transpose(0, 1);
}

float rgb_to_sh(const float value)
{
  constexpr float c0 = 0.28209479177387814F;
  return (value - 0.5F) / c0;
}

torch::Tensor inverse_sigmoid(const torch::Tensor & x)
{
  return torch::log(x / (1.0F - x));
}

GaussianTensorPack make_foreground_tensors_from_pending_points(
  const MapperDataset & dataset,
  const int sh_degree,
  const double scaling_scale,
  const double fx,
  const double fy,
  torch::Device device)
{
  const size_t num_points = dataset.pending_point_count();
  if (num_points == 0) {
    throw std::runtime_error("cannot build foreground Gaussian tensors without pending points");
  }
  if (sh_degree < 0) {
    throw std::runtime_error("sh_degree must be non-negative");
  }
  if (scaling_scale <= 0.0) {
    throw std::runtime_error("scaling_scale must be positive");
  }

  const double focal = (fx + fy) * 0.5;
  if (focal <= std::numeric_limits<double>::epsilon()) {
    throw std::runtime_error("average focal length must be positive");
  }

  const int64_t num = static_cast<int64_t>(num_points);
  const int64_t sh_coeff_count = static_cast<int64_t>(sh_degree + 1) *
    static_cast<int64_t>(sh_degree + 1);

  const auto & points = dataset.pending_points_world();
  const auto & colors = dataset.pending_colors_rgb();
  const auto & depths = dataset.pending_depths_m();
  if (colors.size() != num_points || depths.size() != num_points) {
    throw std::runtime_error("pending point/color/depth buffers have inconsistent sizes");
  }

  std::vector<float> xyz_values(num_points * 3U);
  std::vector<float> feature_values(num_points * 3U * static_cast<size_t>(sh_coeff_count), 0.0F);
  std::vector<float> scale_values(num_points);

  for (size_t i = 0; i < num_points; ++i) {
    xyz_values[i * 3U + 0U] = points[i].x();
    xyz_values[i * 3U + 1U] = points[i].y();
    xyz_values[i * 3U + 2U] = points[i].z();

    const size_t feature_offset = i * 3U * static_cast<size_t>(sh_coeff_count);
    feature_values[feature_offset + 0U * static_cast<size_t>(sh_coeff_count)] =
      rgb_to_sh(colors[i].x());
    feature_values[feature_offset + 1U * static_cast<size_t>(sh_coeff_count)] =
      rgb_to_sh(colors[i].y());
    feature_values[feature_offset + 2U * static_cast<size_t>(sh_coeff_count)] =
      rgb_to_sh(colors[i].z());

    const double clamped_depth = std::max(static_cast<double>(depths[i]), 1e-6);
    scale_values[i] = static_cast<float>(std::log(scaling_scale * clamped_depth / focal));
  }

  const auto cpu_options = torch::TensorOptions().dtype(torch::kFloat32);
  auto xyz = torch::from_blob(xyz_values.data(), {num, 3}, cpu_options).clone().to(device);
  auto features = torch::from_blob(
    feature_values.data(), {num, 3, sh_coeff_count}, cpu_options).clone().to(device);
  auto scaling = torch::from_blob(scale_values.data(), {num}, cpu_options)
    .clone()
    .to(device)
    .unsqueeze(1)
    .repeat({1, 3});

  auto rotation = torch::zeros({num, 4}, cpu_options.device(device));
  rotation.index_put_({torch::indexing::Slice(), 0}, 1.0F);
  auto opacity = inverse_sigmoid(
    0.1F * torch::ones({num, 1}, cpu_options.device(device)));

  GaussianTensorPack pack;
  pack.xyz = xyz;
  pack.features_dc = features.index({
      torch::indexing::Slice(),
      torch::indexing::Slice(),
      torch::indexing::Slice(0, 1)})
    .transpose(1, 2)
    .contiguous();
  pack.features_rest = features.index({
      torch::indexing::Slice(),
      torch::indexing::Slice(),
      torch::indexing::Slice(1, features.size(2))})
    .transpose(1, 2)
    .contiguous();
  pack.scaling = scaling;
  pack.rotation = rotation;
  pack.opacity = opacity;
  pack.foreground_count = num_points;
  return pack;
}

GaussianTensorPack make_skybox_tensors(
  const int skybox_points_num,
  const int sh_degree,
  const double skybox_radius,
  torch::Device device)
{
  GaussianTensorPack pack;
  if (skybox_points_num <= 0) {
    return pack;
  }
  if (sh_degree < 0) {
    throw std::runtime_error("sh_degree must be non-negative");
  }
  if (skybox_radius <= 0.0) {
    throw std::runtime_error("skybox_radius must be positive");
  }

  const int64_t num = static_cast<int64_t>(skybox_points_num);
  const int64_t sh_coeff_count = static_cast<int64_t>(sh_degree + 1) *
    static_cast<int64_t>(sh_degree + 1);
  const auto options = torch::TensorOptions().dtype(torch::kFloat32).device(device);
  const auto pi = torch::acos(torch::tensor(-1.0F, options));
  const auto theta = 2.0F * pi * torch::rand({num}, options);
  const auto phi = torch::acos(1.0F - 1.4F * torch::rand({num}, options));
  const auto sin_phi = torch::sin(phi);
  const float radius = static_cast<float>(skybox_radius * 10.0);

  auto xyz = torch::zeros({num, 3}, options);
  xyz.index_put_({torch::indexing::Slice(), 0}, radius * torch::cos(theta) * sin_phi);
  xyz.index_put_({torch::indexing::Slice(), 1}, radius * torch::sin(theta) * sin_phi);
  xyz.index_put_({torch::indexing::Slice(), 2}, radius * torch::cos(phi));

  auto features = torch::zeros({num, 3, sh_coeff_count}, options);
  features.index_put_({torch::indexing::Slice(), 0, 0}, 0.7F);
  features.index_put_({torch::indexing::Slice(), 1, 0}, 0.8F);
  features.index_put_({torch::indexing::Slice(), 2, 0}, 0.95F);

  constexpr double sphere_area = 4.0 * 3.14159265358979323846;
  const double angular_spacing = std::sqrt(sphere_area / static_cast<double>(skybox_points_num));
  const float scale_seed = static_cast<float>(std::max(radius * angular_spacing, 1e-6));
  auto scaling = std::log(scale_seed) * torch::ones({num, 3}, options);
  auto rotation = torch::zeros({num, 4}, options);
  rotation.index_put_({torch::indexing::Slice(), 0}, 1.0F);
  auto opacity = inverse_sigmoid(0.7F * torch::ones({num, 1}, options));

  pack.xyz = xyz;
  pack.features_dc = features.index({
      torch::indexing::Slice(),
      torch::indexing::Slice(),
      torch::indexing::Slice(0, 1)})
    .transpose(1, 2)
    .contiguous();
  pack.features_rest = features.index({
      torch::indexing::Slice(),
      torch::indexing::Slice(),
      torch::indexing::Slice(1, features.size(2))})
    .transpose(1, 2)
    .contiguous();
  pack.scaling = scaling;
  pack.rotation = rotation;
  pack.opacity = opacity;
  pack.skybox_count = static_cast<size_t>(skybox_points_num);
  return pack;
}

void require_grad_for_map(TorchGaussianMap & map)
{
  map.xyz = map.xyz.contiguous().requires_grad_();
  map.features_dc = map.features_dc.contiguous().requires_grad_();
  map.features_rest = map.features_rest.contiguous().requires_grad_();
  map.scaling = map.scaling.contiguous().requires_grad_();
  map.rotation = map.rotation.contiguous().requires_grad_();
  map.opacity = map.opacity.contiguous().requires_grad_();
}

void zero_gaussian_gradients(TorchGaussianMap & map)
{
  for (auto * tensor : {
      &map.xyz, &map.features_dc, &map.features_rest, &map.scaling, &map.rotation, &map.opacity})
  {
    if (tensor->defined() && tensor->grad().defined()) {
      tensor->grad().zero_();
    }
  }
}

void validate_gaussian_map_for_optimization(const TorchGaussianMap & map)
{
  if (!map.xyz.defined() || !map.features_dc.defined() || !map.opacity.defined()) {
    throw std::runtime_error("cannot optimize an uninitialized Gaussian map");
  }
  if (map.xyz.dim() != 2 || map.xyz.size(1) != 3) {
    throw std::runtime_error("Gaussian xyz tensor must have shape [N, 3]");
  }
  if (map.features_dc.dim() != 3 || map.features_dc.size(1) != 1 || map.features_dc.size(2) != 3) {
    throw std::runtime_error("Gaussian DC feature tensor must have shape [N, 1, 3]");
  }
  if (map.opacity.dim() != 2 || map.opacity.size(1) != 1) {
    throw std::runtime_error("Gaussian opacity tensor must have shape [N, 1]");
  }
  if (map.xyz.size(0) != map.features_dc.size(0) || map.xyz.size(0) != map.opacity.size(0)) {
    throw std::runtime_error("Gaussian optimization tensors have inconsistent point counts");
  }
}

torch::Tensor select_visible_gaussians(
  const TorchGaussianMap & map,
  const TorchCamera & camera,
  const GaussianBackendConfig & config,
  torch::Device device)
{
  const int64_t total_count = map.xyz.size(0);
  const int64_t start_index = static_cast<int64_t>(map.skybox_count);
  if (start_index >= total_count || camera.image_width <= 0 || camera.image_height <= 0) {
    return torch::empty({0}, torch::TensorOptions().dtype(torch::kLong).device(device));
  }

  const auto long_options = torch::TensorOptions().dtype(torch::kLong).device(device);
  const auto float_options = torch::TensorOptions().dtype(torch::kFloat32).device(device);
  const auto xyz = map.xyz.index({torch::indexing::Slice(start_index, torch::indexing::None)})
    .detach()
    .to(device);
  const auto ones = torch::ones({xyz.size(0), 1}, float_options);
  const auto xyz_h = torch::cat({xyz, ones}, 1);
  const auto world_view = camera.world_view_transform.to(device);
  if (world_view.dim() != 2 || world_view.size(0) != 4 || world_view.size(1) != 4) {
    throw std::runtime_error("TorchCamera world_view_transform must have shape [4, 4]");
  }

  const auto xyz_cam = torch::matmul(xyz_h, world_view)
    .index({torch::indexing::Slice(), torch::indexing::Slice(0, 3)});
  const auto z = xyz_cam.index({torch::indexing::Slice(), 2});
  const auto x = xyz_cam.index({torch::indexing::Slice(), 0});
  const auto y = xyz_cam.index({torch::indexing::Slice(), 1});
  const auto u = (static_cast<double>(camera.fx) * x / z) + static_cast<double>(camera.cx);
  const auto v = (static_cast<double>(camera.fy) * y / z) + static_cast<double>(camera.cy);
  const auto u_idx = torch::round(u).to(torch::kLong);
  const auto v_idx = torch::round(v).to(torch::kLong);

  auto valid = torch::logical_and(z.gt(1.0e-3), torch::isfinite(z));
  if (config.max_depth > 0.0) {
    valid = torch::logical_and(valid, z.lt(config.max_depth));
  }
  valid = torch::logical_and(valid, torch::isfinite(u));
  valid = torch::logical_and(valid, torch::isfinite(v));
  valid = torch::logical_and(valid, u_idx.ge(0));
  valid = torch::logical_and(valid, v_idx.ge(0));
  valid = torch::logical_and(valid, u_idx.lt(camera.image_width));
  valid = torch::logical_and(valid, v_idx.lt(camera.image_height));

  auto visible_local = torch::nonzero(valid).flatten().to(long_options);
  if (visible_local.numel() == 0) {
    return visible_local;
  }

  const int max_samples = std::max(config.optimization_max_samples, 0);
  if (max_samples > 0 && visible_local.size(0) > max_samples) {
    visible_local = visible_local.index({
      torch::indexing::Slice(0, static_cast<int64_t>(max_samples))});
  }

  const auto global_offset = torch::full(
    {visible_local.size(0)}, start_index, long_options);
  return visible_local + global_offset;
}

torch::Tensor gather_camera_targets(
  const TorchGaussianMap & map,
  const TorchCamera & camera,
  const torch::Tensor & gaussian_indices,
  torch::Device device)
{
  if (!camera.original_image.defined()) {
    throw std::runtime_error("TorchCamera original_image is not defined");
  }
  const auto image = camera.original_image.to(device).contiguous();
  if (image.dim() != 3 || image.size(0) != 3) {
    throw std::runtime_error("TorchCamera original_image must have shape [3, H, W]");
  }

  const auto xyz = map.xyz.index_select(0, gaussian_indices).detach().to(device);
  const auto ones = torch::ones(
    {xyz.size(0), 1}, torch::TensorOptions().dtype(torch::kFloat32).device(device));
  const auto xyz_h = torch::cat({xyz, ones}, 1);
  const auto xyz_cam = torch::matmul(xyz_h, camera.world_view_transform.to(device))
    .index({torch::indexing::Slice(), torch::indexing::Slice(0, 3)});
  const auto z = xyz_cam.index({torch::indexing::Slice(), 2});
  const auto x = xyz_cam.index({torch::indexing::Slice(), 0});
  const auto y = xyz_cam.index({torch::indexing::Slice(), 1});
  const auto u_idx = torch::round((static_cast<double>(camera.fx) * x / z) + camera.cx)
    .to(torch::kLong);
  const auto v_idx = torch::round((static_cast<double>(camera.fy) * y / z) + camera.cy)
    .to(torch::kLong);

  return image.index({torch::indexing::Slice(), v_idx, u_idx})
    .transpose(0, 1)
    .contiguous()
    .detach();
}

}  // namespace

torch::Tensor cv_mat_to_torch_tensor_float32(
  const cv::Mat & mat,
  torch::Device device,
  const bool use_pinned_memory)
{
  if (mat.empty()) {
    throw std::runtime_error("cannot convert empty cv::Mat to torch tensor");
  }
  if (mat.depth() != CV_32F) {
    throw std::runtime_error("cv_mat_to_torch_tensor_float32 expects CV_32F input");
  }

  torch::Tensor tensor;
  if (mat.channels() == 1) {
    tensor = torch::from_blob(
      const_cast<float *>(mat.ptr<float>()),
      {mat.rows, mat.cols},
      torch::TensorOptions().dtype(torch::kFloat32)).clone();
  } else if (mat.channels() == 3) {
    tensor = torch::from_blob(
      const_cast<float *>(mat.ptr<float>()),
      {mat.rows, mat.cols, mat.channels()},
      torch::TensorOptions().dtype(torch::kFloat32)).clone().permute({2, 0, 1});
  } else {
    throw std::runtime_error("unsupported cv::Mat channel count for torch conversion");
  }

  if (use_pinned_memory && device.is_cpu()) {
    tensor = tensor.pin_memory();
  }
  return tensor.to(device, /*non_blocking=*/use_pinned_memory).contiguous();
}

TorchCamera make_torch_camera(
  const CameraFrameRecord & frame,
  const double fx,
  const double fy,
  const double cx,
  const double cy,
  torch::Device image_device,
  torch::Device geometry_device)
{
  TorchCamera camera;
  camera.image_name = frame.image_name;
  camera.image_width = frame.width;
  camera.image_height = frame.height;
  camera.fx = static_cast<float>(fx);
  camera.fy = static_cast<float>(fy);
  camera.cx = static_cast<float>(cx);
  camera.cy = static_cast<float>(cy);
  camera.fov_x = 2.0F * std::atan(static_cast<float>(frame.width) / (2.0F * camera.fx));
  camera.fov_y = 2.0F * std::atan(static_cast<float>(frame.height) / (2.0F * camera.fy));

  camera.original_image = cv_mat_to_torch_tensor_float32(
    frame.image_rgb_float, image_device, frame.is_keyframe && image_device.is_cpu());
  camera.original_depth = cv_mat_to_torch_tensor_float32(
    frame.depth_m_float, image_device, frame.is_keyframe && image_device.is_cpu());

  camera.world_view_transform = make_world_view_transform(frame.r_wc, frame.t_wc, geometry_device);
  camera.projection_matrix = make_projection_matrix(
    frame.width, frame.height, camera.fx, camera.fy, camera.cx, camera.cy, geometry_device);
  camera.full_proj_transform = (
    camera.world_view_transform.unsqueeze(0).bmm(camera.projection_matrix.unsqueeze(0))).squeeze(0);
  camera.camera_center = camera.world_view_transform.inverse().index({3, torch::indexing::Slice(0, 3)});
  return camera;
}

TorchGaussianMap initialize_gaussian_map(
  const MapperDataset & dataset,
  const int sh_degree,
  const double scaling_scale,
  const double fx,
  const double fy,
  torch::Device device,
  const int skybox_points_num,
  const double skybox_radius)
{
  auto foreground = make_foreground_tensors_from_pending_points(
    dataset, sh_degree, scaling_scale, fx, fy, device);
  auto skybox = make_skybox_tensors(skybox_points_num, sh_degree, skybox_radius, device);

  TorchGaussianMap map;
  if (skybox.skybox_count > 0) {
    map.xyz = torch::cat({skybox.xyz, foreground.xyz}, 0);
    map.features_dc = torch::cat({skybox.features_dc, foreground.features_dc}, 0);
    map.features_rest = torch::cat({skybox.features_rest, foreground.features_rest}, 0);
    map.scaling = torch::cat({skybox.scaling, foreground.scaling}, 0);
    map.rotation = torch::cat({skybox.rotation, foreground.rotation}, 0);
    map.opacity = torch::cat({skybox.opacity, foreground.opacity}, 0);
  } else {
    map.xyz = foreground.xyz;
    map.features_dc = foreground.features_dc;
    map.features_rest = foreground.features_rest;
    map.scaling = foreground.scaling;
    map.rotation = foreground.rotation;
    map.opacity = foreground.opacity;
  }
  require_grad_for_map(map);
  map.sh_degree = sh_degree;
  map.foreground_count = foreground.foreground_count;
  map.skybox_count = skybox.skybox_count;
  return map;
}

TorchGaussianMap initialize_gaussian_map(
  const MapperDataset & dataset,
  const GaussianBackendConfig & config,
  const double fx,
  const double fy,
  torch::Device device)
{
  return initialize_gaussian_map(
    dataset,
    config.sh_degree,
    config.scaling_scale,
    fx,
    fy,
    device,
    config.skybox_points_num,
    config.skybox_radius);
}

size_t append_pending_points_to_gaussian_map(
  TorchGaussianMap & map,
  const MapperDataset & dataset,
  const int sh_degree,
  const double scaling_scale,
  const double fx,
  const double fy,
  torch::Device device)
{
  const size_t num_points = dataset.pending_point_count();
  if (num_points == 0) {
    return 0;
  }
  if (map.sh_degree != sh_degree) {
    throw std::runtime_error("cannot append Gaussian tensors with a different SH degree");
  }
  if (map.xyz.defined() && map.xyz.numel() > 0) {
    device = map.xyz.device();
  }

  auto foreground = make_foreground_tensors_from_pending_points(
    dataset, sh_degree, scaling_scale, fx, fy, device);

  torch::NoGradGuard no_grad;
  map.xyz = torch::cat({map.xyz, foreground.xyz}, 0);
  map.features_dc = torch::cat({map.features_dc, foreground.features_dc}, 0);
  map.features_rest = torch::cat({map.features_rest, foreground.features_rest}, 0);
  map.scaling = torch::cat({map.scaling, foreground.scaling}, 0);
  map.rotation = torch::cat({map.rotation, foreground.rotation}, 0);
  map.opacity = torch::cat({map.opacity, foreground.opacity}, 0);
  require_grad_for_map(map);
  map.foreground_count += foreground.foreground_count;
  return foreground.foreground_count;
}

size_t append_pending_points_to_gaussian_map(
  TorchGaussianMap & map,
  const MapperDataset & dataset,
  const GaussianBackendConfig & config,
  const double fx,
  const double fy,
  torch::Device device)
{
  return append_pending_points_to_gaussian_map(
    map, dataset, config.sh_degree, config.scaling_scale, fx, fy, device);
}

TorchOptimizationResult optimize_gaussian_map_from_camera(
  TorchGaussianMap & map,
  const TorchCamera & camera,
  const GaussianBackendConfig & config,
  const int steps,
  torch::Device device)
{
  TorchOptimizationResult result;
  if (!config.enable_photometric_optimization || steps <= 0) {
    return result;
  }

  validate_gaussian_map_for_optimization(map);
  if (map.xyz.defined() && map.xyz.numel() > 0) {
    device = map.xyz.device();
  }
  if (config.feature_lr <= 0.0 && config.opacity_lr <= 0.0) {
    throw std::runtime_error("photometric optimization requires a positive feature_lr or opacity_lr");
  }

  require_grad_for_map(map);
  zero_gaussian_gradients(map);

  const auto gaussian_indices = select_visible_gaussians(map, camera, config, device);
  result.supervised_count = static_cast<size_t>(gaussian_indices.numel());
  if (result.supervised_count == 0) {
    return result;
  }

  const auto targets = gather_camera_targets(map, camera, gaussian_indices, device);
  constexpr float sh_c0 = 0.28209479177387814F;
  for (int step = 0; step < steps; ++step) {
    zero_gaussian_gradients(map);
    const auto dc = map.features_dc.index_select(0, gaussian_indices).select(1, 0);
    const auto predicted = torch::clamp(dc * sh_c0 + 0.5F, 0.0F, 1.0F);
    auto loss = torch::mean(torch::abs(predicted - targets));
    if (config.opacity_lr > 0.0) {
      const auto opacity = map.opacity.index_select(0, gaussian_indices);
      const auto opacity_regularizer = torch::mean(torch::pow(torch::sigmoid(opacity) - 0.5F, 2));
      loss = loss + 0.001F * opacity_regularizer;
    }
    loss.backward();

    {
      torch::NoGradGuard no_grad;
      if (config.feature_lr > 0.0 && map.features_dc.grad().defined()) {
        map.features_dc.add_(map.features_dc.grad(), -config.feature_lr);
      }
      if (config.opacity_lr > 0.0 && map.opacity.grad().defined()) {
        map.opacity.add_(map.opacity.grad(), -config.opacity_lr);
      }
      result.photometric_l1 = loss.detach().to(torch::kCPU).item<float>();
    }
    ++result.steps;
  }

  zero_gaussian_gradients(map);
  require_grad_for_map(map);
  return result;
}

TorchPruneResult prune_gaussian_map(
  TorchGaussianMap & map,
  const GaussianBackendConfig & config)
{
  TorchPruneResult result;
  result.before_count = map.foreground_count + map.skybox_count;
  if (!config.enable_density_control) {
    result.after_count = result.before_count;
    return result;
  }
  validate_gaussian_map_for_optimization(map);
  const int64_t total_count = map.xyz.size(0);
  const int64_t skybox_count = static_cast<int64_t>(map.skybox_count);
  const int64_t foreground_count = total_count - skybox_count;
  if (foreground_count <= 0) {
    result.after_count = result.before_count;
    return result;
  }

  const auto device = map.xyz.device();
  const auto long_options = torch::TensorOptions().dtype(torch::kLong).device(device);
  const auto foreground_slice = torch::indexing::Slice(skybox_count, torch::indexing::None);
  const auto opacity = torch::sigmoid(map.opacity.index({foreground_slice}).detach()).flatten();

  torch::Tensor keep_foreground;
  if (config.prune_min_opacity > 0.0) {
    keep_foreground = torch::nonzero(opacity.ge(config.prune_min_opacity)).flatten().to(long_options);
  } else {
    keep_foreground = torch::arange(foreground_count, long_options);
  }

  const int max_foreground = std::max(config.max_foreground_gaussians, 0);
  if (max_foreground > 0 && keep_foreground.size(0) > max_foreground) {
    const auto kept_scores = opacity.index_select(0, keep_foreground);
    const auto topk = kept_scores.topk(static_cast<int64_t>(max_foreground), 0, true, true);
    keep_foreground = keep_foreground.index_select(0, std::get<1>(topk));
    keep_foreground = std::get<0>(keep_foreground.sort());
  }

  torch::Tensor keep_indices;
  if (skybox_count > 0) {
    const auto keep_skybox = torch::arange(skybox_count, long_options);
    keep_indices = torch::cat({keep_skybox, keep_foreground + skybox_count}, 0);
  } else {
    keep_indices = keep_foreground;
  }

  const size_t kept_foreground = static_cast<size_t>(keep_foreground.size(0));
  if (kept_foreground == map.foreground_count && keep_indices.size(0) == total_count) {
    result.after_count = result.before_count;
    return result;
  }

  torch::NoGradGuard no_grad;
  map.xyz = map.xyz.index_select(0, keep_indices).contiguous();
  map.features_dc = map.features_dc.index_select(0, keep_indices).contiguous();
  map.features_rest = map.features_rest.index_select(0, keep_indices).contiguous();
  map.scaling = map.scaling.index_select(0, keep_indices).contiguous();
  map.rotation = map.rotation.index_select(0, keep_indices).contiguous();
  map.opacity = map.opacity.index_select(0, keep_indices).contiguous();
  map.foreground_count = kept_foreground;
  require_grad_for_map(map);

  result.after_count = map.foreground_count + map.skybox_count;
  result.removed_count = result.before_count > result.after_count ?
    result.before_count - result.after_count : 0U;
  return result;
}

}  // namespace gaussian_lic_mapping
