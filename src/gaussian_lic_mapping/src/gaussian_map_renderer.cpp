// SPDX-License-Identifier: GPL-3.0-or-later
//
// Option-A standalone renderer: re-render an on-disk 3DGS map (point_cloud.ply)
// at a warm-started continuous-time (CT) body trajectory and emit per-frame PNGs
// keyed by the OBSERVED image header stamp. Reuses the existing CUDA rasterizer
// (render_gaussian_map_from_camera) entirely OUTSIDE the C++ CT node. No torch in
// the CT package; this tool lives only in gaussian_lic_mapping.
//
// Usage:
//   gaussian_map_renderer <point_cloud.ply> <ct_seed.tum> <observed_stamps.json>
//                         <out_dir>
//
// observed_stamps.json: a JSON array of {"ns": <int>, ...} produced from the
// source bag /camera/image header stamps (see /tmp/extract_image_stamps.py).

#include <cuda_runtime.h>
#include <torch/torch.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <gaussian_lic_mapping/backend_config.hpp>
#include <gaussian_lic_mapping/mapper_dataset.hpp>
#include <gaussian_lic_mapping/torch_backend.hpp>

namespace
{

// --- Camera intrinsics + extrinsic (verified from runtime_config.yaml +
// continuous_time_native_reference_parity.sh:130-131). ---
constexpr double kFx = 646.78472;
constexpr double kFy = 646.65775;
constexpr double kCx = 313.456795;
constexpr double kCy = 261.399612;
constexpr int kWidth = 640;
constexpr int kHeight = 512;

// camera->IMU(body) extrinsic, xyzw
constexpr double kCamImuQx = -0.4991948721;
constexpr double kCamImuQy = 0.5038197882;
constexpr double kCamImuQz = -0.4930665852;
constexpr double kCamImuQw = 0.5038406923;
constexpr double kCamImuTx = 0.0673699;
constexpr double kCamImuTy = 0.0412418;
constexpr double kCamImuTz = 0.0764217;

struct TumPose
{
  double stamp_s{0.0};
  Eigen::Quaterniond q;  // body->world rotation
  Eigen::Vector3d t;     // body position in world
};

// Stream-parse the ASCII PLY into 62 float columns/row. Custom fast float parse
// to avoid the heavy operator>> path on 1.18 GB.
struct PlyColumns
{
  int64_t n{0};
  std::vector<float> xyz;        // n*3
  std::vector<float> dc;         // n*3  (f_dc_0..2)
  std::vector<float> rest;       // n*45 (f_rest_0..44)
  std::vector<float> opacity;    // n*1
  std::vector<float> scaling;    // n*3
  std::vector<float> rotation;   // n*4
};

bool parse_ply(const std::string & path, PlyColumns & out)
{
  std::ifstream in(path, std::ios::binary);
  if (!in.is_open()) {
    std::cerr << "failed to open PLY " << path << "\n";
    return false;
  }
  std::string line;
  int64_t vertex_count = -1;
  // header
  while (std::getline(in, line)) {
    if (line.rfind("element vertex", 0) == 0) {
      vertex_count = std::stoll(line.substr(std::string("element vertex").size()));
    }
    if (line.rfind("end_header", 0) == 0) {
      break;
    }
  }
  if (vertex_count <= 0) {
    std::cerr << "PLY header missing vertex count\n";
    return false;
  }
  out.n = vertex_count;
  out.xyz.resize(static_cast<size_t>(vertex_count) * 3U);
  out.dc.resize(static_cast<size_t>(vertex_count) * 3U);
  out.rest.resize(static_cast<size_t>(vertex_count) * 45U);
  out.opacity.resize(static_cast<size_t>(vertex_count) * 1U);
  out.scaling.resize(static_cast<size_t>(vertex_count) * 3U);
  out.rotation.resize(static_cast<size_t>(vertex_count) * 4U);

  // Read remainder in large chunks; parse 59 floats per row with strtof.
  constexpr int kCols = 59;  // 3 + 3 + 45 + 1 + 3 + 4 = 59
  std::string rest_of_file;
  {
    std::ostringstream ss;
    ss << in.rdbuf();
    rest_of_file = ss.str();
  }
  const char * p = rest_of_file.c_str();
  const char * end = p + rest_of_file.size();
  float row[kCols];
  for (int64_t r = 0; r < vertex_count; ++r) {
    for (int c = 0; c < kCols; ++c) {
      // skip whitespace
      while (p < end && (*p == ' ' || *p == '\n' || *p == '\r' || *p == '\t')) {
        ++p;
      }
      if (p >= end) {
        std::cerr << "PLY truncated at row " << r << " col " << c << "\n";
        return false;
      }
      char * np = nullptr;
      row[c] = std::strtof(p, &np);
      if (np == p) {
        std::cerr << "PLY parse error at row " << r << " col " << c << "\n";
        return false;
      }
      p = np;
    }
    const size_t i = static_cast<size_t>(r);
    out.xyz[i * 3 + 0] = row[0];
    out.xyz[i * 3 + 1] = row[1];
    out.xyz[i * 3 + 2] = row[2];
    out.dc[i * 3 + 0] = row[3];
    out.dc[i * 3 + 1] = row[4];
    out.dc[i * 3 + 2] = row[5];
    for (int k = 0; k < 45; ++k) {
      out.rest[i * 45 + static_cast<size_t>(k)] = row[6 + k];
    }
    out.opacity[i] = row[51];
    out.scaling[i * 3 + 0] = row[52];
    out.scaling[i * 3 + 1] = row[53];
    out.scaling[i * 3 + 2] = row[54];
    out.rotation[i * 4 + 0] = row[55];
    out.rotation[i * 4 + 1] = row[56];
    out.rotation[i * 4 + 2] = row[57];
    out.rotation[i * 4 + 3] = row[58];
  }
  return true;
}

gaussian_lic_mapping::TorchGaussianMap build_map(const PlyColumns & c, torch::Device device)
{
  const auto cpu = torch::TensorOptions().dtype(torch::kFloat32);
  const int64_t n = c.n;

  auto xyz = torch::from_blob(const_cast<float *>(c.xyz.data()), {n, 3}, cpu).clone();
  // f_dc: 3 cols on disk = [N,3] -> reshape[N,3,1].transpose(1,2) -> [N,1,3]
  auto dc = torch::from_blob(const_cast<float *>(c.dc.data()), {n, 3}, cpu)
              .clone()
              .reshape({n, 3, 1})
              .transpose(1, 2)
              .contiguous();  // [N,1,3]
  // f_rest: 45 cols on disk = [N,45] -> reshape[N,3,15].transpose(1,2) -> [N,15,3]
  auto rest = torch::from_blob(const_cast<float *>(c.rest.data()), {n, 45}, cpu)
                .clone()
                .reshape({n, 3, 15})
                .transpose(1, 2)
                .contiguous();  // [N,15,3]
  auto opacity = torch::from_blob(const_cast<float *>(c.opacity.data()), {n, 1}, cpu).clone();
  auto scaling = torch::from_blob(const_cast<float *>(c.scaling.data()), {n, 3}, cpu).clone();
  auto rotation = torch::from_blob(const_cast<float *>(c.rotation.data()), {n, 4}, cpu).clone();

  gaussian_lic_mapping::TorchGaussianMap map;
  map.xyz = xyz.to(device).contiguous();
  map.features_dc = dc.to(device).contiguous();
  map.features_rest = rest.to(device).contiguous();
  map.opacity = opacity.to(device).contiguous();   // RAW (rasterizer applies sigmoid)
  map.scaling = scaling.to(device).contiguous();    // RAW (rasterizer applies exp)
  map.rotation = rotation.to(device).contiguous();  // RAW (rasterizer normalizes)
  map.sh_degree = 3;
  map.foreground_count = static_cast<size_t>(n);
  map.skybox_count = 0;
  return map;
}

bool parse_tum(const std::string & path, std::vector<TumPose> & poses)
{
  std::ifstream in(path);
  if (!in.is_open()) {
    std::cerr << "failed to open TUM " << path << "\n";
    return false;
  }
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream ss(line);
    double s, tx, ty, tz, qx, qy, qz, qw;
    if (!(ss >> s >> tx >> ty >> tz >> qx >> qy >> qz >> qw)) {
      continue;
    }
    TumPose p;
    p.stamp_s = s;
    p.q = Eigen::Quaterniond(qw, qx, qy, qz).normalized();
    p.t = Eigen::Vector3d(tx, ty, tz);
    poses.push_back(p);
  }
  std::sort(poses.begin(), poses.end(), [](const TumPose & a, const TumPose & b) {
    return a.stamp_s < b.stamp_s;
  });
  return !poses.empty();
}

// Naive JSON-array-of-objects ns extractor: pull every integer following "ns":
std::vector<int64_t> parse_stamps_ns(const std::string & path)
{
  std::vector<int64_t> out;
  std::ifstream in(path);
  std::ostringstream ss;
  ss << in.rdbuf();
  const std::string text = ss.str();
  const std::string key = "\"ns\":";
  size_t pos = 0;
  while ((pos = text.find(key, pos)) != std::string::npos) {
    pos += key.size();
    while (pos < text.size() && (text[pos] == ' ' || text[pos] == '\t')) {
      ++pos;
    }
    size_t startp = pos;
    while (pos < text.size() && (std::isdigit(static_cast<unsigned char>(text[pos])))) {
      ++pos;
    }
    if (pos > startp) {
      out.push_back(std::stoll(text.substr(startp, pos - startp)));
    }
  }
  return out;
}

// SE3 interpolate body pose at time s (clamped to trajectory endpoints).
void interp_pose(
  const std::vector<TumPose> & poses, double s, Eigen::Quaterniond & q_out,
  Eigen::Vector3d & t_out)
{
  if (s <= poses.front().stamp_s) {
    q_out = poses.front().q;
    t_out = poses.front().t;
    return;
  }
  if (s >= poses.back().stamp_s) {
    q_out = poses.back().q;
    t_out = poses.back().t;
    return;
  }
  // binary search for upper bound
  size_t lo = 0, hi = poses.size() - 1;
  while (hi - lo > 1) {
    size_t mid = (lo + hi) / 2;
    if (poses[mid].stamp_s <= s) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  const auto & a = poses[lo];
  const auto & b = poses[hi];
  const double denom = (b.stamp_s - a.stamp_s);
  const double u = denom > 1e-12 ? (s - a.stamp_s) / denom : 0.0;
  q_out = a.q.slerp(u, b.q);
  t_out = (1.0 - u) * a.t + u * b.t;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 5) {
    std::cerr << "usage: gaussian_map_renderer <ply> <ct_tum> <stamps_json> <out_dir>\n";
    return 2;
  }
  const std::string ply_path = argv[1];
  const std::string tum_path = argv[2];
  const std::string stamps_path = argv[3];
  const std::string out_dir = argv[4];

  if (!torch::cuda::is_available()) {
    std::cerr << "CUDA is not available\n";
    return 2;
  }
  const auto device = torch::Device(torch::kCUDA);

  std::cout << "[renderer] parsing PLY " << ply_path << std::endl;
  PlyColumns cols;
  if (!parse_ply(ply_path, cols)) {
    return 1;
  }
  std::cout << "[renderer] PLY N=" << cols.n << std::endl;

  auto map = build_map(cols, device);
  // free host PLY buffers
  PlyColumns().xyz.swap(cols.xyz);
  PlyColumns().rest.swap(cols.rest);
  std::cout << "[renderer] map on CUDA: xyz=" << map.xyz.sizes()
            << " dc=" << map.features_dc.sizes()
            << " rest=" << map.features_rest.sizes()
            << " opacity=" << map.opacity.sizes()
            << " scaling=" << map.scaling.sizes()
            << " rotation=" << map.rotation.sizes() << std::endl;

  std::vector<TumPose> poses;
  if (!parse_tum(tum_path, poses)) {
    std::cerr << "failed to parse TUM poses\n";
    return 1;
  }
  std::cout << "[renderer] TUM poses=" << poses.size()
            << " span=[" << poses.front().stamp_s << "," << poses.back().stamp_s << "]"
            << std::endl;

  const auto stamps = parse_stamps_ns(stamps_path);
  if (stamps.empty()) {
    std::cerr << "no observed stamps parsed\n";
    return 1;
  }
  std::cout << "[renderer] observed stamps=" << stamps.size() << std::endl;

  const Eigen::Quaterniond q_cam_to_imu =
    Eigen::Quaterniond(kCamImuQw, kCamImuQx, kCamImuQy, kCamImuQz).normalized();
  const Eigen::Vector3d p_cam_to_imu(kCamImuTx, kCamImuTy, kCamImuTz);

  gaussian_lic_mapping::GaussianBackendConfig config;
  config.white_background = false;

  // dummy image/depth (rasterizer only reads geometry for render)
  const cv::Mat dummy_img(kHeight, kWidth, CV_32FC3, cv::Scalar(0, 0, 0));
  const cv::Mat dummy_depth(kHeight, kWidth, CV_32FC1, cv::Scalar(0));

  std::string mk = std::string("mkdir -p '") + out_dir + "'";
  if (std::system(mk.c_str()) != 0) {
    std::cerr << "failed to create out_dir\n";
    return 1;
  }

  torch::NoGradGuard no_grad;
  int64_t written = 0;
  int64_t skipped = 0;
  float global_max = 0.0F;
  std::ofstream stamps_out(out_dir + "/stamps.json");
  stamps_out << "[";
  bool first_json = true;

  for (size_t idx = 0; idx < stamps.size(); ++idx) {
    const int64_t ns = stamps[idx];
    const double s = static_cast<double>(ns) * 1e-9;

    Eigen::Quaterniond q_body;
    Eigen::Vector3d t_body;
    interp_pose(poses, s, q_body, t_body);

    gaussian_lic_mapping::CameraFrameRecord frame;
    frame.frame_index = idx;
    frame.is_keyframe = false;
    frame.image_name = std::to_string(ns);
    frame.width = kWidth;
    frame.height = kHeight;
    frame.image_rgb_float = dummy_img;
    frame.depth_m_float = dummy_depth;
    // EXACT mapping_node composition (mapping_node.cpp:1181-1182)
    frame.r_wc = (q_body.normalized() * q_cam_to_imu).normalized().toRotationMatrix();
    frame.t_wc = t_body + q_body.normalized() * p_cam_to_imu;

    const auto camera = gaussian_lic_mapping::make_torch_camera(
      frame, kFx, kFy, kCx, kCy, device, device);
    const auto result =
      gaussian_lic_mapping::render_gaussian_map_from_camera(map, camera, config, device);

    // rendered_image [3,H,W] float in [0,1] -> HWC bgr8
    auto img = result.rendered_image.detach().clamp(0.0F, 1.0F).to(torch::kCPU).contiguous();
    const float fmax = img.max().item<float>();
    global_max = std::max(global_max, fmax);
    auto hwc = img.permute({1, 2, 0}).contiguous();  // H,W,3 (RGB)
    auto u8 = (hwc * 255.0F).round().clamp(0.0F, 255.0F).to(torch::kU8).contiguous();
    cv::Mat rgb(kHeight, kWidth, CV_8UC3, u8.data_ptr<uint8_t>());
    cv::Mat bgr;
    cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
    const std::string png = out_dir + "/" + std::to_string(ns) + ".png";
    if (!cv::imwrite(png, bgr)) {
      std::cerr << "imwrite failed for " << png << "\n";
      return 1;
    }
    if (!first_json) {
      stamps_out << ",";
    }
    stamps_out << ns;
    first_json = false;
    ++written;

    if (idx % 100 == 0 || idx + 1 == stamps.size()) {
      std::cout << "[renderer] frame " << idx << "/" << stamps.size()
                << " ns=" << ns << " visible=" << result.visible_count
                << " max=" << fmax << std::endl;
    }
  }
  stamps_out << "]";
  stamps_out.close();
  cudaDeviceSynchronize();

  std::cout << "[renderer] done written=" << written << " skipped=" << skipped
            << " global_max=" << global_max << std::endl;
  if (written == 0 || global_max <= 0.0F) {
    std::cerr << "[renderer] produced no non-empty frames\n";
    return 1;
  }
  return 0;
}
