// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_mapping/depth_completer.hpp>

#include <cuda_runtime_api.h>

#include <cstring>
#include <fstream>
#include <iostream>
#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace gaussian_lic_mapping
{

DepthCompleter::DepthCompleter(
  const std::string & engine_path,
  const int input_width,
  const int input_height)
: input_width_(input_width),
  input_height_(input_height)
{
  if (input_width_ <= 0 || input_height_ <= 0) {
    throw std::runtime_error("DepthCompleter input dimensions must be positive");
  }
  init_engine(engine_path);
}

DepthCompleter::~DepthCompleter()
{
  context_.reset();
  engine_.reset();
  runtime_.reset();
  for (void * buffer : device_buffers_) {
    if (buffer != nullptr) {
      cudaFree(buffer);
    }
  }
}

void DepthCompleter::Logger::log(nvinfer1::ILogger::Severity severity, const char * msg) noexcept
{
  if (severity <= nvinfer1::ILogger::Severity::kWARNING) {
    std::cout << "[TensorRT] " << msg << std::endl;
  }
}

cv::Mat DepthCompleter::complete(const cv::Mat & rgb_image, const cv::Mat & sparse_depth_m)
{
  if (rgb_image.empty() || sparse_depth_m.empty()) {
    throw std::runtime_error("DepthCompleter expects non-empty RGB and sparse depth inputs");
  }
  if (rgb_image.rows != input_height_ || rgb_image.cols != input_width_) {
    throw std::runtime_error("DepthCompleter RGB image dimensions do not match the TensorRT engine");
  }
  if (sparse_depth_m.rows != input_height_ || sparse_depth_m.cols != input_width_) {
    throw std::runtime_error("DepthCompleter sparse depth dimensions do not match the TensorRT engine");
  }

  cv::Mat rgb_float;
  if (rgb_image.type() == CV_32FC3) {
    rgb_float = rgb_image;
  } else {
    rgb_image.convertTo(rgb_float, CV_32FC3, 1.0 / 255.0);
  }

  cv::Mat depth_float;
  sparse_depth_m.convertTo(depth_float, CV_32F, 1.0F / 200.0F);
  prepare_inputs(rgb_float, depth_float);

  if (!context_->executeV2(device_buffers_.data())) {
    throw std::runtime_error("TensorRT depth completion inference failed");
  }
  return process_output();
}

void DepthCompleter::init_engine(const std::string & engine_path)
{
  const auto engine_data = read_file(engine_path);
  runtime_.reset(nvinfer1::createInferRuntime(logger_));
  if (!runtime_) {
    throw std::runtime_error("failed to create TensorRT runtime");
  }
  engine_.reset(runtime_->deserializeCudaEngine(engine_data.data(), engine_data.size()));
  if (!engine_) {
    throw std::runtime_error("failed to deserialize TensorRT depth completion engine: " + engine_path);
  }
  context_.reset(engine_->createExecutionContext());
  if (!context_) {
    throw std::runtime_error("failed to create TensorRT depth completion execution context");
  }
  allocate_buffers();
}

std::vector<char> DepthCompleter::read_file(const std::string & filename) const
{
  std::ifstream file(filename, std::ios::binary | std::ios::ate);
  if (!file) {
    throw std::runtime_error("unable to open TensorRT engine: " + filename);
  }
  const std::streamsize size = file.tellg();
  if (size <= 0) {
    throw std::runtime_error("TensorRT engine is empty: " + filename);
  }
  file.seekg(0, std::ios::beg);
  std::vector<char> buffer(static_cast<size_t>(size));
  if (!file.read(buffer.data(), size)) {
    throw std::runtime_error("failed to read TensorRT engine: " + filename);
  }
  return buffer;
}

void DepthCompleter::allocate_buffers()
{
  const int binding_count = engine_->getNbBindings();
  if (binding_count < 4) {
    throw std::runtime_error("TensorRT depth completion engine must expose RGB, depth, mask, and output bindings");
  }
  device_buffers_.assign(static_cast<size_t>(binding_count), nullptr);
  host_buffers_.resize(static_cast<size_t>(binding_count));

  for (int i = 0; i < binding_count; ++i) {
    const nvinfer1::Dims dims = engine_->getBindingDimensions(i);
    const size_t element_count = volume(dims);
    host_buffers_[static_cast<size_t>(i)].resize(element_count);
    if (cudaMalloc(&device_buffers_[static_cast<size_t>(i)], element_count * sizeof(float)) != cudaSuccess) {
      throw std::runtime_error("CUDA memory allocation failed for TensorRT depth completion");
    }
  }
}

size_t DepthCompleter::volume(const nvinfer1::Dims & dims) const
{
  size_t result = 1;
  for (int i = 0; i < dims.nbDims; ++i) {
    result *= static_cast<size_t>(dims.d[i]);
  }
  return result;
}

void DepthCompleter::prepare_inputs(const cv::Mat & rgb_image, const cv::Mat & sparse_depth_m)
{
  std::vector<cv::Mat> rgb_channels(3);
  cv::split(rgb_image, rgb_channels);
  const size_t plane_size = static_cast<size_t>(input_height_) * static_cast<size_t>(input_width_);
  for (int channel = 0; channel < 3; ++channel) {
    std::memcpy(
      host_buffers_[0].data() + static_cast<size_t>(channel) * plane_size,
      rgb_channels[static_cast<size_t>(channel)].data,
      plane_size * sizeof(float));
  }

  std::memcpy(host_buffers_[1].data(), sparse_depth_m.data, plane_size * sizeof(float));

  cv::Mat mask = sparse_depth_m > 0.0F;
  mask.convertTo(mask, CV_32F, 1.0 / 255.0);
  std::memcpy(host_buffers_[2].data(), mask.data, plane_size * sizeof(float));

  for (size_t i = 0; i + 1U < device_buffers_.size(); ++i) {
    if (cudaMemcpy(
        device_buffers_[i],
        host_buffers_[i].data(),
        host_buffers_[i].size() * sizeof(float),
        cudaMemcpyHostToDevice) != cudaSuccess)
    {
      throw std::runtime_error("CUDA H2D copy failed for TensorRT depth completion");
    }
  }
}

cv::Mat DepthCompleter::process_output()
{
  auto & output = host_buffers_.back();
  if (cudaMemcpy(
      output.data(),
      device_buffers_.back(),
      output.size() * sizeof(float),
      cudaMemcpyDeviceToHost) != cudaSuccess)
  {
    throw std::runtime_error("CUDA D2H copy failed for TensorRT depth completion");
  }

  cv::Mat result(input_height_, input_width_, CV_32F, output.data());
  return result.clone() * 200.0F;
}

}  // namespace gaussian_lic_mapping
