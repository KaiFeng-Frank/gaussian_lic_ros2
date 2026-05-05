// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_mapping/depth_completer.hpp>

#include <cuda_runtime_api.h>

#include <algorithm>
#include <cctype>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace gaussian_lic_mapping
{

namespace
{

nvinfer1::Dims make_fixed_spnet_dims(
  const std::string & name,
  const int input_width,
  const int input_height)
{
  nvinfer1::Dims dims{};
  dims.nbDims = 4;
  dims.d[0] = 1;
  dims.d[1] = name.find("rgb") != std::string::npos ? 3 : 1;
  dims.d[2] = input_height;
  dims.d[3] = input_width;
  return dims;
}

std::string dims_to_string(const nvinfer1::Dims & dims)
{
  std::ostringstream out;
  out << "[";
  for (int i = 0; i < dims.nbDims; ++i) {
    if (i > 0) {
      out << "x";
    }
    out << dims.d[i];
  }
  out << "]";
  return out.str();
}

}  // namespace

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

  if (!run_inference()) {
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
#if NV_TENSORRT_MAJOR >= 10
  const int binding_count = engine_->getNbIOTensors();
#else
  const int binding_count = engine_->getNbBindings();
#endif
  if (binding_count < 4) {
    throw std::runtime_error("TensorRT depth completion engine must expose RGB, depth, mask, and output bindings");
  }
  device_buffers_.assign(static_cast<size_t>(binding_count), nullptr);
  host_buffers_.resize(static_cast<size_t>(binding_count));
  tensor_names_.clear();
  input_indices_.clear();
  rgb_index_ = -1;
  depth_index_ = -1;
  mask_index_ = -1;
  output_index_ = -1;

  for (int i = 0; i < binding_count; ++i) {
#if NV_TENSORRT_MAJOR >= 10
    const char * tensor_name = engine_->getIOTensorName(i);
    if (tensor_name == nullptr) {
      throw std::runtime_error("TensorRT engine returned a null tensor name");
    }
    const std::string name(tensor_name);
    tensor_names_.push_back(name);
    const bool is_input = engine_->getTensorIOMode(tensor_name) == nvinfer1::TensorIOMode::kINPUT;
    if (is_input) {
      const nvinfer1::Dims fixed_dims = make_fixed_spnet_dims(name, input_width_, input_height_);
      if (!context_->setInputShape(tensor_name, fixed_dims)) {
        throw std::runtime_error(
                "failed to set TensorRT input shape for " + name + " to " + dims_to_string(fixed_dims));
      }
      input_indices_.push_back(i);
    }
    assign_tensor_index(name, i, is_input);
  }

  for (int i = 0; i < binding_count; ++i) {
    const char * tensor_name = tensor_names_[static_cast<size_t>(i)].c_str();
    nvinfer1::Dims dims = context_->getTensorShape(tensor_name);
    if (dims.nbDims <= 0) {
      dims = engine_->getTensorShape(tensor_name);
    }
#else
    const nvinfer1::Dims dims = engine_->getBindingDimensions(i);
    const char * binding_name = engine_->getBindingName(i);
    const bool is_input = engine_->bindingIsInput(i);
    assign_tensor_index(binding_name == nullptr ? "" : std::string(binding_name), i, is_input);
    if (is_input) {
      input_indices_.push_back(i);
    }
#endif
    const size_t element_count = volume(dims);
    if (element_count == 0U) {
      throw std::runtime_error("TensorRT tensor has an empty shape: " + dims_to_string(dims));
    }
    host_buffers_[static_cast<size_t>(i)].resize(element_count);
    if (cudaMalloc(&device_buffers_[static_cast<size_t>(i)], element_count * sizeof(float)) != cudaSuccess) {
      throw std::runtime_error("CUDA memory allocation failed for TensorRT depth completion");
    }
#if NV_TENSORRT_MAJOR >= 10
    if (!context_->setTensorAddress(tensor_name, device_buffers_[static_cast<size_t>(i)])) {
      throw std::runtime_error("failed to bind TensorRT tensor address for " + tensor_names_[static_cast<size_t>(i)]);
    }
#endif
  }

  if (
    rgb_index_ < 0 || depth_index_ < 0 || mask_index_ < 0 || output_index_ < 0 ||
    rgb_index_ >= binding_count || depth_index_ >= binding_count ||
    mask_index_ >= binding_count || output_index_ >= binding_count)
  {
    throw std::runtime_error("TensorRT SPNet engine must expose rgb, depth, mask, and one output tensor");
  }
}

void DepthCompleter::assign_tensor_index(const std::string & name, const int index, const bool is_input)
{
  std::string lower = name;
  std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
  if (is_input && lower.find("rgb") != std::string::npos) {
    rgb_index_ = index;
  } else if (is_input && lower.find("depth") != std::string::npos) {
    depth_index_ = index;
  } else if (is_input && lower.find("mask") != std::string::npos) {
    mask_index_ = index;
  } else if (!is_input && output_index_ < 0) {
    output_index_ = index;
  }
}

bool DepthCompleter::run_inference()
{
#if NV_TENSORRT_MAJOR >= 10
  for (size_t i = 0; i < tensor_names_.size(); ++i) {
    if (!context_->setTensorAddress(tensor_names_[i].c_str(), device_buffers_[i])) {
      return false;
    }
  }
  cudaStream_t stream = nullptr;
  return context_->enqueueV3(stream);
#else
  return context_->executeV2(device_buffers_.data());
#endif
}

size_t DepthCompleter::volume(const nvinfer1::Dims & dims) const
{
  size_t result = 1;
  for (int i = 0; i < dims.nbDims; ++i) {
    if (dims.d[i] <= 0) {
      return 0;
    }
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
      host_buffers_[static_cast<size_t>(rgb_index_)].data() + static_cast<size_t>(channel) * plane_size,
      rgb_channels[static_cast<size_t>(channel)].data,
      plane_size * sizeof(float));
  }

  std::memcpy(host_buffers_[static_cast<size_t>(depth_index_)].data(), sparse_depth_m.data, plane_size * sizeof(float));

  cv::Mat mask = sparse_depth_m > 0.0F;
  mask.convertTo(mask, CV_32F, 1.0 / 255.0);
  std::memcpy(host_buffers_[static_cast<size_t>(mask_index_)].data(), mask.data, plane_size * sizeof(float));

  for (const int input_index : input_indices_) {
    const size_t i = static_cast<size_t>(input_index);
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
  auto & output = host_buffers_[static_cast<size_t>(output_index_)];
  if (cudaMemcpy(
      output.data(),
      device_buffers_[static_cast<size_t>(output_index_)],
      output.size() * sizeof(float),
      cudaMemcpyDeviceToHost) != cudaSuccess)
  {
    throw std::runtime_error("CUDA D2H copy failed for TensorRT depth completion");
  }

  cv::Mat result(input_height_, input_width_, CV_32F, output.data());
  return result.clone() * 200.0F;
}

}  // namespace gaussian_lic_mapping
