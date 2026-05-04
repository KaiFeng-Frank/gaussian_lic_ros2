// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include <NvInfer.h>

namespace gaussian_lic_mapping
{

class DepthCompleter
{
public:
  DepthCompleter(const std::string & engine_path, int input_width, int input_height);
  ~DepthCompleter();

  DepthCompleter(const DepthCompleter &) = delete;
  DepthCompleter & operator=(const DepthCompleter &) = delete;

  cv::Mat complete(const cv::Mat & rgb_image, const cv::Mat & sparse_depth_m);

private:
  struct InferDeleter
  {
    template<typename T>
    void operator()(T * obj) const
    {
      delete obj;
    }
  };

  class Logger final : public nvinfer1::ILogger
  {
  public:
    void log(nvinfer1::ILogger::Severity severity, const char * msg) noexcept override;
  };

  void init_engine(const std::string & engine_path);
  std::vector<char> read_file(const std::string & filename) const;
  void allocate_buffers();
  size_t volume(const nvinfer1::Dims & dims) const;
  void prepare_inputs(const cv::Mat & rgb_image, const cv::Mat & sparse_depth_m);
  cv::Mat process_output();

  Logger logger_;
  std::unique_ptr<nvinfer1::IRuntime, InferDeleter> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine, InferDeleter> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext, InferDeleter> context_;
  std::vector<void *> device_buffers_;
  std::vector<std::vector<float>> host_buffers_;
  int input_width_{0};
  int input_height_{0};
};

}  // namespace gaussian_lic_mapping
