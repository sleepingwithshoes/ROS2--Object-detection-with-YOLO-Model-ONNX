#pragma once

#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>
#include <memory>

namespace yolo_onnx
{

  struct Detection
  {
    int class_id;
    float confidence;
    cv::Rect box;
    std::string class_name;
    cv::Scalar box_color;
  };

  struct YoloOnnxConfig
  {
    std::string model_path;
    std::vector<std::string> class_names;
    std::vector<cv::Scalar> box_colors;
    float conf_threshold;
    float nms_threshold;
    uint32_t input_width;
    uint32_t input_height;
    uint32_t num_threads;
    bool use_gpu;
  };

  class YoloOnnx
  {
  public:
    YoloOnnx(const YoloOnnxConfig &config);
    ~YoloOnnx() = default;

    std::vector<Detection> detect(const cv::Mat &image);

  private:
    using input_buffer_t = std::vector<float>;
    using output_buffer_t = std::vector<float>;

    void preprocess(const cv::Mat &image, cv::Mat &blob);
    void run_onnx(const cv::Mat &blob, output_buffer_t &output);
    std::vector<Detection> postprocess(const output_buffer_t &output, const cv::Mat &original_image);

    YoloOnnxConfig config_;
    std::unique_ptr<Ort::Env> env_;
    std::unique_ptr<Ort::Session> session_;

    // Pre-allocated memory for performance
    cv::Mat padded_image_;
    cv::Mat rgb_image_;
    cv::Mat blob_;
    input_buffer_t input_tensor_values_;
    output_buffer_t output_buffer_;
    std::vector<cv::Rect> nms_boxes_;
    std::vector<float> nms_scores_;
    std::vector<int> nms_indices_;

    // Cached values
    Ort::MemoryInfo memory_info_;
    std::vector<int64_t> input_shape_;
    std::vector<const char *> input_names_;
    std::vector<const char *> output_names_;

    // Constants
    static constexpr size_t NUM_DETECTIONS = 8400;
  };

} // namespace yolo_onnx