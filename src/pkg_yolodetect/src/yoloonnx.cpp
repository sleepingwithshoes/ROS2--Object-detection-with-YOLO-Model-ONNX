#include "yoloonnx.hpp"
#include <thread>
namespace yolo_onnx
{

  YoloOnnx::YoloOnnx(const YoloOnnxConfig &config)
      : config_(std::move(config)), memory_info_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)), input_shape_{1, 3, static_cast<int64_t>(config_.input_height), static_cast<int64_t>(config_.input_width)}, input_names_{"images"}, output_names_{"output0"}
  {
    if (config_.class_names.empty())
    {
      throw std::runtime_error("Class names are empty.");
    }

    env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "YOLO-ONNX");
    Ort::SessionOptions session_options;

    if (config_.num_threads == 0)
    {
      // Adaptive Thread-Konfiguration
      int optimal_threads = []()
      {
        int cores = std::thread::hardware_concurrency();

        if (cores <= 2)
        {
          return std::min(1, cores);
        }
        if (cores <= 4)
        {
          return std::min(3, cores - 1);
        }
        return std::min(6, cores - 1);
      }();
      config_.num_threads = optimal_threads;
    }
    session_options.SetIntraOpNumThreads(config_.num_threads);
    session_options.SetInterOpNumThreads(1);
    session_options.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL); // mostly better for yolo
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    session_options.AddConfigEntry("session.intra_op.allow_spinning", "0");
    session_options.AddConfigEntry("session.enable_mem_pattern", "1");

    // Enable GPU if requested
    if (config_.use_gpu)
    {
      try
      {
        session_options.AppendExecutionProvider_CUDA({});
      }
      catch (const std::exception &e)
      {
        // Fallback to CPU if CUDA fails
      }
    }

    session_ = std::make_unique<Ort::Session>(*env_, config_.model_path.c_str(), session_options);

    // Pre-allocate memory buffers
    const size_t input_tensor_size = config_.input_width * config_.input_height * 3;
    const size_t output_tensor_size = NUM_DETECTIONS * (4 + static_cast<uint32_t>(config_.class_names.size()));

    input_tensor_values_.reserve(input_tensor_size);
    output_buffer_.reserve(output_tensor_size);
    nms_boxes_.reserve(NUM_DETECTIONS);
    nms_scores_.reserve(NUM_DETECTIONS);
    nms_indices_.reserve(NUM_DETECTIONS);

    // Pre-allocate OpenCV matrices
    padded_image_ = cv::Mat::zeros(config_.input_height, config_.input_width, CV_8UC3);
    rgb_image_.create(config_.input_height, config_.input_width, CV_8UC3);
    blob_.create(1, 3 * config_.input_height * config_.input_width, CV_32F);
  }

  std::vector<Detection> YoloOnnx::detect(const cv::Mat &image)
  {
    if (image.empty())
    {
      throw std::invalid_argument("Input image is empty");
    }

    preprocess(image, blob_);
    run_onnx(blob_, output_buffer_);
    return postprocess(output_buffer_, image);
  }

  void YoloOnnx::preprocess(const cv::Mat &image, cv::Mat &blob)
  {
    // Calculate scale and padding (same as before but cached)
    const float scale = std::min(
        static_cast<float>(config_.input_width) / image.cols,
        static_cast<float>(config_.input_height) / image.rows);

    const int new_w = static_cast<int>(image.cols * scale);
    const int new_h = static_cast<int>(image.rows * scale);
    const int pad_x = (config_.input_width - new_w) / 2;
    const int pad_y = (config_.input_height - new_h) / 2;

    // Reset padded image to zeros (faster than creating new)
    padded_image_.setTo(cv::Scalar::all(0));

    // Resize and copy to pre-allocated buffer
    cv::Mat resized_roi = padded_image_(cv::Rect(pad_x, pad_y, new_w, new_h));
    cv::resize(image, resized_roi, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);

    // Color conversion in-place
    cv::cvtColor(padded_image_, rgb_image_, cv::COLOR_BGR2RGB);

    // Manual normalization and NCHW conversion (faster than blobFromImage)
    float *dst_data = blob.ptr<float>();

    // Convert uint8 to float32 and normalize in one pass
    const size_t hw = config_.input_height * config_.input_width;
    const uint8_t *src_u8 = rgb_image_.data;

    // Optimized loop with pointer arithmetic
    for (size_t c = 0; c < 3; ++c)
    {
      for (size_t i = 0; i < hw; ++i)
      {
        dst_data[c * hw + i] = src_u8[i * 3 + c] / 255.0f;
      }
    }
  }

  void YoloOnnx::run_onnx(const cv::Mat &blob, output_buffer_t &output)
  {
    // Reuse pre-allocated tensor values
    const float *blob_data = blob.ptr<float>();
    const size_t tensor_size = blob.total();

    if (input_tensor_values_.size() != tensor_size)
    {
      input_tensor_values_.resize(tensor_size);
    }

    // Fast memory copy
    std::memcpy(input_tensor_values_.data(), blob_data, tensor_size * sizeof(float));

    // Create tensor without copying again
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info_,
        input_tensor_values_.data(),
        input_tensor_values_.size(),
        input_shape_.data(),
        input_shape_.size());

    // Run inference
    auto outputs = session_->Run(
        Ort::RunOptions{nullptr},
        input_names_.data(),
        &input_tensor,
        1,
        output_names_.data(),
        output_names_.size());

    // Copy output efficiently
    const float *output_data = outputs[0].GetTensorData<float>();
    const size_t output_size = outputs[0].GetTensorTypeAndShapeInfo().GetElementCount();

    if (output.size() != output_size)
    {
      output.resize(output_size);
    }

    std::memcpy(output.data(), output_data, output_size * sizeof(float));
  }

  std::vector<Detection> YoloOnnx::postprocess(const output_buffer_t &output,
                                               const cv::Mat &original_image)
  {
    // Clear reusable containers
    nms_boxes_.clear();
    nms_scores_.clear();
    nms_indices_.clear();

    // Pre-calculate transformation parameters
    const float scale = std::min(
        static_cast<float>(config_.input_width) / original_image.cols,
        static_cast<float>(config_.input_height) / original_image.rows);

    const int new_w = static_cast<int>(original_image.cols * scale);
    const int new_h = static_cast<int>(original_image.rows * scale);
    const int pad_x = (config_.input_width - new_w) / 2;
    const int pad_y = (config_.input_height - new_h) / 2;
    const float inv_scale = 1.0f / scale;

    // Pre-allocate detection storage
    std::vector<Detection> detections;
    detections.reserve(256); // Reasonable estimate

    // Optimized detection loop with early filtering
    const size_t num_classes = config_.class_names.size();
    for (size_t i = 0; i < NUM_DETECTIONS; ++i)
    {
      // Find the class with maximum score
      float max_score = 0.0f;
      int class_id = 0;

      for (size_t c = 0; c < num_classes; ++c)
      {
        const float class_score = output[(4 + c) * NUM_DETECTIONS + i];
        if (class_score > max_score)
        {
          max_score = class_score;
          class_id = static_cast<int>(c);
        }
      }

      // Early confidence filtering
      if (max_score < config_.conf_threshold)
        continue;

      // Extract coordinates only if we pass confidence threshold
      const float cx = output[i];
      const float cy = output[NUM_DETECTIONS + i];
      const float w = output[2 * NUM_DETECTIONS + i];
      const float h = output[3 * NUM_DETECTIONS + i];

      // Fast coordinate transformation
      const float x1 = ((cx - w * 0.5f) - pad_x) * inv_scale;
      const float y1 = ((cy - h * 0.5f) - pad_y) * inv_scale;
      const float x2 = ((cx + w * 0.5f) - pad_x) * inv_scale;
      const float y2 = ((cy + h * 0.5f) - pad_y) * inv_scale;

      // Clamp coordinates
      const int final_x1 = std::max(0, std::min(original_image.cols, static_cast<int>(x1)));
      const int final_y1 = std::max(0, std::min(original_image.rows, static_cast<int>(y1)));
      const int final_x2 = std::max(0, std::min(original_image.cols, static_cast<int>(x2)));
      const int final_y2 = std::max(0, std::min(original_image.rows, static_cast<int>(y2)));

      const int box_width = final_x2 - final_x1;
      const int box_height = final_y2 - final_y1;

      // Skip invalid boxes
      if (box_width <= 0 || box_height <= 0)
        continue;

      const cv::Rect box(final_x1, final_y1, box_width, box_height);

      // Use string_view or pre-computed class names for performance
      const std::string &class_name = (class_id < static_cast<int>(config_.class_names.size())) ? config_.class_names[class_id] : config_.class_names[0]; // fallback

      const cv::Scalar &box_color = (class_id < static_cast<int>(config_.box_colors.size())) ? config_.box_colors[class_id] : config_.box_colors[0]; // fallback

      detections.emplace_back(Detection{class_id, max_score, box, class_name, box_color});
      nms_boxes_.push_back(box);
      nms_scores_.push_back(max_score);
    }

    // Skip NMS if no detections
    if (detections.empty())
    {
      return detections;
    }

    // Fast NMS
    cv::dnn::NMSBoxes(nms_boxes_, nms_scores_, config_.conf_threshold, config_.nms_threshold, nms_indices_);

    // Build final results efficiently
    std::vector<Detection> final_detections;
    final_detections.reserve(nms_indices_.size());

    for (const int idx : nms_indices_)
    {
      final_detections.emplace_back(std::move(detections[idx]));
    }

    return final_detections;
  }

} // namespace yolo_onnx
