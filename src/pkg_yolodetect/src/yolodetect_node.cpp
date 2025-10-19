#include "yolodetect_node.hpp"
#include <functional>
#include <fstream>
#include <chrono>
#include <algorithm>
#include <ranges>

#include "fps_tracker.hpp"

YoloDetectNode::YoloDetectNode()
    : Node("yolodetect"), fps_tracker_(30)
{
  this->declare_parameter<std::string>("yolodetect_input_topic", "camera/image");
  this->declare_parameter<std::string>("yolodetect_output_topic", "yolodetect/image");
  this->declare_parameter<std::string>("yolodetect_detections_topic", "yolodetect/detections");
  this->declare_parameter<std::string>("model_path", "");
  this->declare_parameter<std::string>("class_path", "");
  this->declare_parameter<std::string>("color_path", "");
  this->declare_parameter<int>("model_num_threads", 1);
  this->declare_parameter<double>("model_nms_threshold", 0.45f);
  this->declare_parameter<double>("model_conf_threshold", 0.5f);
  this->declare_parameter<int>("model_input_width", 640);
  this->declare_parameter<int>("model_input_height", 640);
  this->declare_parameter<bool>("model_use_gpu", false);

  using namespace std::placeholders;

  subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      this->get_parameter("yolodetect_input_topic").as_string(), 10, std::bind(&YoloDetectNode::image_callback, this, _1));

  publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
      this->get_parameter("yolodetect_output_topic").as_string(), 10);

  detections_publisher_ = this->create_publisher<yolo_msgs::msg::DetectionArray>(
      this->get_parameter("yolodetect_detections_topic").as_string(), 10);

  if (this->get_parameter("class_path").as_string().empty())
  {
    RCLCPP_ERROR(this->get_logger(), "Class path is empty.");
    throw std::runtime_error("Class path is empty.");
  }
  // Load class names from file
  std::vector<std::string> class_names;
  std::ifstream class_file(this->get_parameter("class_path").as_string());
  std::string line;
  while (std::getline(class_file, line))
  {
    class_names.push_back(line);
  }

  if (class_names.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "Class names are empty.");
    throw std::runtime_error("Class names are empty.");
  }
  RCLCPP_INFO(this->get_logger(), "Loaded %d class names.", static_cast<int>(class_names.size()));

  if (this->get_parameter("color_path").as_string().empty())
  {
    RCLCPP_ERROR(this->get_logger(), "Color path is empty. Using default colors.");
    throw std::runtime_error("Color path is empty.");
  }
  // Load colors from file
  std::ifstream color_file(this->get_parameter("color_path").as_string());
  std::string color_line;
  std::vector<cv::Scalar> box_colors;
  while (std::getline(color_file, color_line))
  {
    int r, g, b;
    if (sscanf(color_line.c_str(), "%d,%d,%d", &r, &g, &b) == 3)
    {
      box_colors.emplace_back(b, g, r); // OpenCV uses BGR format
    }
  }
  RCLCPP_INFO(this->get_logger(), "Loaded %d box colors.", static_cast<int>(box_colors.size()));

  yolo_onnx::YoloOnnxConfig config{
      .model_path = this->get_parameter("model_path").as_string(),
      .class_names = class_names,
      .box_colors = box_colors,
      .conf_threshold = static_cast<float>(this->get_parameter("model_conf_threshold").as_double()),
      .nms_threshold = static_cast<float>(this->get_parameter("model_nms_threshold").as_double()),
      .input_width = static_cast<uint32_t>(this->get_parameter("model_input_width").as_int()),
      .input_height = static_cast<uint32_t>(this->get_parameter("model_input_height").as_int()),
      .num_threads = static_cast<uint32_t>(this->get_parameter("model_num_threads").as_int()),
      .use_gpu = this->get_parameter("model_use_gpu").as_bool()};

  yolo_onnx_ = std::make_unique<yolo_onnx::YoloOnnx>(config);

  // print config
  RCLCPP_INFO(this->get_logger(), "YoloOnnxConfig:");
  RCLCPP_INFO(this->get_logger(), "  model_path: %s", config.model_path.c_str());
  RCLCPP_INFO(this->get_logger(), "  class_names size: %d", static_cast<int>(config.class_names.size()));
  RCLCPP_INFO(this->get_logger(), "  box_colors size: %d", static_cast<int>(config.box_colors.size()));
  RCLCPP_INFO(this->get_logger(), "  conf_threshold: %f", config.conf_threshold);
  RCLCPP_INFO(this->get_logger(), "  nms_threshold: %f", config.nms_threshold);
  RCLCPP_INFO(this->get_logger(), "  input_width: %d", config.input_width);
  RCLCPP_INFO(this->get_logger(), "  input_height: %d", config.input_height);
  RCLCPP_INFO(this->get_logger(), "  num_threads: %d", config.num_threads);
  RCLCPP_INFO(this->get_logger(), "  use_gpu: %s", config.use_gpu ? "true" : "false");

  RCLCPP_INFO(this->get_logger(), "YoloDetectNode has been started.");
}

YoloDetectNode::~YoloDetectNode() = default;

void YoloDetectNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  // Convert ROS message to OpenCV image and process it
  std::vector<yolo_onnx::Detection> detections;
  image_ = process_image(cv_bridge::toCvShare(msg, "bgr8")->image, detections);

  // Convert processed OpenCV image back to ROS message and publish it
  auto image_msg = cv_bridge::CvImage(msg->header, "bgr8", image_).toImageMsg();
  publisher_->publish(*image_msg);

  if (detections.empty())
  {
    return;
  }

  // Publish the detection array
  auto detections_msg = yolo_msgs::msg::DetectionArray();
  detections_msg.header = msg->header;

  for (const auto &detection : detections)
  {
    yolo_msgs::msg::Detection det;
    det.class_id = detection.class_id;
    det.class_name = detection.class_name;
    det.confidence = detection.confidence;
    det.x = detection.box.x;
    det.y = detection.box.y;
    det.width = detection.box.width;
    det.height = detection.box.height;
    detections_msg.detections.push_back(det);
  }

  detections_publisher_->publish(detections_msg);
}

cv::Mat YoloDetectNode::process_image(const cv::Mat &image, std::vector<yolo_onnx::Detection> &detections)
{
  auto start_time = std::chrono::high_resolution_clock::now();

  auto results = yolo_onnx_->detect(image);

  // Store latest detections for publishing
  detections = results;

  if (results.empty())
  {
    return image;
  }

  // Draw detection results on the original image
  for (const auto &result : results)
  {
    cv::rectangle(image, result.box, result.box_color, 2);
    cv::putText(image, result.class_name + " " + std::to_string(static_cast<int>(result.confidence * 100)) + "%",
                cv::Point(result.box.x, result.box.y - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, result.box_color, 2);
  }

  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
  // Update FPS tracker
  std::ignore = fps_tracker_.update(duration);

  // draw fps on image
  cv::putText(image, "FPS: " + std::to_string(static_cast<int>(fps_tracker_.getAverage())), cv::Point(10, 30),
              cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 2);

  return image;
}
