
#pragma once
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "yolo_msgs/msg/detection_array.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <string>
#include "yoloonnx.hpp"
#include "fps_tracker.hpp"

class YoloDetectNode : public rclcpp::Node
{
public:
  YoloDetectNode();
  ~YoloDetectNode() override;

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  cv::Mat process_image(const cv::Mat &image, std::vector<yolo_onnx::Detection> &detections);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Publisher<yolo_msgs::msg::DetectionArray>::SharedPtr detections_publisher_;
  cv::Mat image_;
  std::unique_ptr<yolo_onnx::YoloOnnx> yolo_onnx_;
  FPSTracker fps_tracker_;
};
