
#pragma once
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <string>

class CameraPublisher : public rclcpp::Node {
  public:
  CameraPublisher();
  ~CameraPublisher();

  private:
  void publish_image();
  void resizeAndCropImage(cv::Mat& image, int target_width, int target_height);

  std::string source_;
  std::string video_path_;
  int camera_id_;
  int fps_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  cv::VideoCapture cap_;
};
