
#pragma once
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <string>

class ShowNode : public rclcpp::Node
{
public:
  ShowNode();
  ~ShowNode();

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};
