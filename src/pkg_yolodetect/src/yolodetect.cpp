#include <rclcpp/rclcpp.hpp>
#include "yolodetect_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<YoloDetectNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
