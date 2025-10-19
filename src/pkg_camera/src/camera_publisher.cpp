
#include <rclcpp/rclcpp.hpp>
#include "camera_publischer_node.hpp"

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
