
#include <rclcpp/rclcpp.hpp>
#include "show_node.hpp"

int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ShowNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
