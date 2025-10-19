#include "show_node.hpp"
#include <functional>

ShowNode::ShowNode()
    : Node("show")
{
  this->declare_parameter<std::string>("show_input_topic", "camera/image");

  using namespace std::placeholders;

  subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      this->get_parameter("show_input_topic").as_string(), 10, std::bind(&ShowNode::image_callback, this, _1));

  RCLCPP_INFO(this->get_logger(), "Show node initialized.");
}

ShowNode::~ShowNode()
{
  cv::destroyAllWindows();
}

void ShowNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  // Convert ROS message to OpenCV image
  cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;

  cv::imshow("Camera Image", image);
  cv::waitKey(1);
}
