

#include "camera_publischer_node.hpp"
#include <algorithm>

CameraPublisher::CameraPublisher()
    : Node("camera_publisher")
{
  this->declare_parameter<std::string>("source", "camera");
  this->declare_parameter<std::string>("video_path", "");
  this->declare_parameter<int>("camera_id", 0);
  this->declare_parameter<int>("fps", 30);
  this->declare_parameter<std::string>("camera_output_topic", "camera/image");

  source_ = this->get_parameter("source").as_string();
  video_path_ = this->get_parameter("video_path").as_string();
  camera_id_ = this->get_parameter("camera_id").as_int();
  fps_ = this->get_parameter("fps").as_int();

  if (source_ != "camera" && source_ != "video")
  {
    RCLCPP_ERROR(this->get_logger(), "Invalid source parameter: %s. Must be 'camera' or 'video'.", source_.c_str());
    throw std::runtime_error("Invalid source parameter");
  }

  if (source_ == "video")
  {
    if (video_path_.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "Video path parameter is not set!");
      throw std::runtime_error("Video path parameter is required for video source");
    }
    cap_.open(video_path_);
  }
  else
  {
    if (camera_id_ < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Invalid camera ID: %d", camera_id_);
      throw std::runtime_error("Invalid camera ID");
    }
    cap_.open(camera_id_);
  }
  fps_ = this->get_parameter("fps").as_int();
  int interval_ms = 1000 / fps_;

  if (!cap_.isOpened())
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to open camera or video source");
    throw std::runtime_error("Failed to open camera or video source");
  }

  publisher_ = this->create_publisher<sensor_msgs::msg::Image>(this->get_parameter("camera_output_topic").as_string(), 10);
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(interval_ms),
      std::bind(&CameraPublisher::publish_image, this));

  RCLCPP_INFO(this->get_logger(), "Camera publisher started");
}

CameraPublisher::~CameraPublisher()
{
  cap_.release();
}

void CameraPublisher::publish_image()
{
  cv::Mat image;
  cap_ >> image; // Capture a new image from the camera

  if (image.empty())
  {
    if (source_ == "video")
    {
      cap_.set(cv::CAP_PROP_POS_FRAMES, 0); // Reset to first frame
      cap_ >> image;                        // Read first frame again
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to capture image from camera");
      return;
    }
  }

  // Convert OpenCV image to ROS message
  sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(
                                               std_msgs::msg::Header(), "bgr8", image)
                                               .toImageMsg();

  publisher_->publish(*msg);
}
