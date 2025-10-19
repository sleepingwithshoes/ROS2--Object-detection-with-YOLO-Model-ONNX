
add_executable(camera_publisher
  src/camera_publisher.cpp
  src/camera_publisher_node.cpp
)

ament_target_dependencies(
  camera_publisher
  rclcpp
  sensor_msgs
  cv_bridge
)
target_link_libraries(
  camera_publisher
  ${OpenCV_LIBS}
)
