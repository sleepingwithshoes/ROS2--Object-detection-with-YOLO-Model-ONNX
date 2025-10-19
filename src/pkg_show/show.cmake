
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)


add_executable(show
  src/show.cpp
  src/show_node.cpp
)

ament_target_dependencies(
  show
  rclcpp
  sensor_msgs
  cv_bridge
)

target_link_libraries(
  show
  ${OpenCV_LIBS}
)
