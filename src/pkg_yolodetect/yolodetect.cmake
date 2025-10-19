# CMake configuration for yolodetect C++ executable

# Find required dependencies for C++ implementation
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(yolo_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Add ONNX Runtime
set(ONNXRUNTIME_INCLUDE_DIR "/usr/local/include")
set(ONNXRUNTIME_LIB_DIR "/usr/local/lib")
find_library(ONNXRUNTIME_LIBRARY
  NAMES onnxruntime
  PATHS ${ONNXRUNTIME_LIB_DIR}
  REQUIRED
)

# Create the C++ executable
add_executable(yolodetect
  src/yolodetect.cpp
  src/yolodetect_node.cpp
  src/yoloonnx.cpp
  src/fps_tracker.cpp
)

# Set include directories
target_include_directories(yolodetect PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${ONNXRUNTIME_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

# Set C++ standard
target_compile_features(yolodetect PUBLIC c_std_99 cxx_std_20)

# Link dependencies
ament_target_dependencies(
  yolodetect
  "rclcpp"
  "sensor_msgs"
  "cv_bridge"
  "yolo_msgs"
)

# Link ONNX Runtime library
target_link_libraries(yolodetect
  ${ONNXRUNTIME_LIBRARY}
  ${OpenCV_LIBS}
)

# Install the executable
install(TARGETS yolodetect
  DESTINATION lib/${PROJECT_NAME})
