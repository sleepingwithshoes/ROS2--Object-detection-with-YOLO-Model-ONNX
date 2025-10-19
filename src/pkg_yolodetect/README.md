# Package yolodetect

## Description

The `pkg_yolodetect` package is a ROS 2 (Jazzy) package that provides real-time object detection using YOLO (You Only Look Once) models with ONNX Runtime. This package subscribes to camera image topics, performs object detection using a pre-trained YOLO model, and publishes both annotated images with bounding boxes and structured detection data as custom ROS2 messages.

### Features

- **YOLO Model Support**: Compatible with YOLO11 ONNX models
- **Real-time Object Detection**: Efficient inference with configurable threading
- **GPU Acceleration**: Optional GPU support for faster inference
- **Customizable Detection Parameters**: Configurable confidence and NMS thresholds
- **ROS 2 Integration**: Seamless integration with ROS 2 sensor_msgs/Image topics
- **FPS Tracking**: Built-in performance monitoring
- **COCO Dataset Support**: Pre-configured for COCO dataset classes and colors

## Dependencies

- ROS 2 Jazzy
- rclcpp
- sensor_msgs
- cv_bridge
- yolo_msgs (custom message package)
- OpenCV
- ONNX Runtime

## Build Instructions

### Build the Package

Navigate to your ROS 2 workspace and build the package using colcon:

```bash
cd /workspace
source /opt/ros/jazzy/setup.zsh
colcon build --symlink-install --packages-select yolo_msgs pkg_yolodetect
source install/setup.zsh
```

**Note:** The `yolo_msgs` package must be built before `pkg_yolodetect` as it provides the custom message definitions.

### Build All Packages (Alternative)

To build all packages in the workspace:

```bash
cd /workspace
source /opt/ros/jazzy/setup.zsh
colcon build --symlink-install
source install/setup.zsh
```

## Running the Package

### Launch the YoloDetect Node Standalone

To run only the YOLO detection node:

```bash
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch pkg_yolodetect yolodetect.launch.py
```

## Launch File Configuration Parameters

The `yolodetect.launch.py` file provides the following configurable parameters:

### Topic Configuration

| Parameter | Type | Default Value | Description |
|-----------|------|---------------|-------------|
| `yolodetect_input_topic` | string | `camera/image` | Input topic name for receiving camera images |
| `yolodetect_output_topic` | string | `yolodetect/image` | Output topic name for publishing annotated images with detections |
| `yolodetect_detections_topic` | string | `yolodetect/detections` | Output topic name for publishing structured detection data |

### Model Configuration

| Parameter | Type | Default Value | Description |
|-----------|------|---------------|-------------|
| `model_path` | string | `/workspace/src/pkg_yolodetect/models/yolo11n.onnx` | Path to the YOLO ONNX model file |
| `class_path` | string | `/workspace/src/pkg_yolodetect/models/coco.names` | Path to the class names file (COCO dataset) |
| `color_path` | string | `/workspace/src/pkg_yolodetect/models/coco.colors` | Path to the color mapping file for visualization |

### Inference Configuration

| Parameter | Type | Default Value | Description |
|-----------|------|---------------|-------------|
| `model_num_threads` | int | `0` | Number of threads for model inference (0 = adaptive threading) |
| `model_nms_threshold` | float | `0.4` | Non-Maximum Suppression threshold for filtering overlapping detections (range: 0.0-1.0) |
| `model_conf_threshold` | float | `0.5` | Confidence threshold for filtering low-confidence detections (range: 0.0-1.0) |
| `model_use_gpu` | bool | `false` | Enable GPU acceleration for inference (requires CUDA-enabled ONNX Runtime) |

### Example: Custom Launch Configuration

To launch the node with custom parameters:

```bash
ros2 launch pkg_yolodetect yolodetect.launch.py \
    yolodetect_input_topic:=/my_camera/image \
    yolodetect_output_topic:=/my_detections/image \
    yolodetect_detections_topic:=/my_detections/data \
    model_conf_threshold:=0.6 \
    model_nms_threshold:=0.45 \
    model_use_gpu:=true \
    model_num_threads:=4
```

## Models

The package includes the following pre-configured models in the `models/` directory:

- **yolo11n.onnx**: YOLO11 nano model (lightweight, fast inference)
- **coco.names**: COCO dataset class names (80 classes)
- **coco.colors**: Color mapping for visualization

To use a different YOLO model, update the `model_path` parameter to point to your custom ONNX model file.

## Topics

### Subscribed Topics

- `camera/image` (sensor_msgs/Image): Input images from camera

### Published Topics

- `yolodetect/image` (sensor_msgs/Image): Output images with bounding boxes and labels
- `yolodetect/detections` (yolo_msgs/DetectionArray): Structured detection data containing:
  - Header with timestamp and frame_id
  - Array of detections with class_id, class_name, confidence, and bounding box (x, y, width, height)

## Working with Detection Messages

### Viewing Detection Data

To view the structured detection messages in real-time:

```bash
# Echo all detection data
ros2 topic echo /yolodetect/detections

# View detection message rate
ros2 topic hz /yolodetect/detections

# View detection message info
ros2 topic info /yolodetect/detections
```

### Subscribing to Detections in Code

**C++ Example:**

```cpp
#include "yolo_msgs/msg/detection_array.hpp"

auto subscription = this->create_subscription<yolo_msgs::msg::DetectionArray>(
    "yolodetect/detections", 
    10,
    [this](const yolo_msgs::msg::DetectionArray::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received %zu detections", msg->detections.size());
        for (const auto& det : msg->detections) {
            RCLCPP_INFO(this->get_logger(), 
                "Detected %s with confidence %.2f at (%d, %d, %d, %d)", 
                det.class_name.c_str(), det.confidence, 
                det.x, det.y, det.width, det.height);
        }
    });
```

**Python Example:**

```python
from yolo_msgs.msg import DetectionArray

def detection_callback(self, msg):
    self.get_logger().info(f'Received {len(msg.detections)} detections')
    for det in msg.detections:
        self.get_logger().info(
            f'Detected {det.class_name} (ID: {det.class_id}) '
            f'with confidence {det.confidence:.2f} '
            f'at bbox: ({det.x}, {det.y}, {det.width}, {det.height})')

self.subscription = self.create_subscription(
    DetectionArray,
    'yolodetect/detections',
    self.detection_callback,
    10)
```

## Performance Tips

1. **GPU Acceleration**: Set `model_use_gpu:=true` if you have a CUDA-capable GPU for significant performance improvements
2. **Threading**: Adjust `model_num_threads` based on your CPU cores (0 for auto-detection)
3. **Threshold Tuning**: 
   - Lower `model_conf_threshold` to detect more objects (may increase false positives)
   - Increase `model_nms_threshold` to keep more overlapping detections
4. **Model Selection**: Use smaller models (e.g., yolo11n) for real-time performance on edge devices

## Troubleshooting

- **No output images**: Check that the input topic is publishing and matches the configured `yolodetect_input_topic`
- **Low FPS**: Consider reducing image resolution, using GPU acceleration, or switching to a lighter model
- **Missing detections**: Lower the `model_conf_threshold` parameter
- **Too many overlapping boxes**: Increase the `model_nms_threshold` parameter

## License

MIT License

Copyright (c) 2025 Steffen Stautmeister

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.


