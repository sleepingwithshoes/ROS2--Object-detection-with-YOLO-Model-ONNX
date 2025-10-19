# yolo_msgs

Custom ROS2 message definitions for YOLO object detection results.

## Overview

This package provides message definitions for publishing structured YOLO detection data in ROS2. It enables nodes to communicate detected objects with detailed information including class IDs, names, confidence scores, and bounding box coordinates.

## Messages

### Detection.msg

Represents a single object detection result from YOLO.

**Fields:**
- `int32 class_id` - Class ID of the detected object (e.g., 0 for 'person', 1 for 'bicycle', etc.)
- `string class_name` - Human-readable class name (e.g., "person", "car", "dog")
- `float32 confidence` - Detection confidence score (range: 0.0 to 1.0)
- `int32 x` - Top-left x coordinate of the bounding box (in pixels)
- `int32 y` - Top-left y coordinate of the bounding box (in pixels)
- `int32 width` - Width of the bounding box (in pixels)
- `int32 height` - Height of the bounding box (in pixels)

**Example:**
```
class_id: 0
class_name: "person"
confidence: 0.95
x: 100
y: 150
width: 80
height: 200
```

### DetectionArray.msg

Contains an array of detections with timestamp and frame information.

**Fields:**
- `std_msgs/Header header` - Timestamp and frame ID for the detections
- `Detection[] detections` - Array of individual Detection messages

**Example:**
```
header:
  stamp:
    sec: 1234567890
    nanosec: 123456789
  frame_id: "camera_frame"
detections:
  - class_id: 0
    class_name: "person"
    confidence: 0.95
    x: 100
    y: 150
    width: 80
    height: 200
  - class_id: 2
    class_name: "car"
    confidence: 0.88
    x: 300
    y: 200
    width: 150
    height: 120
```

## Usage

### In C++

```cpp
#include "yolo_msgs/msg/detection_array.hpp"

// Create a publisher
auto publisher = this->create_publisher<yolo_msgs::msg::DetectionArray>(
    "yolodetect/detections", 10);

// Create and publish a message
auto msg = yolo_msgs::msg::DetectionArray();
msg.header.stamp = this->now();
msg.header.frame_id = "camera_frame";

yolo_msgs::msg::Detection det;
det.class_id = 0;
det.class_name = "person";
det.confidence = 0.95;
det.x = 100;
det.y = 150;
det.width = 80;
det.height = 200;

msg.detections.push_back(det);
publisher->publish(msg);
```

### In Python

```python
from yolo_msgs.msg import Detection, DetectionArray

# Create a publisher
self.publisher = self.create_publisher(
    DetectionArray, 
    'yolodetect/detections', 
    10)

# Create and publish a message
msg = DetectionArray()
msg.header.stamp = self.get_clock().now().to_msg()
msg.header.frame_id = 'camera_frame'

det = Detection()
det.class_id = 0
det.class_name = 'person'
det.confidence = 0.95
det.x = 100
det.y = 150
det.width = 80
det.height = 200

msg.detections.append(det)
self.publisher.publish(msg)
```

### Subscribing to Detections

```cpp
// C++
auto subscription = this->create_subscription<yolo_msgs::msg::DetectionArray>(
    "yolodetect/detections", 
    10,
    [this](const yolo_msgs::msg::DetectionArray::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received %zu detections", msg->detections.size());
        for (const auto& det : msg->detections) {
            RCLCPP_INFO(this->get_logger(), 
                "Detected %s with confidence %.2f at (%d, %d)", 
                det.class_name.c_str(), det.confidence, det.x, det.y);
        }
    });
```

```python
# Python
def detection_callback(self, msg):
    self.get_logger().info(f'Received {len(msg.detections)} detections')
    for det in msg.detections:
        self.get_logger().info(
            f'Detected {det.class_name} with confidence {det.confidence:.2f} '
            f'at ({det.x}, {det.y})')

self.subscription = self.create_subscription(
    DetectionArray,
    'yolodetect/detections',
    self.detection_callback,
    10)
```

## Command Line Tools

### View message definition
```bash
ros2 interface show yolo_msgs/msg/Detection
ros2 interface show yolo_msgs/msg/DetectionArray
```

### Echo detection messages
```bash
ros2 topic echo /yolodetect/detections
```

### View detection rate
```bash
ros2 topic hz /yolodetect/detections
```

### View message info
```bash
ros2 topic info /yolodetect/detections
```

## Building

This package is built using `colcon`:

```bash
cd /workspace
colcon build --packages-select yolo_msgs
source install/setup.bash
```

## Dependencies

- `ament_cmake` - Build system
- `rosidl_default_generators` - Message generation
- `std_msgs` - Standard message definitions (for Header)

## Related Packages

- `pkg_yolodetect` - YOLO detection node that publishes these messages
- `pkg_show` - Visualization node that may consume these messages

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

