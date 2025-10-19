# Object Detection Launch Package

## Description

The `objectdetection` package is a ROS 2 (Jazzy) meta-package that provides launch configurations for the complete object detection pipeline. It orchestrates the camera, YOLO detection, and visualization nodes into a cohesive system.

## Features

- **Unified Launch Configuration**: Single launch file to start all nodes
- **Configurable Topics**: Launch arguments for customizing topic names
- **Modular Design**: Includes individual launch files from component packages
- **Easy Integration**: Simplified deployment of the complete detection system

## Dependencies

- ROS 2 Jazzy
- launch_ros
- pkg_camera
- pkg_yolodetect
- pkg_show
- yolo_msgs

## Build Instructions

Navigate to your ROS 2 workspace and build the package:

```bash
cd /workspace
source /opt/ros/jazzy/setup.zsh
colcon build --symlink-install
source install/setup.zsh
```

## Running the Complete System

Launch the entire object detection pipeline:

```bash
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch objectdetection objectdetection.launch.py
```

## Launch Arguments

The main launch file supports the following configurable parameters:

- `camera_output_topic` (default: `camera/image`) - Topic where camera publishes images
- `yolodetect_input_topic` (default: `camera/image`) - Topic for YOLO detection input
- `yolodetect_output_topic` (default: `yolodetect/image`) - Topic for annotated detection output
- `show_input_topic` (default: `yolodetect/image`) - Topic for visualization display

### Example with Custom Topics

```bash
ros2 launch objectdetection objectdetection.launch.py \
  camera_output_topic:=my_camera/image \
  yolodetect_input_topic:=my_camera/image \
  yolodetect_output_topic:=detections/image \
  show_input_topic:=detections/image
```

## Package Structure

```
objectdetection/
├── launch/
│   └── objectdetection.launch.py    # Main launch file
├── package.xml                       # Package metadata
└── setup.py                          # Python package setup
```

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

## Maintainer

Steffen Stautmeister (steffen.stautmeister@gmail.com)
