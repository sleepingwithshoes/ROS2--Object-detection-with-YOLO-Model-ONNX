# Package Show

## Description

The `pkg_show` package is a ROS 2 (Jazzy) package that provides real-time image visualization using OpenCV. This lightweight package subscribes to ROS 2 image topics and displays the images in an OpenCV window, making it ideal for visualizing camera feeds, processed images, or detection results from other nodes.

### Features

- **Real-time Image Display**: Efficient visualization of ROS 2 sensor_msgs/Image topics
- **OpenCV Integration**: Uses OpenCV's imshow for responsive image rendering
- **Configurable Topic**: Easy topic name configuration via launch parameters
- **Minimal Resource Usage**: Lightweight node designed for efficient operation
- **ROS 2 Native**: Full integration with ROS 2 message passing system

## Dependencies

- ROS 2 Jazzy
- rclcpp
- sensor_msgs
- cv_bridge
- OpenCV

## Build Instructions

### Build the Package

Navigate to your ROS 2 workspace and build the package using colcon:

```bash
cd /workspace
source /opt/ros/jazzy/setup.zsh
colcon build --symlink-install --packages-select pkg_show
source install/setup.zsh
```

### Build All Packages (Alternative)

To build all packages in the workspace:

```bash
cd /workspace
source /opt/ros/jazzy/setup.zsh
colcon build --symlink-install
source install/setup.zsh
```

## Running the Package

### Launch the Show Node Standalone

To run only the image visualization node:

```bash
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch pkg_show show.launch.py
```

## Launch File Configuration Parameters

The `show.launch.py` file provides the following configurable parameter:

### Topic Configuration

| Parameter | Type | Default Value | Description |
|-----------|------|---------------|-------------|
| `show_input_topic` | string | `camera/image` | Input topic name for receiving images to display |

### Example: Custom Launch Configuration

To launch the node with a custom input topic:

```bash
ros2 launch pkg_show show.launch.py show_input_topic:=/my_camera/image
```

To visualize YOLO detection results:

```bash
ros2 launch pkg_show show.launch.py show_input_topic:=/yolodetect/image
```

## Topics

### Subscribed Topics

- `camera/image` (sensor_msgs/Image): Input images to display (configurable via `show_input_topic` parameter)

### Published Topics

None - this package only subscribes to image topics for visualization purposes.

## Usage Scenarios

1. **Camera Feed Visualization**: Display raw camera images
   ```bash
   ros2 launch pkg_show show.launch.py show_input_topic:=/camera/image
   ```

2. **Detection Results Visualization**: Display images with object detection bounding boxes
   ```bash
   ros2 launch pkg_show show.launch.py show_input_topic:=/yolodetect/image
   ```

3. **Pipeline Debugging**: Visualize intermediate processing results at any stage of an image processing pipeline

## Window Controls

- The image is displayed in an OpenCV window titled "Camera Image"
- Press `ESC` or close the window to stop the node
- Window can be resized and moved as needed

## Integration with Other Packages

The `pkg_show` package is designed to work seamlessly with:

- **pkg_camera**: Visualize raw camera feeds
- **pkg_yolodetect**: Display object detection results with bounding boxes
- Any ROS 2 package publishing `sensor_msgs/Image` messages

## Troubleshooting

- **No window appears**: 
  - Check that the input topic is being published: `ros2 topic list`
  - Verify the topic name matches: `ros2 topic echo <topic_name>`
  - Ensure you have X11 forwarding enabled if running in a remote/container environment
  
- **Window freezes**: 
  - Check that the image messages are being published continuously
  - Verify sufficient system resources are available
  
- **Black/empty window**: 
  - Confirm the image format is correct (should be BGR8)
  - Check image dimensions are valid

## Performance Notes

- The node uses `cv::waitKey(1)` for efficient GUI processing
- Minimal CPU overhead for standard image sizes
- No buffering - displays latest received image only

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


