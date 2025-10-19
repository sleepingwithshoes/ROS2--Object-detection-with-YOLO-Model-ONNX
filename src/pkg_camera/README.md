# Package Camera

## Description

The `pkg_camera` package is a ROS 2 (Jazzy) package that provides camera capture and video playback capabilities with seamless ROS 2 integration. This versatile package can capture images from physical cameras or play video files, publishing frames as ROS 2 sensor_msgs/Image messages for downstream processing.

### Features

- **Dual Source Support**: Capture from physical cameras or play video files
- **Configurable Frame Rate**: Adjustable FPS for video playback and camera capture
- **Multiple Camera Support**: Select from available camera devices by ID
- **Video Looping**: Automatic restart when video reaches the end
- **ROS 2 Native**: Publishes standard sensor_msgs/Image messages
- **OpenCV Integration**: Leverages OpenCV VideoCapture for robust media handling
- **Configurable Topic Names**: Easy integration with downstream processing nodes

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
colcon build --symlink-install --packages-select pkg_camera
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

### Launch the Camera Publisher Node Standalone

To run only the camera publisher node:

```bash
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch pkg_camera camera.launch.py
```

## Launch File Configuration Parameters

The `camera.launch.py` file provides the following configurable parameters:

### Source Configuration

| Parameter | Type | Default Value | Description |
|-----------|------|---------------|-------------|
| `source` | string | `camera` | Source type: `camera` for physical camera or `video` for video file |
| `camera_id` | int | `0` | Camera device ID (used when source=camera). Typically 0 for built-in camera, 1+ for external cameras |
| `video_path` | string | `` (empty) | Absolute path to video file (required when source=video) |

### Output Configuration

| Parameter | Type | Default Value | Description |
|-----------|------|---------------|-------------|
| `camera_output_topic` | string | `camera/image` | Output topic name for publishing camera frames |
| `fps` | int | `30` | Frame rate in frames per second for publishing images |

### Example: Launch Configurations

**Use default camera (camera ID 0) at 30 FPS:**
```bash
ros2 launch pkg_camera camera.launch.py
```

**Use external camera (camera ID 1) at 60 FPS:**
```bash
ros2 launch pkg_camera camera.launch.py camera_id:=1 fps:=60
```

**Play a video file at 25 FPS:**
```bash
ros2 launch pkg_camera camera.launch.py \
    source:=video \
    video_path:=/path/to/your/video.mp4 \
    fps:=25
```

**Use camera with custom output topic:**
```bash
ros2 launch pkg_camera camera.launch.py \
    camera_output_topic:=/my_robot/camera/image \
    fps:=15
```

## Topics

### Subscribed Topics

None - this package publishes camera/video frames.

### Published Topics

- `camera/image` (sensor_msgs/Image): Camera frames in BGR8 format (configurable via `camera_output_topic` parameter)

## Usage Scenarios

### 1. Real-time Camera Capture

Capture frames from your system's default camera:

```bash
ros2 launch pkg_camera camera.launch.py source:=camera camera_id:=0 fps:=30
```

### 2. External USB Camera

Use an external USB camera (typically camera ID 1 or higher):

```bash
ros2 launch pkg_camera camera.launch.py source:=camera camera_id:=1 fps:=30
```

### 3. Video File Playback

Play a recorded video file with automatic looping:

```bash
ros2 launch pkg_camera camera.launch.py \
    source:=video \
    video_path:=/workspace/test_videos/sample.mp4 \
    fps:=25
```

### 4. Testing and Development

Use pre-recorded video for testing object detection without requiring a physical camera:

```bash
ros2 launch pkg_camera camera.launch.py \
    source:=video \
    video_path:=/workspace/test_data/test_video.avi \
    camera_output_topic:=camera/image \
    fps:=30
```

## Behavior Details

### Camera Mode (`source:=camera`)
- Opens the specified camera device using OpenCV VideoCapture
- Continuously captures frames at the specified FPS
- If frame capture fails, an error is logged

### Video Mode (`source:=video`)
- Loads the specified video file
- Plays frames at the configured FPS
- **Automatic looping**: When the video reaches the end, it automatically restarts from the beginning
- Useful for continuous testing and demonstrations

## Integration with Other Packages

The `pkg_camera` package is designed to work seamlessly with:

- **pkg_yolodetect**: Provides image input for object detection
- **pkg_show**: Displays raw camera feed
- Any ROS 2 package that subscribes to `sensor_msgs/Image` messages

### Example Pipeline

```
pkg_camera → pkg_yolodetect → pkg_show
(captures)   (detects)        (displays)
```

## Troubleshooting

### Camera Issues

- **"Failed to open camera or video source"**:
  - Verify camera is connected and not in use by another application
  - Check camera permissions: `ls -l /dev/video*`
  - Try different camera IDs: 0, 1, 2, etc.
  - Test camera with: `v4l2-ctl --list-devices` (Linux)

- **No image published**:
  - Check if topic is active: `ros2 topic list`
  - Monitor topic: `ros2 topic echo /camera/image`
  - Verify camera permissions in container/Docker environment

### Video File Issues

- **"Failed to open camera or video source" with video**:
  - Verify the video file path is correct and absolute
  - Check file format is supported by OpenCV (mp4, avi, mkv, etc.)
  - Ensure read permissions on the video file
  - Confirm video codec is available

- **Video plays but then stops**:
  - This shouldn't happen - videos loop automatically
  - Check logs for errors: `ros2 topic echo /rosout`

### Performance Issues

- **High CPU usage**:
  - Reduce FPS to a lower value
  - Use lower resolution video/camera settings
  
- **Delayed frames**:
  - Increase FPS parameter
  - Check system resources (CPU, memory)
  - Verify no other processes are using the camera

## Technical Details

- **Image Format**: BGR8 (8-bit color, standard OpenCV format)
- **Timer-based Publishing**: Uses ROS 2 wall timer for precise frame timing
- **Frame Timing**: Calculated as `1000ms / fps` for consistent publishing rate
- **Auto-restart**: Video files automatically loop to first frame upon completion

## Performance Notes

- Default 30 FPS provides good balance between smoothness and CPU usage
- Lower FPS (10-15) recommended for bandwidth-limited environments
- Higher FPS (60+) useful for fast-motion detection scenarios
- Timer precision depends on system scheduler and load

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

