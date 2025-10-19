# DetectionArray.msg
# Array of YOLO detection results with header

# Header with timestamp and frame information
std_msgs/Header header

# Array of detected objects
Detection[] detections
