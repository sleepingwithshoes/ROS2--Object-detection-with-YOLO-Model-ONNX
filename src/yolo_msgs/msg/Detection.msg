# Detection.msg
# Single object detection result from YOLO

# Class ID of the detected object
int32 class_id

# Class name of the detected object
string class_name

# Confidence score (0.0 to 1.0)
float32 confidence

# Bounding box coordinates
int32 x      # Top-left x coordinate
int32 y      # Top-left y coordinate
int32 width  # Width of bounding box
int32 height # Height of bounding box
