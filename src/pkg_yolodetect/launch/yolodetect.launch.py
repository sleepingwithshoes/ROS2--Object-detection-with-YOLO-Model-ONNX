from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([

            DeclareLaunchArgument(
                'yolodetect_input_topic',
                default_value='camera/image',
                description='Camera topic name'
            ),

            DeclareLaunchArgument(
                'yolodetect_output_topic',
                default_value='yolodetect/image',
                description='YoloDetect output topic name'
            ),
            
            DeclareLaunchArgument(
                'yolodetect_detections_topic',
                default_value='yolodetect/detections',
                description='YoloDetect detections topic name'
            ),

            DeclareLaunchArgument(
                'model_path',
                default_value='/workspace/src/pkg_yolodetect/models/yolo11n.onnx',
                description='Path to the YoloDetect model file'
            ),

            DeclareLaunchArgument(
                'class_path',
                default_value='/workspace/src/pkg_yolodetect/models/coco.names',
                description='Path to the YoloDetect class file'
            ),
            
            DeclareLaunchArgument(
                'color_path',
                default_value='/workspace/src/pkg_yolodetect/models/coco.colors',
                description='Path to the YoloDetect color file'
            ),

            DeclareLaunchArgument(
                'model_num_threads',
                default_value='0',
                description='Number of threads for the model inference (0 used adaptive threading)'
            ),
            DeclareLaunchArgument(
                'model_nms_threshold',
                default_value='0.4',
                description='Non-Maximum Suppression threshold for the model'
            ),
            DeclareLaunchArgument(
                'model_conf_threshold',
                default_value='0.5',
                description='Confidence threshold for the model'
            ),
            DeclareLaunchArgument(
                'model_use_gpu',
                default_value='false',
                description='Use GPU for inference'
            ),

            Node(
                package='pkg_yolodetect',
                executable='yolodetect',
                name='yolodetect',
                parameters=[{
                    'yolodetect_input_topic': LaunchConfiguration('yolodetect_input_topic'),
                    'yolodetect_output_topic': LaunchConfiguration('yolodetect_output_topic'),
                    'yolodetect_detections_topic': LaunchConfiguration('yolodetect_detections_topic'),
                    'model_path': LaunchConfiguration('model_path'),
                    'class_path': LaunchConfiguration('class_path'),
                    'color_path': LaunchConfiguration('color_path'),
                    'model_num_threads': LaunchConfiguration('model_num_threads'),
                    'model_nms_threshold': LaunchConfiguration('model_nms_threshold'),
                    'model_conf_threshold': LaunchConfiguration('model_conf_threshold'),
                    'model_use_gpu': LaunchConfiguration('model_use_gpu'),
                    }]
            ),

    ])
