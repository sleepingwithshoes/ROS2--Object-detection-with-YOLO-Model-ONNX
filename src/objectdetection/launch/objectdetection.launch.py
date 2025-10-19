import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    camera_shared_dir = get_package_share_directory('pkg_camera')
    camera_launch_path = os.path.join(camera_shared_dir, 'launch', 'camera.launch.py')

    yolodetect_shared_dir = get_package_share_directory('pkg_yolodetect')
    yolodetect_launch_path = os.path.join(yolodetect_shared_dir, 'launch', 'yolodetect.launch.py')

    show_shared_dir = get_package_share_directory('pkg_show')
    show_launch_path = os.path.join(show_shared_dir, 'launch', 'show.launch.py')

    return LaunchDescription([
        
        # Declare launch arguments for topic configuration
        DeclareLaunchArgument(
            'camera_output_topic',
            default_value='camera/image',
            description='Camera output topic name'
        ),
        
        DeclareLaunchArgument(
            'yolodetect_input_topic',
            default_value='camera/image',
            description='YOLODetect input topic name'
        ),
        
        DeclareLaunchArgument(
            'yolodetect_output_topic',
            default_value='yolodetect/image',
            description='YOLODetect output topic name'
        ),
        
        DeclareLaunchArgument(
            'show_input_topic',
            default_value='yolodetect/image',
            description='Show input topic name'
        ),
   
        # Launch camera node with configurable output topic
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch_path),
            launch_arguments={
                'camera_output_topic': LaunchConfiguration('camera_output_topic'),
            }.items()
        ),

        # Launch YOLODetect node with configurable input and output topics
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(yolodetect_launch_path),
            launch_arguments={
                'yolodetect_input_topic': LaunchConfiguration('yolodetect_input_topic'),
                'yolodetect_output_topic': LaunchConfiguration('yolodetect_output_topic'),
            }.items()
        ),

        # Launch show node with configurable input topic
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(show_launch_path),
            launch_arguments={
                'show_input_topic': LaunchConfiguration('show_input_topic'),
            }.items()
        ),
        
    ])