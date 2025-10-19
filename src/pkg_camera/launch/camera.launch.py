from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
            # Declare launch arguments
            DeclareLaunchArgument(
                'source',
                default_value='video',
                description='Source type: camera or video'
            ),

            DeclareLaunchArgument(
                'camera_id',
                default_value='0',
                description='ID of the camera to use'
            ),

            DeclareLaunchArgument(
                'video_path',
                default_value='/workspace/resource/traffic.mp4',
                description='Path to the video file'
            ),

            DeclareLaunchArgument(
                'fps',
                default_value='30',
                description='Frames per second for video playback'
            ),
            DeclareLaunchArgument(
                'camera_output_topic',
                default_value='camera/image',
                description='Image topic name'
            ),

            Node(
                package='pkg_camera',
                executable='camera_publisher',
                name='camera_publisher',
                parameters=[{
                    'source': LaunchConfiguration('source'),
                    'camera_id': LaunchConfiguration('camera_id'),
                    'video_path': LaunchConfiguration('video_path'),
                    'fps': LaunchConfiguration('fps'),
                    'camera_output_topic': LaunchConfiguration('camera_output_topic'),
                }]
            ),
    ])
