from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
            DeclareLaunchArgument(
                'show_input_topic',
                default_value='camera/image',
                description='Image topic name'
            ),

            Node(
                package='pkg_show',
                executable='show',
                name='show',
                parameters=[{
                    'show_input_topic': LaunchConfiguration('show_input_topic'),
                }]
            ),
    
    ])
