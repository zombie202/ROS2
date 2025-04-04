from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'tower_height',
            default_value='4',
            description='height'
        ),

        Node(
            package='lab2',
            executable='tower',
            name='tower',
            parameters=[
                {'height': LaunchConfiguration('tower_height')}
            ]
        )
    ])