from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'turtle2',
            default_value='Leonardo',
            description='Name of second turtle'
        ),
        DeclareLaunchArgument(
            'turtle3',
            default_value='Donatello',
            description='Name of third turtle'
        ),

        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='simulation'
        ),
        Node(
            package='ninjaturtle',
            executable='movement',
            name='movement',
            parameters=[
                {'turtle2_name': LaunchConfiguration('turtle2')},
                {'turtle3_name': LaunchConfiguration('turtle3')}
            ]
        )
    ])