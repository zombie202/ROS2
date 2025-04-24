import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('dobot'),
        'config',
        'params.yaml'
    )

    visual_launch_file = os.path.join(
        get_package_share_directory('lab3'),
        'launch',
        'visual.py'
    )

    visual_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(visual_launch_file)
    )

    lab4_node = Node(
        package='lab4',
        executable='lab4',
        name='lab4',
        output='screen',
        parameters=[config]
    )

    return LaunchDescription([
        visual_launch,
        lab4_node
    ])