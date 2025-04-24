import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Path to the xacro file
    xacro_file = os.path.join(
        get_package_share_directory('dobot'),
        'urdf',
        'dobot.xacro'
    )

    config = os.path.join(
        get_package_share_directory('dobot'),
        'config',
        'params.yaml'
    )

    forward_kin_node = Node(
        package='dobot',
        executable='forwardKin',
        name='ForwardKin',
        output='screen',
        parameters=[config]
    )

    inverse_kin_node = Node(
        package='dobot_IK',
        executable='IK',
        name='InverseKin',
        output='screen',
        parameters=[config]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', str(xacro_file)]),
                value_type=str)
            }],
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    return LaunchDescription([
        forward_kin_node,
        inverse_kin_node,
        robot_state_publisher_node,
        rviz_node
    ])