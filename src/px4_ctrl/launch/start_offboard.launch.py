import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'px4_ctrl'

    config_file_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'geometric_controller.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock from /clock'),

        Node(
            package=package_name,
            executable='offboard_control_node',
            name='offboard_geometric_controller',
            output='screen',
            parameters=[
                config_file_path,
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        )
    ])
