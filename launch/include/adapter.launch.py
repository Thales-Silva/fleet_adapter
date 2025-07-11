#!/usr/bin/python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('fleet_config_path', default_value = ''),
        DeclareLaunchArgument('nav_graph_path', default_value = ''),
        Node(
            name='fleet_adapter',
            package='fleet_adapter',
            executable='fleet_adapter',
            output='screen',
            parameters=[{
                'fleet_config_path': LaunchConfiguration('fleet_config_path'),
                'nav_graph_path': LaunchConfiguration('nav_graph_path')
            }],
        )
    ])