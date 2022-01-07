#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    return LaunchDescription([
        
        Node(
            package='turtlesim',
            node_executable='turtlesim_node',
            node_name='turtlesim_node',
            output='screen'),
        
        Node(
            package='moving_turtle',
            node_executable='RVD_subscriber',
            node_name='RVD_subscriber',
            output='screen'),
        
        Node(
            package='moving_turtle',
            node_executable='RVD_publisher',
            node_name='RVD_pubblisher',
            output='screen'
        ),
        
    ])
